#include "global.h"
#include "sim.h"
#include "jikong_can.h"
#include "Can_Slider.h"
#include "swconfig.h"
#include "realtime.h"
#include "boat_frame.h"
#include "boat_log.h"
#include "imd.h"
#include "VCU_State.h"
#include "string.h"

/* ==============================================================
 *  Frame Ping 350B — khung SmartCharger, ánh xạ thuyền.
 *  Tài liệu chốt: BOAT_MQTT_FRAME_MAP.md
 *  Decoder đối chiếu: DeviceSchargeLog_DataHandle.cs (không sửa)
 * ============================================================== */
#define SCHARGE_FRAME_SIZE      350u
#define SCHARGE_CHECKSUM_POS    349u

/* Giới hạn Bms.CurrentData = int16/100 → ±327.67 A */
#define BMS_CURRENT_RAW_MAX     32767
#define BMS_CURRENT_RAW_MIN   (-32768)

uint8_t FrameCheck(uint8_t *buf, uint32_t size)
{
    if (buf == NULL || size == 0u)
        return 0u;
    uint32_t cs = 0;
    for (uint32_t i = 0; i < size; i++)
        cs += buf[i];
    return ((cs & 0xFF) == 0xFF) ? 1u : 0u;
}

#define SCALE_001(v)       ((int32_t)((v) * 100.0f))
#define SCALE_CURR_001A(a) ((uint16_t)((a) * 100.0f))
#define SCALE_VOLT_01V(v)  ((int32_t)((v) * 10.0f))
#define SCALE_AH_TO_MAH(a) ((uint32_t)((a) * 1000.0f))
#define SCALE_AH_01(a)     ((uint32_t)((a) * 10.0f))

static inline void put_u8(uint8_t *buf, uint32_t pos, uint8_t v)
{
    buf[pos] = v;
}

static inline void put_i16(uint8_t *buf, uint32_t pos, int16_t v)
{
    buf[pos]     = (uint8_t)(v & 0xFF);
    buf[pos + 1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void put_u16(uint8_t *buf, uint32_t pos, uint16_t v)
{
    buf[pos]     = (uint8_t)(v & 0xFF);
    buf[pos + 1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void put_u32(uint8_t *buf, uint32_t pos, uint32_t v)
{
    buf[pos]     = (uint8_t)(v & 0xFF);
    buf[pos + 1] = (uint8_t)((v >> 8) & 0xFF);
    buf[pos + 2] = (uint8_t)((v >> 16) & 0xFF);
    buf[pos + 3] = (uint8_t)((v >> 24) & 0xFF);
}

static inline void put_i32(uint8_t *buf, uint32_t pos, int32_t v)
{
    put_u32(buf, pos, (uint32_t)v);
}

/* Backend: ConvertLongitudeCoordinates — độ + phút thập phân
 * encoded = deg*1e6 + round(minutes*1e4), minutes = frac_deg*60
 */
static uint32_t encode_gps_degmin(int32_t microdeg_signed)
{
    uint32_t abs_udeg;
    if (microdeg_signed < 0)
        abs_udeg = (uint32_t)(-microdeg_signed);
    else
        abs_udeg = (uint32_t)microdeg_signed;

    uint32_t deg = abs_udeg / 10000000u;
    uint32_t frac_udeg = abs_udeg % 10000000u; /* phần thập phân × 1e7 */
    /* minutes × 1e4 = frac_udeg * 60 / 1000  (vì 1e7/60 → phút, *1e4) */
    uint32_t minutes_x10000 = (frac_udeg * 60u + 500u) / 1000u;
    if (minutes_x10000 >= 600000u) { /* làm tròn tràn phút */
        deg += 1u;
        minutes_x10000 = 0u;
    }
    return deg * 1000000u + minutes_x10000;
}

static int16_t clamp_i16(int32_t v)
{
    if (v > 32767)
        return (int16_t)32767;
    if (v < -32768)
        return (int16_t)-32768;
    return (int16_t)v;
}

static uint16_t temp_x10_u16(int32_t temp_c)
{
    int32_t v = temp_c * 10;
    if (v < 0)
        v = 0; /* Backend BytesToIntDouble(size=2) đọc UInt16 — âm sẽ sai */
    if (v > 65535)
        v = 65535;
    return (uint16_t)v;
}

static uint8_t count_ntc(void)
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < 5u; i++) {
        if ((bms.allTemp.mask & (1u << i)) != 0u)
            n++;
    }
    return (n > 0u) ? n : 5u;
}

static uint16_t build_bms_warn16(void)
{
    uint16_t w = 0;
    if (bms.almInfo.soc_low        > 0) w |= (1u << 0);
    if (bms.almInfo.temp_high      > 0) w |= (1u << 4);
    if (bms.almInfo.chg_oc         > 0) w |= (1u << 5);
    if (bms.almInfo.dchg_oc        > 0) w |= (1u << 6);
    if (bms.almInfo.delta_over     > 0) w |= (1u << 7);
    if (bms.almInfo.temp_high      > 0) w |= (1u << 8);
    if (bms.almInfo.temp_low       > 0) w |= (1u << 9);
    if (bms.almInfo.cell_overvolt  > 0) w |= (1u << 10);
    if (bms.almInfo.cell_undervolt > 0) w |= (1u << 11);
    if (bms.almInfo.comm_fault     > 0) w |= (1u << 12);
    return w;
}

static uint16_t build_bms_status16(void)
{
    uint16_t s = 0;
    if (bms.swSta.chgMOS)  s |= (1u << 0);
    if (bms.swSta.dchgMOS) s |= (1u << 1);
    if (bms.swSta.balance) s |= (1u << 2);
    if (bms.swSta.heat)    s |= (1u << 3);
    if (bms.swSta.chgPlug) s |= (1u << 4);
    if (bms.swSta.acc)     s |= (1u << 5);
    return s;
}

/* 0=N, 1=D, 2=R, 3=xung đột tiến/lùi — từ cờ driver, không phải chiều trục */
static uint8_t encode_gear_driver(void)
{
    const uint8_t fwd = can_slider.slider_1.vehicle_mode.forward ? 1u : 0u;
    const uint8_t rev = can_slider.slider_1.vehicle_mode.reverse ? 1u : 0u;
    if (fwd && rev)
        return 3u;
    if (fwd)
        return 1u;
    if (rev)
        return 2u;
    return 0u;
}

static uint32_t build_warn32(void)
{
    uint32_t w = 0;
    if (bms.almInfo.soc_low        > 0) w |= (1u << 0);
    if (bms.almInfo.temp_high      > 0) w |= (1u << 4);
    if (bms.almInfo.chg_oc         > 0) w |= (1u << 5);
    if (bms.almInfo.dchg_oc        > 0) w |= (1u << 6);
    if (bms.almInfo.delta_over     > 0) w |= (1u << 7);
    if (bms.almInfo.cell_overvolt  > 0) w |= (1u << 10);
    if (bms.almInfo.cell_undervolt > 0) w |= (1u << 11);
    if (bms.almInfo.comm_fault     > 0) w |= (1u << 18);
    if (bms.bmsErrInfo.mos_overtemp)     w |= (1u << 19);
    if (bms.bmsErrInfo.chg_mos_fault)    w |= (1u << 20);
    if (bms.bmsErrInfo.dchg_mos_fault)   w |= (1u << 21);
    if (bms.bmsErrInfo.chg_short)        w |= (1u << 22);
    if (bms.bmsErrInfo.dchg_short)       w |= (1u << 23);
    if (bms.bmsErrInfo.pack_overvolt)    w |= (1u << 24);
    if (bms.bmsErrInfo.pack_undervolt)   w |= (1u << 25);
    if (bms.bmsErrInfo.cur_sensor_fault) w |= (1u << 26);
    /* Bit SmartCharge mượn cho thuyền — backend: Warning.* */
    if (IMD_IsPosFault() || IMD_IsNegFault()) w |= (1u << 14); /* → ChargeInDischargePort */
    if (IMD_IsBothFault())                    w |= (1u << 15); /* → IsChargingPortShort */
    if (IMD_IsMeasureFault())                 w |= (1u << 16); /* → IsDischargingPortShort */
    if (!Can_Slider_IsAlive(1000u))           w |= (1u << 17); /* → IsLowSocAlarm (mượn: mất CAN) */
    if (can_slider.effective_error_code)      w |= (1u << 30);
    return w;
}

uint32_t Boat_Frame_Ping_Complete(uint8_t *buf, uint32_t size)
{
    if (size < SCHARGE_FRAME_SIZE)
        return 0;

    memset(buf, 0x00, SCHARGE_FRAME_SIZE);

    if (!ValidTime(&currentTime))
        return 0;

    const bool gps_ok = (gpsData.state == 3u);
    const bool driver_alive = Can_Slider_IsAlive(1000u);

    /* ---- 0–12: RTC + FW + ID ---- */
    put_u8(buf,  0, currentTime.sec);
    put_u8(buf,  1, currentTime.minute);
    put_u8(buf,  2, currentTime.hour);
    put_u8(buf,  3, currentTime.day);
    put_u8(buf,  4, currentTime.month);
    put_u8(buf,  5, currentTime.year);
    put_u8(buf,  6, VERSION_NUMBER_0);
    put_u8(buf,  7, VERSION_NUMBER_1);
    put_u8(buf,  8, VERSION_NUMBER_2);
    put_u32(buf, 9, (uint32_t)ID_DEVICE);

    /* ---- 13–41: GPS ----
     * NorthSouth/EastWest: backend ByteToBoolean → true=N / true=E
     * Lat/Lng: dạng độ–phút (ConvertLongitudeCoordinates)
     * Mất GPS: không giữ tọa độ cũ
     */
    put_u8(buf, 13, gpsData.state);
    if (gps_ok) {
        put_u8(buf, 14, (gpsData.latitude  >= 0) ? 1u : 0u); /* N=1, S=0 */
        put_u8(buf, 15, (gpsData.longitude >= 0) ? 1u : 0u); /* E=1, W=0 */
        put_u8(buf, 16, currentTime.sec);
        put_u8(buf, 17, currentTime.minute);
        put_u8(buf, 18, currentTime.hour);
        put_u8(buf, 19, currentTime.day);
        put_u8(buf, 20, currentTime.month);
        put_u8(buf, 21, currentTime.year);
        put_u32(buf, 22, encode_gps_degmin(gpsData.latitude));
        put_u32(buf, 26, encode_gps_degmin(gpsData.longitude));
        put_u32(buf, 30, (uint32_t)gpsData.sog);  /* km/h × 100 */
        put_u32(buf, 34, (uint32_t)gpsData.cog);  /* deg × 100 */
        put_u32(buf, 38, (uint32_t)gpsData.sog);
    } else {
        put_u8(buf, 14, 1u);
        put_u8(buf, 15, 1u);
        put_u8(buf, 16, currentTime.sec);
        put_u8(buf, 17, currentTime.minute);
        put_u8(buf, 18, currentTime.hour);
        put_u8(buf, 19, currentTime.day);
        put_u8(buf, 20, currentTime.month);
        put_u8(buf, 21, currentTime.year);
        put_u32(buf, 22, 0u);
        put_u32(buf, 26, 0u);
        put_u32(buf, 30, 0u);
        put_u32(buf, 34, 0u);
        put_u32(buf, 38, 0u);
    }

    /* ---- 42–75: BMS ---- */
    put_i16(buf, 42, clamp_i16(SCALE_001(bms.batt1.voltage_V)));
    {
        /* Bms.CurrentData = int16/100 — giới hạn ±327.67 A (không đủ ±350 A) */
        int32_t cur_raw = SCALE_001(bms.batt1.current_A);
        if (cur_raw > BMS_CURRENT_RAW_MAX)
            cur_raw = BMS_CURRENT_RAW_MAX;
        if (cur_raw < BMS_CURRENT_RAW_MIN)
            cur_raw = BMS_CURRENT_RAW_MIN;
        put_i16(buf, 44, (int16_t)cur_raw);
    }
    put_u8(buf, 46, bms.batt1.soc_percent);
    put_u8(buf, 47, count_ntc());
    put_u16(buf, 48, bms.batt2.cycle_count);
    put_u16(buf, 50, (uint16_t)bms.cellArray.cell_count);
    put_u16(buf, 52, (uint16_t)(uint8_t)bms.cellTemp.avg_temp_C);
    put_u16(buf, 54, (uint16_t)(uint8_t)bms.allTemp.temp[0]);
    put_u16(buf, 56, (uint16_t)(uint8_t)bms.allTemp.temp[1]);
    put_u32(buf, 58, SCALE_AH_TO_MAH(bms.batt2.cap_full_Ah));
    put_u32(buf, 62, SCALE_AH_TO_MAH(bms.batt2.cap_remain_Ah));
    put_u16(buf, 66, build_bms_warn16());
    put_u16(buf, 68, build_bms_status16());
    {
        uint16_t bms_conn = (bms.batt1.voltage_V > 0.0f) ? 1u : 0u;
        put_u16(buf, 70, bms_conn);
        put_u8(buf, 196, bms.swSta.chgPlug ? 1u : 0u);
        put_u8(buf, 197, (uint8_t)bms_conn);
    }
    put_i16(buf, 72, clamp_i16(SCALE_001(bms.chgInfo.chg_volt_V)));
    put_i16(buf, 74, (int16_t)bms.chgInfo.chg_curr_A);

    /* ---- 76–195: 40 cell ---- */
    for (uint8_t i = 0; i < 40u; i++) {
        uint32_t pos = 76u + (uint32_t)i * 3u;
        if (i < bms.cellArray.cell_count && i < 30u) {
            put_i16(buf, pos, (int16_t)bms.cellArray.cell_mV[i]);
            put_u8(buf, pos + 2, 1u);
        } else {
            put_i16(buf, pos, 0);
            put_u8(buf, pos + 2, 0u);
        }
    }

    /* ---- 198–216: BlockAir (backend đọc số) ---- */
    put_u8(buf, 198, driver_alive ? 1u : 0u);                 /* IsSlaveConnected */
    put_u16(buf, 199, can_slider.slider_1.motor_rpm);         /* RmpSpeed */
    put_u16(buf, 201, (uint16_t)(can_slider.effective_error_code & 0xFFFFu));
    put_u16(buf, 203, (uint16_t)((can_slider.effective_error_code >> 16) & 0xFFFFu));
    put_u16(buf, 205, (uint16_t)(can_slider.effective_warning_code & 0xFFFFu));
    put_u16(buf, 207, (uint16_t)((can_slider.effective_warning_code >> 16) & 0xFFFFu));
    put_u16(buf, 209, (uint16_t)(can_slider.slider_2.battery_voltage + 0.5f));
    {
        float motor_curr = can_slider.slider_2.dc_current;
        if (motor_curr < 0.0f)
            motor_curr = -motor_curr;
        put_u16(buf, 211, SCALE_CURR_001A(motor_curr)); /* OutputCurrent — dòng motor, KHÔNG phải BMS */
    }
    /* AmbTemp / ModuleTemp: backend ÷10 */
    put_u16(buf, 213, temp_x10_u16((int32_t)can_slider.slider_1.motor_temp));
    put_u16(buf, 215, temp_x10_u16((int32_t)can_slider.slider_1.controller_temp));

    /* ---- 217–224: charge/discharge volt (cờ sạc/xả nếu >5V) ---- */
    put_i32(buf, 217, SCALE_VOLT_01V(bms.chgInfo.chg_volt_V));
    put_i32(buf, 221, SCALE_VOLT_01V(bms.batt1.voltage_V));

    /* ---- 225–232: TempAlu1/2 Value÷10; IsOpen = bool (KHÔNG dùng cho IMD) ---- */
    put_u16(buf, 225, temp_x10_u16((int32_t)can_slider.slider_1.motor_temp));
    put_u8(buf, 227, 0u); /* IsSensorOpenCircuit */
    put_u8(buf, 228, 0u); /* padding backend idx++ */
    put_u16(buf, 229, temp_x10_u16((int32_t)can_slider.slider_1.controller_temp));
    put_u8(buf, 231, 0u);
    put_u8(buf, 232, 0u);

    /* ---- 233–236: TempColdWater.Value÷10 = IMD+ kΩ; IsOpen KHÔNG chứa số ---- */
    {
        uint32_t pos_ohm = IMD_GetRisoPosOhm();
        uint16_t pos_k = (uint16_t)((pos_ohm + 500u) / 1000u);
        put_u16(buf, 233, (uint16_t)(pos_k * 10u)); /* Value = kΩ */
        put_u8(buf, 235, 0u);
        put_u8(buf, 236, 0u);
    }

    /* ---- 237–240: TempHotWater.Value÷10 = gear; IsOpen = mất CAN / số không hợp lệ ---- */
    {
        uint8_t gear = encode_gear_driver();
        put_u16(buf, 237, (uint16_t)(gear * 10u)); /* → JSON 0/1/2/3 */
        put_u8(buf, 239, driver_alive ? 0u : 1u);  /* true = số không hợp lệ */
        put_u8(buf, 240, 0u);
    }

    /* ---- 241–245: relay + sóng + IMD measure + cờ ---- */
    {
        uint16_t relay = 0;
        if (bms.swSta.chgMOS)                 relay |= (1u << 0); /* Charge */
        if (vcu_ctx.inputs.contactor_request) relay |= (1u << 1); /* PreCharge = lệnh contactor */
        if (bms.swSta.dchgMOS)                relay |= (1u << 2); /* Discharge */
        if (vcu_ctx.outputs.contactor_on)     relay |= (1u << 3); /* PreDischarge = tiếp điểm thực */
        if (vcu_ctx.inputs.KSI)               relay |= (1u << 4); /* AirCondition = chìa khóa */
        if (!vcu_ctx.outputs.disable_motor)   relay |= (1u << 5); /* OutSpeak = motor enable */
        put_u16(buf, 241, relay);
    }
    put_u8(buf, 243, Sim_SignalQuanlityPercent());
    put_u8(buf, 244, IMD_IsMeasureFault() ? 1u : 0u); /* SdCardCode — mượn: lỗi đo IMD */
    {
        uint8_t flags = 0;
        if (IMD_IsPosFault() || IMD_IsNegFault()) flags |= (1u << 0);
        if (IMD_IsBothFault())                    flags |= (1u << 1);
        if (IMD_IsMeasureFault())                 flags |= (1u << 2);
        if (gps_ok)                               flags |= (1u << 3);
        if (driver_alive)                         flags |= (1u << 4);
        if (bms.swSta.chgPlug)                    flags |= (1u << 5);
        if (encode_gear_driver() != 0u)           flags |= (1u << 6);
        if (bms.swSta.balance)                    flags |= (1u << 7);
        put_u8(buf, 245, flags); /* CardState */
    }

    put_u32(buf, 246, SCALE_AH_01(bms.batt2.cycle_cap_Ah));
    put_i16(buf, 250, (int16_t)bms.cellVolt.max_cell_mV);
    put_i16(buf, 252, (int16_t)bms.cellVolt.min_cell_mV);
    put_u32(buf, 254, build_warn32());
    put_u32(buf, 258, bms.info.runtime_s);

    /* ---- 262–273: TempTank.Value÷10 = IMD− kΩ; TempHot2Water.Value÷10 = ga thô ---- */
    {
        uint32_t neg_ohm = IMD_GetRisoNegOhm();
        uint16_t neg_k = (uint16_t)((neg_ohm + 500u) / 1000u);
        put_u16(buf, 262, (uint16_t)(neg_k * 10u)); /* TempTank.Value = kΩ */
        put_u8(buf, 264, 0u);
        put_u8(buf, 265, 0u);
    }
    {
        uint32_t thr = (uint32_t)can_slider.slider_2.thr_Val * 10u;
        if (thr > 65535u)
            thr = 65535u;
        put_u16(buf, 266, (uint16_t)thr); /* TempHot2Water.Value = thr_Val */
        put_u8(buf, 268, 0u);
        put_u8(buf, 269, 0u);
    }
    put_u32(buf, 270, 0u); /* Ipump — không dùng (backend UInt32/1000, mất dấu) */

    /* ---- 274–348: backend hiện tại KHÔNG đọc (trừ IsBatteryLock nếu FW≥1.0.17) ----
     * Không dùng làm đường truyền chính. Giữ 0.
     */

    /* ---- 349: checksum ---- */
    {
        uint32_t cs_sum = 0;
        for (uint32_t i = 0; i < SCHARGE_CHECKSUM_POS; i++)
            cs_sum += buf[i];
        buf[SCHARGE_CHECKSUM_POS] = (uint8_t)(0xFF - (cs_sum & 0xFF));
    }

    return SCHARGE_FRAME_SIZE;
}

uint32_t Boat_Frame_Event(uint8_t *buf, uint32_t size)
{
    if (size < 8u)
        return 0;

    boat_event_log_t item_log;
    if (BoatEventLog_Dequeue((uint8_t *)&item_log) == 0)
        return 0;

    memcpy(buf, item_log.date_time, 6u);
    buf[6] = 0u;
    buf[7] = (uint8_t)item_log.event;
    return 8u;
}
