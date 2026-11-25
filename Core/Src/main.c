/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Led_Error.h"
#include "display_curtis.h" /* button_* cho EXTI; LCD đã bỏ */
#include "wdt.h"
#include "clock.h"
#include "extern_rom.h"
#include "dwt_delay.h"
#include "realtime.h"
#include "network.h"
#include "swconfig.h"
#include "sim.h"
#include "system_update.h"
#include "extern_flash.h"
#include "boat_log.h"
#include "modbus_slave_comp.h"
#include "modbus_rtu_interface.h"
#include "gnss_uart_dma.h"
#include "imd.h"
#include "Can_Slider.h"
#include "string.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;

/* Giữ symbol cho Display/ST7565 (LCD đã bỏ, không init SPI2) */
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USART1 đang dùng cho YPC Modbus — debug_print tắt tạm */
void debug_print(const char* message)
{
	(void)message;
}

/*
 * Ngay sau reset chân còn Hi-Z → driver relay dễ tạch.
 * Kéo Ctr_Out + EN về OUTPUT LOW trước mọi delay/init dài.
 * (Giả định active-high: LOW = nhả relay)
 */
static void Early_RelaySafeInit(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOCEN;
	(void)RCC->AHB1ENR;

	/* PE2..PE6 = output PP, write 0 */
	GPIOE->MODER = (GPIOE->MODER & ~0x3FF0u) | 0x1550u;
	GPIOE->OTYPER &= ~(0x7Cu);
	GPIOE->PUPDR &= ~0x3FF0u;
	GPIOE->BSRR = (0x7Cu << 16);

	/* PC10, PC11, PC13 = output PP, write 0 */
	GPIOC->MODER = (GPIOC->MODER & ~((3u << 20) | (3u << 22) | (3u << 26)))
	             |  ((1u << 20) | (1u << 22) | (1u << 26));
	GPIOC->OTYPER &= ~((1u << 10) | (1u << 11) | (1u << 13));
	GPIOC->PUPDR &= ~((3u << 20) | (3u << 22) | (3u << 26));
	GPIOC->BSRR = (((1u << 10) | (1u << 11) | (1u << 13)) << 16);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  Early_RelaySafeInit();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* GPIO relay trước — không delay khi chân còn nổi */
  MX_GPIO_Init();
  Early_RelaySafeInit();
  SystemUpdate_Init();
  HAL_Delay(200);
  CyDelay_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* MX_GPIO_Init đã gọi ở SysInit */
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* IWDG chỉ bật sau khi init xong — tránh reset giữa chừng / khi debug */
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  debug_print("\r\n[BOOT] after I2C1\r\n");
  if (hi2c1.Instance != NULL && hi2c1.State == HAL_I2C_STATE_READY) {
    IMD_Init(&hi2c1); /* ADS1115 PB8/PB9 — bỏ qua nếu không có HW */
  }
  // Khởi tạo Modbus RTU Slave
  ModbusRTU_UartInit();
  ModbusSlaveComp_Init();
  ClockTimer_Init();
  ExRom_Init();
  Flash_Init();
  RealTime_Init();
  BoatEventLog_Init();
  Network_Init();
  Sim_Init();
  /* CAN1 PD0/PD1: BMS ; CAN2 PB5/PB6: Driver
   * HAL_CAN_Start cần bus recessive — thiếu/hỏng transceiver sẽ timeout.
   * Không Error_Handler để vẫn boot (LED/Modbus) khi đang bring-up HW. */
  BMS_Jikong_Init(&hcan1);
  Can_Slider_Init(&hcan2);
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    debug_print("[BOOT] CAN1 Start FAIL (check PD0/PD1 transceiver)\r\n");
  } else {
    debug_print("[BOOT] CAN1 Start OK\r\n");
  }
  if (HAL_CAN_Start(&hcan2) != HAL_OK) {
    debug_print("[BOOT] CAN2 Start FAIL (check PB5/PB6 transceiver)\r\n");
  } else {
    debug_print("[BOOT] CAN2 Start OK\r\n");
  }
  VCU_StateInit();
#ifndef DISABLE_WDT
  MX_IWDG_Init();
#else
  debug_print("[BOOT] IWDG disabled (bring-up)\r\n");
#endif
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) {
    debug_print("[BOOT] WARN: previous reset was IWDG\r\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET) {
    debug_print("[BOOT] previous reset was SOFTWARE\r\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET) {
    debug_print("[BOOT] previous reset was PIN/POR\r\n");
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
  debug_print("[BOOT] Controller_Slider OK\r\n");

// GNSS UART RX DMA-to-IDLE 
#ifdef GNSS_UART_HANDLE
  (void)GNSS_UART_DMA_Start(GNSS_UART_HANDLE);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifndef DISABLE_WDT
	Wdt_Reset();
#endif
	// Parse GPS
#ifdef GNSS_UART_HANDLE
    GNSS_UART_DMA_Task();
    BoatGPS_Task();
#endif
    ClockTimer_Run();
    Realtime_Run();
    Sim_Work();
    Network_Supervision();
    IMD_Task();
    VCU_StateTask();
#if BOARD_HOLD_CTR_OUT_OFF
    /* Ép 6 Ctr_Out OFF sau VCU — test CAN/RS485 không kéo relay */
    HAL_GPIO_WritePin(CTR_OUT_1_GPIO_Port, CTR_OUT_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CTR_OUT_2_GPIO_Port, CTR_OUT_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CTR_OUT_3_GPIO_Port, CTR_OUT_3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CTR_OUT_4_GPIO_Port, CTR_OUT_4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CTR_OUT_5_GPIO_Port, CTR_OUT_5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CTR_OUT_6_GPIO_Port, CTR_OUT_6_Pin, GPIO_PIN_RESET);
#endif
    led_process();
    SystemUpdate_Run();
    BoatEventUpdate();
    ModbusSlaveComp_Run();

    /* Heartbeat ~2Hz: STATUS_LED PB1 (không dùng Toggle — tránh bị IMD/relay đè) */
    {
      static uint32_t hb_tick = 0;
      static uint8_t hb_on = 0;
      if ((HAL_GetTick() - hb_tick) >= 250u) {
        hb_tick = HAL_GetTick();
        hb_on ^= 1u;
        HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin,
                           hb_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
        if (hb_on) {
          char line[96];
          (void)snprintf(line, sizeof(line),
            "[HB] t=%lu drv=%u modbus=%u\r\n",
            (unsigned long)hb_tick,
            Can_Slider_IsAlive(1000u) ? 1u : 0u,
            ModbusSlaveComp_IsOnline(3000u) ? 1u : 0u);
          debug_print(line);
        }
      }
    }
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function (Driver: PB5 RX / PB6 TX)
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 12;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function — IMD ADS1115 (PB8 SCL / PB9 SDA)
  * Không Error_Handler: board bring-up có thể chưa hàn ADS1115.
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    debug_print("[BOOT] I2C1 init FAIL — skip IMD\r\n");
    hi2c1.Instance = NULL;
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 467;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600; /* YPC Modbus RTU */
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|SCK_RTC_Pin|SDA_RTC_Pin|RST_RTC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5|SOFT_I2C_SCL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SOFT_I2C_SDA_Pin, GPIO_PIN_RESET);

  /* PB0 free, PB1 = STATUS_LED */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|STATUS_LED_Pin|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level — SD_CS/SD_RST + PWR_KEY */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|PWR_KEY_Pin, GPIO_PIN_RESET);

  /* Ctr_Out_2..6: PE6 PE5 PE4 PE3 PE2 — tất cả OUTPUT (không để PE5 input) */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* PE1 input (nếu dùng) */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* Ctr_Out_1=PC13, IMD EN_POS=PC10, EN_NEG=PC11; giữ PC4/PC5 nếu còn dùng */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_13, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* STATUS_LED PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|STATUS_LED_Pin|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* SD_CS PD8, SD_RST PD9 (không còn LCD) */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* Buttons PD11/12/13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PWR_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PWR_KEY_GPIO_Port, &GPIO_InitStruct);

  /* EEPROM soft-I2C: SDA=PA8, SCL=PC9 */
  GPIO_InitStruct.Pin = SOFT_I2C_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SOFT_I2C_SDA_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SOFT_I2C_SCL_Pin;
  HAL_GPIO_Init(SOFT_I2C_SCL_GPIO_Port, &GPIO_InitStruct);

  /* DS1302: SCK/SDA/RST trên PE11/PE14/PE15 */
  GPIO_InitStruct.Pin = SCK_RTC_Pin|SDA_RTC_Pin|RST_RTC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* W25Q soft-SPI pins init trong SoftSpiFlash_InitPins() */

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// GPIO interrupt callback function
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  uint32_t current_time = HAL_GetTick();

  //debug_print("*** GPIO INTERRUPT TRIGGERED ***\r\n");

  switch (GPIO_Pin) {
      case GPIO_PIN_13: // DOWN button
          if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_RESET) {
              // Button pressed (LOW)
              button_down_time = current_time;
              button_down_pressed = 1;
          } else {
              // Button released (HIGH)
              button_down_pressed = 0;
          }
          break;

      case GPIO_PIN_12: // ENTER button
          if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_RESET) {
              // Button pressed (LOW)
              button_enter_time = current_time;
              button_enter_pressed = 1;
          } else {
              // Button released (HIGH)
              button_enter_pressed = 0;
          }
          break;

      case GPIO_PIN_11: // UP button
          if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) == GPIO_PIN_RESET) {
              // Button pressed (LOW)
              button_up_time = current_time;
              button_up_pressed = 1;
          } else {
              // Button released (HIGH)
              button_up_pressed = 0;
          }
          break;

      default:
          break;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
