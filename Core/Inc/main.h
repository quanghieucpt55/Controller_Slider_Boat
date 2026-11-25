/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* Legacy: Display/ST7565 còn reference — LCD đã bỏ, không init SPI2 */
extern SPI_HandleTypeDef hspi2;
extern CAN_HandleTypeDef hcan1; /* BMS: PD0=RX, PD1=TX */
extern CAN_HandleTypeDef hcan2; /* Driver: PB5=RX, PB6=TX */
extern UART_HandleTypeDef huart1; /* RS485 YPC (test): PA9=TX, PA10=RX */
extern UART_HandleTypeDef huart2; /* USART2 PD5/PD6 — tạm không dùng YPC */
extern UART_HandleTypeDef huart6; /* SIM: PC6/PC7 */
extern UART_HandleTypeDef huart3; /* GPS: PB10/PB11 */
extern I2C_HandleTypeDef hi2c1;   /* IMD ADS1115: PB8=SCL, PB9=SDA */
extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim2;

#define uart_sim &huart6
#define htimer &htim2

// GPS Handlde
//#define GNSS_UART_HANDLE (&huart3)
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern void debug_print(const char* message);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* W25Q16 — soft SPI trên PE (schematic CS/CLK/MISO/MOSI_FLASH) */
#define SPI_FLASH_CS_Pin GPIO_PIN_7
#define SPI_FLASH_CS_GPIO_Port GPIOE
#define SPI_FLASH_SCK_Pin GPIO_PIN_8
#define SPI_FLASH_SCK_GPIO_Port GPIOE
#define SPI_FLASH_MISO_Pin GPIO_PIN_9
#define SPI_FLASH_MISO_GPIO_Port GPIOE
#define SPI_FLASH_MOSI_Pin GPIO_PIN_10
#define SPI_FLASH_MOSI_GPIO_Port GPIOE
#define GPS_Tx_Pin GPIO_PIN_10
#define GPS_Tx_GPIO_Port GPIOB
#define GPS_Rx_Pin GPIO_PIN_11
#define GPS_Rx_GPIO_Port GPIOB
#define PWR_KEY_Pin GPIO_PIN_15
#define PWR_KEY_GPIO_Port GPIOD
/* EEPROM 24C04 — net SCL_DS1307 / SDA_DS1307 (schematic) */
#define SOFT_I2C_SCL_Pin GPIO_PIN_9
#define SOFT_I2C_SCL_GPIO_Port GPIOC
#define SOFT_I2C_SDA_Pin GPIO_PIN_8
#define SOFT_I2C_SDA_GPIO_Port GPIOA
/* DS1302 RTC — net RST_RTC / SDA_RTC / SCK_RTC */
#define RST_RTC_Pin GPIO_PIN_15
#define RST_RTC_GPIO_Port GPIOE
#define SDA_RTC_Pin GPIO_PIN_14
#define SDA_RTC_GPIO_Port GPIOE
#define SCK_RTC_Pin GPIO_PIN_11
#define SCK_RTC_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
/*
 * Bật khi đã lắp phần cứng ổ khóa nhiều nấc.
 * Cần bổ sung 2 tín hiệu đọc:
 * - IGN_LEVEL1_Pin / IGN_LEVEL1_GPIO_Port
 * - IGN_LEVEL2_Pin / IGN_LEVEL2_GPIO_Port
 */
/* #define VCU_MULTI_LEVEL_IGNITION */

/* Ctr_Out (schematic): PC13, PE6, PE5, PE4, PE3, PE2 */
#define CTR_OUT_1_Pin GPIO_PIN_13
#define CTR_OUT_1_GPIO_Port GPIOC
#define CTR_OUT_2_Pin GPIO_PIN_6
#define CTR_OUT_2_GPIO_Port GPIOE
#define CTR_OUT_3_Pin GPIO_PIN_5
#define CTR_OUT_3_GPIO_Port GPIOE
#define CTR_OUT_4_Pin GPIO_PIN_4
#define CTR_OUT_4_GPIO_Port GPIOE
#define CTR_OUT_5_Pin GPIO_PIN_3
#define CTR_OUT_5_GPIO_Port GPIOE
#define CTR_OUT_6_Pin GPIO_PIN_2
#define CTR_OUT_6_GPIO_Port GPIOE

/* VCU load relays — map vào Ctr_Out */
#define RELAY_DISABLE_MOTOR_Pin CTR_OUT_6_Pin       /* PE2 */
#define RELAY_DISABLE_MOTOR_GPIO_Port CTR_OUT_6_GPIO_Port
#define RELAY_CONTACTOR_Pin CTR_OUT_5_Pin           /* PE3 */
#define RELAY_CONTACTOR_GPIO_Port CTR_OUT_5_GPIO_Port

#define STATUS_LED_Pin GPIO_PIN_1
#define STATUS_LED_GPIO_Port GPIOB

/* IMD EN_POS/EN_NEG — schematic PC10/PC11 (KHÔNG dùng Ctr_Out) */
#define RELAY_POS_Pin GPIO_PIN_10
#define RELAY_POS_GPIO_Port GPIOC
#define RELAY_NEG_Pin GPIO_PIN_11
#define RELAY_NEG_GPIO_Port GPIOC

/* Board đang bring-up: EN_POS/EN_NEG bị X trên schematic — không đóng relay đo */
#ifndef IMD_DRIVE_EN_RELAYS
#define IMD_DRIVE_EN_RELAYS 0
#endif

/* Giữ 6 Ctr_Out OFF (chống VCU/Modbus kéo relay khi test CAN/RS485) */
#ifndef BOARD_HOLD_CTR_OUT_OFF
#define BOARD_HOLD_CTR_OUT_OFF 1
#endif

/* Bring-up: tắt IWDG — Sim/MQTT block lâu có thể reset → chân nổi → relay tạch */
#if BOARD_HOLD_CTR_OUT_OFF
#ifndef DISABLE_WDT
#define DISABLE_WDT
#endif
#endif

/*
 * KSI: PE5 là Ctr_Out_3 (OUTPUT) — không được đọc làm input.
 * Tạm tắt đọc GPIO; bật lại khi có net KSI đúng trên schematic.
 */
/* #define KSI_GPIO_Port GPIOx */
/* #define KSI_Pin GPIO_PIN_x */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
