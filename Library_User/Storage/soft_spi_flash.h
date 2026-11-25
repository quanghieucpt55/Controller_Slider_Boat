#ifndef SOFT_SPI_FLASH_H_
#define SOFT_SPI_FLASH_H_

#include <stdint.h>
#include <stddef.h>
#include "stm32f4xx_hal.h"

/* W25Q trên PE7–PE10 (schematic không có AF SPI HW) */
void SoftSpiFlash_InitPins(void);
HAL_StatusTypeDef SoftSpiFlash_TransmitReceive(uint8_t *tx, uint8_t *rx, size_t size);
HAL_StatusTypeDef SoftSpiFlash_Transmit(uint8_t *tx, size_t size);
HAL_StatusTypeDef SoftSpiFlash_Receive(uint8_t *rx, size_t size);

#endif /* SOFT_SPI_FLASH_H_ */
