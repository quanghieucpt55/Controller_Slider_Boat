#include "soft_spi_flash.h"
#include "main.h"
#include "dwt_delay.h"

#define FLASH_CLK_HI()  HAL_GPIO_WritePin(SPI_FLASH_SCK_GPIO_Port, SPI_FLASH_SCK_Pin, GPIO_PIN_SET)
#define FLASH_CLK_LO()  HAL_GPIO_WritePin(SPI_FLASH_SCK_GPIO_Port, SPI_FLASH_SCK_Pin, GPIO_PIN_RESET)
#define FLASH_MOSI_HI() HAL_GPIO_WritePin(SPI_FLASH_MOSI_GPIO_Port, SPI_FLASH_MOSI_Pin, GPIO_PIN_SET)
#define FLASH_MOSI_LO() HAL_GPIO_WritePin(SPI_FLASH_MOSI_GPIO_Port, SPI_FLASH_MOSI_Pin, GPIO_PIN_RESET)
#define FLASH_MISO()    HAL_GPIO_ReadPin(SPI_FLASH_MISO_GPIO_Port, SPI_FLASH_MISO_Pin)

static void soft_spi_delay(void)
{
	CyDelayUs(1);
}

void SoftSpiFlash_InitPins(void)
{
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_GPIOE_CLK_ENABLE();

	FLASH_CLK_LO();
	FLASH_MOSI_LO();
	HAL_GPIO_WritePin(SPI_FLASH_CS_GPIO_Port, SPI_FLASH_CS_Pin, GPIO_PIN_SET);

	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	gpio.Pin = SPI_FLASH_CS_Pin | SPI_FLASH_SCK_Pin | SPI_FLASH_MOSI_Pin;
	HAL_GPIO_Init(GPIOE, &gpio);

	gpio.Pin = SPI_FLASH_MISO_Pin;
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOE, &gpio);
}

static uint8_t soft_spi_xfer_byte(uint8_t tx)
{
	uint8_t rx = 0;
	for (int i = 7; i >= 0; i--) {
		if (tx & (1u << i)) {
			FLASH_MOSI_HI();
		} else {
			FLASH_MOSI_LO();
		}
		soft_spi_delay();
		FLASH_CLK_HI();
		soft_spi_delay();
		if (FLASH_MISO() == GPIO_PIN_SET) {
			rx |= (uint8_t)(1u << i);
		}
		FLASH_CLK_LO();
	}
	return rx;
}

HAL_StatusTypeDef SoftSpiFlash_TransmitReceive(uint8_t *tx, uint8_t *rx, size_t size)
{
	for (size_t i = 0; i < size; i++) {
		uint8_t out = (tx != NULL) ? tx[i] : 0xFFu;
		uint8_t in = soft_spi_xfer_byte(out);
		if (rx != NULL) {
			rx[i] = in;
		}
	}
	return HAL_OK;
}

HAL_StatusTypeDef SoftSpiFlash_Transmit(uint8_t *tx, size_t size)
{
	return SoftSpiFlash_TransmitReceive(tx, NULL, size);
}

HAL_StatusTypeDef SoftSpiFlash_Receive(uint8_t *rx, size_t size)
{
	return SoftSpiFlash_TransmitReceive(NULL, rx, size);
}
