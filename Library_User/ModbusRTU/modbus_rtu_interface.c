/*
 * modbus_rtu_interface.c
 * RS485 YPC (test): USART1 — PA9 TX / PA10 RX
 */

#include <stdint.h>
#include "modbus_rtu_interface.h"
#include "modbus_slave_comp.h"
#include "main.h"
#include "stm32f4xx_hal.h"

typedef struct
{
  uint8_t * buffer;
  uint16_t bufferSize;
  volatile uint32_t head;
  volatile uint32_t tail;
} modbus_ring_buffer_t;

#define BUFFER_UART_SIZE 255
static uint8_t buf_uart_rs485[BUFFER_UART_SIZE];
static modbus_ring_buffer_t tx_ring_buffer_rs485;

extern UART_HandleTypeDef huart1;

void ModbusRTU_UartInit(void)
{
	tx_ring_buffer_rs485.buffer = buf_uart_rs485;
	tx_ring_buffer_rs485.bufferSize = BUFFER_UART_SIZE;
	tx_ring_buffer_rs485.head = 0;
	tx_ring_buffer_rs485.tail = 0;

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void ModbusRTU_UartWriteByte(uint8_t c)
{
	int i = (tx_ring_buffer_rs485.head + 1) % tx_ring_buffer_rs485.bufferSize;
	while (i == tx_ring_buffer_rs485.tail);

	tx_ring_buffer_rs485.buffer[tx_ring_buffer_rs485.head] = (uint8_t)c;
	tx_ring_buffer_rs485.head = i;

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
}

void ModbusRTU_UartPutArray(uint8_t * frame, uint32_t len)
{
	for(uint32_t i=0;i<len;i++)
	{
		ModbusRTU_UartWriteByte(frame[i]);
	}
}

/**
 * @brief UART ISR cho USART1 (YPC)
 * Gọi từ USART1_IRQHandler
 */
void ModbusRTU_UartIsrHandle(void)
{
	UART_HandleTypeDef *huart=&huart1;
	uint32_t isrflags   = READ_REG(huart->Instance->SR);
	uint32_t cr1its     = READ_REG(huart->Instance->CR1);

    if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
		huart->Instance->SR;
        uint8_t c = huart->Instance->DR;
        ModbusSlaveComp_DecodeMessage(c);
    }

    if (((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
    {
    	if(tx_ring_buffer_rs485.head == tx_ring_buffer_rs485.tail)
		{
		  __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
		}
    	else
		{
		  unsigned char c = tx_ring_buffer_rs485.buffer[tx_ring_buffer_rs485.tail];
		  tx_ring_buffer_rs485.tail = (tx_ring_buffer_rs485.tail + 1) % tx_ring_buffer_rs485.bufferSize;

		  huart->Instance->SR;
		  huart->Instance->DR = c;
		}
    }
}
