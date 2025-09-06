/*
 *  imu.c
 *
 *  Created on: May 12, 2024
 *  Author: Huy Hoang Hoang - u7671528
 */

#include <stdio.h>

#include "global.h"

void write_serial(UART_HandleTypeDef *uart_dev, const char* format, ...)
{
	// Initialise the buffer to hold the message
	char txMsg[255];

	// Format the message
	va_list args;
	va_start(args, format);
	vsnprintf(txMsg, sizeof(txMsg), format, args); // Format the string
	va_end(args);

	// Send UART message
	HAL_UART_Transmit(uart_dev, (uint8_t*)txMsg, strlen(txMsg), 1000);
}
