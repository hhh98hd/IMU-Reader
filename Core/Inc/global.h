/*
 * global.h
 *
 *  Created on: May 12, 2024
 *  Author: Huy Hoang Hoang - u7671528
 */

#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_

#include <stdarg.h>
#include <string.h>
#include <stdarg.h>
#include <string.h>

#include "stm32f4xx_hal.h"

#define ARM_MATH_CM4
#include "arm_math.h"  //CMSIS DSP library

#define LCD_ADDRESS (0x27 << 1)
#define IMU_ADDRESS (0x68 << 1)
#define G  9.81
#define RAD_TO_DEG 57.2957795131
#define CALIB_TIME 5              // Calibrate gyroscope for 5 seconds
#define DELTA_T 0.01               // 10ms

#define FILTER_SIZE 256
#define CONVOLVE_BLOCK_SIZE 2048
#define NUM_CHANNELS 2

void write_serial(UART_HandleTypeDef *uart_dev, const char* format, ...);

#endif /* INC_GLOBAL_H_ */
