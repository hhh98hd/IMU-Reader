/*
 * imu.c
 *
 *  Created on: May 12, 2024
 *  Author: Huy Hoang Hoang - u7671528
 */

#include "global.h"
#include "imu.h"

int16_t offset_z = 0;
extern UART_HandleTypeDef huart2;

void imu_init(I2C_HandleTypeDef* i2c_dev)
{
	uint8_t cmd;

	// Reset the IMU
	cmd = 0x00;
	HAL_I2C_Mem_Write(i2c_dev, IMU_ADDRESS, PWR_MGMT_1, 1, &cmd, 1, 1000);
	HAL_Delay(100);

	// Bring the IMU out of sleep mode and skip all self-tests;
	cmd = 0x01;
	HAL_I2C_Mem_Write(i2c_dev, IMU_ADDRESS, PWR_MGMT_1, 1, &cmd, 1, 1000);
	HAL_Delay(100);

	// Set the acceleration range to +/- 2G
	cmd = ACCEL_RANGE_2G;
	HAL_I2C_Mem_Write(i2c_dev, IMU_ADDRESS, ACCEL_CONFIG, 1, &cmd, 1, 1000);
	HAL_Delay(100);

	// Set the angular speed range to +/- 250degrees/s
	cmd = GYRO_RANGE_250DEG;
	HAL_I2C_Mem_Write(i2c_dev, IMU_ADDRESS, GYRO_CONFIG, 1, &cmd, 1, 1000);
	HAL_Delay(100);
}

void imu_calibarate(I2C_HandleTypeDef *i2c_dev)
{
	write_serial(&huart2, "Calibrating IMU...\n");

	int32_t gyro_sum = 0;
	uint32_t gyro_cnt = 0;
	uint32_t start_time = HAL_GetTick();

	while(1) {

		if (HAL_GetTick() - start_time >= CALIB_TIME * 1000) {
			offset_z = gyro_sum / gyro_cnt;
			break;
		}

		gyro_sum += get_raw_gyro_z(i2c_dev);
		gyro_cnt += 1;

		HAL_Delay(10);
	}

	write_serial(&huart2, "Calibration done! Offset = %d!\n", offset_z);
}


int16_t get_raw_gyro_z(I2C_HandleTypeDef* i2c_dev)
{
	// 16-bit integer value of angular speed
	uint8_t high_bits = 0, low_bits = 0;

	// Get angular speed of Z-axis
	HAL_I2C_Mem_Read(i2c_dev, IMU_ADDRESS, GYRO_ZOUT_H_REG, 1, &high_bits, 1, 1000);
	HAL_I2C_Mem_Read(i2c_dev, IMU_ADDRESS, GYRO_ZOUT_L_REG, 1, &low_bits, 1, 1000);
	int16_t raw_gyro_z = (high_bits << 8) | low_bits;

	if(raw_gyro_z >= 0 && raw_gyro_z - offset_z < 0)
		raw_gyro_z = 0;
	else
		raw_gyro_z = raw_gyro_z - offset_z;
	
	return raw_gyro_z;
}

float imu_get_gyro_Z(I2C_HandleTypeDef* i2c_dev)
{
	return get_raw_gyro_z(i2c_dev) / GYRO_SENSITIVITY_250DEG;
}

