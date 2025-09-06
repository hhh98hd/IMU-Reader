/*
 * imu.h
 *
 *  Created on: May 12, 2024
 *  Author: Huy Hoang Hoang - u7671528
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#define PWR_MGMT_1 				0x6B // Power management register
#define CONFIG					0x1A // Fsync and low-pass filter configuration register
#define GYRO_CONFIG             0x27 // Gyroscopes configuration register
#define ACCEL_CONFIG			0x1C // Accelerators configuration register

#define GYRO_ZOUT_H_REG 		0x47 // Angular speed on Z-axis
#define GYRO_ZOUT_L_REG 		0x48

#define GYRO_RANGE_250DEG		0x00
#define GYRO_SENSITIVITY_250DEG 131.f

#define ACCEL_RANGE_2G			0x00
#define ACCEL_SENSITIVITY_2G    16384.f

void imu_init(I2C_HandleTypeDef* i2c_dev);
void imu_calibarate(I2C_HandleTypeDef* i2c_dev);

int16_t get_raw_gyro_z(I2C_HandleTypeDef* i2c_dev);
float imu_get_gyro_Z(I2C_HandleTypeDef* i2c_dev);

#endif /* INC_IMU_H_ */
