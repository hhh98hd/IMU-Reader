/*
 * lcd.c
 *
 *  Created on: May 18, 2024
 *  Author: Huy Hoang Hoang - U7671528
 */

#include <stdio.h>

#include "global.h"
#include "lcd.h"

extern I2C_HandleTypeDef hi2c1;

void lcd_init(void)
{
    HAL_Delay(20);

	// Function Set: Set to 4-bit mode. The LCD is still in 8-bit mode so we only need to send once
	lcd_send_cmd(0x20, 4);
	HAL_Delay(10);

	// Function Set: 2-line display, 5x8 character font
	lcd_send_cmd(0x28, 4);
	HAL_Delay(10);

	// Entry Mode Set: Increment address by 1, set cursor to the right
	lcd_send_cmd(0x06, 4);
	HAL_Delay(10);

	lcd_clear();
	HAL_Delay(10);

	// Display Control: Display ON, Cursor OFF, Blink OFF
	lcd_send_cmd(0x0C, 4);
	HAL_Delay(10);
}

void lcd_send_cmd (char cmd, int i2c_frame_size)
{
    uint8_t i2c_frame_data[4];

    i2c_frame_data[0] = (cmd & 0xF0) | 0x0c;
    i2c_frame_data[1] = i2c_frame_data[0] & 0xFB;

    i2c_frame_data[2] = ((cmd << 4) & 0xF0) | 0x0c;
    i2c_frame_data[3] = i2c_frame_data[2] & 0xFB;

    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, i2c_frame_data, i2c_frame_size, 1000);
    
    HAL_Delay(1);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t i2c_frame_data[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	i2c_frame_data[0] = data_u|0x0D;  //en=1, rs=0
	i2c_frame_data[1] = data_u|0x09;  //en=0, rs=0
	i2c_frame_data[2] = data_l|0x0D;  //en=1, rs=0
	i2c_frame_data[3] = data_l|0x09;  //en=0, rs=0
	
	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, i2c_frame_data, 4, 1000);
	
	HAL_Delay(1);
}

void lcd_send_string (const char* format, ...)
{
//	lcd_send_cmd(0x80, 4);

    // Buffer to store the formatted string
    char tx_msg[255];

    // Format the message
	va_list args;
	va_start(args, format);
	vsnprintf(tx_msg, sizeof(tx_msg), format, args); // Format the string
	va_end(args);

    // Send the message
	for(int i = 0; i < strlen(tx_msg); i++) {
		lcd_send_data(tx_msg[i]);
	}
}

void lcd_set_cursor(int row, int col)
{
	uint8_t cmd;

	if(row == 0)
		cmd = (0x80 | (col + 0x00));
	else
		cmd = (0x80 | (col + 0x40));

	lcd_send_cmd(cmd, 4);
}

void lcd_clear (void)
{
	lcd_send_cmd(0x01, 4);
}
