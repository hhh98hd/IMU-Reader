/*
 * lcd.h
 *
 *  Created on: May 18, 2024
 *  Author: Huy Hoang Hoang - U7671528
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

void lcd_init(void);
void lcd_send_cmd (char cmd, int i2c_frame_size);
void lcd_send_data (char data);
void lcd_send_string (const char* format, ...);
void lcd_set_cursor(int row, int col);
void lcd_clear(void);

#endif /* INC_LCD_H_ */
