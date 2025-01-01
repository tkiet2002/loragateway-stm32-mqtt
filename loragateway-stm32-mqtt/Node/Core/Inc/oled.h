/*
 * OLED.h
 *
 *  Created on: Nov 21, 2024
 *      Author: khoav
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

extern I2C_HandleTypeDef hi2c1;


void oled_send_command(unsigned char command);

void oled_send_data(unsigned char dt);

void oled_init(void);
void oled_display_pattern(void);

void oled_display_char(unsigned char c);

void oled_set_cursor(unsigned char x, unsigned char y);
void ssd1306_char_f8x16(unsigned char ch);

void oled_display_string(const char *str);

#endif /* INC_OLED_H_ */
