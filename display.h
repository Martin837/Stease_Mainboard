#ifndef __DISPLAY_H__
#define __DISPLAY_H__


#include <stdio.h>

void i2c_write_byte(uint8_t val);  
void lcd_toggle_enable(uint8_t val);
void lcd_send_byte(uint8_t val, int mode);
void lcd_clear(void);
void lcd_set_cursor(int line, int position);
static inline void lcd_char(char val);
void lcd_string(const char *s);
void lcd_init();


#endif