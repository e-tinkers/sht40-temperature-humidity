#ifndef LCD1602_DRIVER_H_
#define LCD1602_DRIVER_H_

#include <avr/io.h>
#include <util/delay.h>
#include "lcd1602.h"

// commands
#define LCD_CLEAR_DISPLAY         0x01
#define LCD_RETURN_HOME           0x02
#define LCD_ENTRY_MODE_SET        0x04
#define LCD_DISPLAY_CONTROL       0x08
#define LCD_CURSOR_SHIFT          0x10
#define LCD_FUNCTION_SET          0x20
#define LCD_SET_CGRAM_ADDR        0x40
#define LCD_SET_DDRAM_ADDR        0x80

// flags for display entry mode
#define LCD_ENTRY_RIGHT           0x00
#define LCD_ENTRY_LEFT            0x02
#define LCD_ENTRY_SHIFT_DISPLAY   0x01
#define LCD_ENTRY_SHIFT_CURSOR    0x00

// flags for display on/off control
#define LCD_DISPLAY_ON            0x04
#define LCD_DISPLAY_OFF           0x00
#define LCD_CURSOR_ON             0x02
#define LCD_CURSOR_OFF            0x00
#define LCD_BLINK_ON              0x01
#define LCD_BLINK_OFF             0x00

// flags for display/cursor shift
#define LCD_DISPLAY_MOVE          0x08
#define LCD_CURSOR_MOVE           0x00
#define LCD_MOVE_RIGHT            0x04
#define LCD_MOVE_LEFT             0x00

// flags for function set
#define LCD_8BIT_MODE             0x10
#define LCD_4BIT_MODE             0x00
#define LCD_2LINE                 0x08
#define LCD_1LINE                 0x00
#define LCD_5x10_DOTS             0x04
#define LCD_5x8_DOTS              0x00

// operation modes
#define CMD  0
#define DATA 1

// Maximum 8 custom char can be upload to CGRAM using lcd_create_char()
const char thermomitorSym[] = { 0x04, 0x0A, 0x0A, 0x0e, 0x0e, 0x1f, 0x1f, 0x0e}; // temperature (thermometer)
const char humiditySym[] = { 0x04, 0x04, 0x0A, 0x0A, 0x11, 0x11, 0x11, 0x0e};    // humidity (water drop)
const char speakerSym[] = {0x01, 0x03, 0x07, 0x1f, 0x1f, 0x07, 0x03, 0x01};      // speaker
const char musicSym[] = { 0x01, 0x03, 0x05, 0x09, 0x09, 0x0b, 0x1b, 0x18};       // music
const char plusMinusSym[] = {0x04, 0x04, 0x1F, 0x04, 0x04, 0x00, 0x1F, 0x00};    // +-
const char bellSym[] = {0x00, 0x04, 0x0E, 0x0E, 0x0E, 0x1F, 0x04, 0x00};         // bell
const char batteryLSym[] = {0x0e, 0x1b, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1f};     // battery low
const char batterySym[] = {0x0e, 0x1b, 0x11, 0x11, 0x13, 0x17, 0x1f, 0x1f};      // battery

typedef struct {
    PORT_t * port;
    uint8_t rs;
    uint8_t en;
    uint8_t d4;
    uint8_t d5;
    uint8_t d6;
    uint8_t d7;
} LCD_t;

extern LCD_t lcd;

void lcd_config() {

    (lcd.port)->DIRSET = lcd.rs | lcd.en | lcd.d4 | lcd.d5 | lcd.d6 | lcd.d7;
    (lcd.port)->OUTCLR = lcd.rs | lcd.en | lcd.d4 | lcd.d5 | lcd.d6 | lcd.d7;

}

void lcd_write(uint8_t value, uint8_t mode) {
	if(mode) // data
		(lcd.port)->OUTSET = lcd.rs;
	else   // cmd
		(lcd.port)->OUTCLR = lcd.rs;

	uint8_t lbits = value & 0x0f;
	uint8_t hbits = value & 0xf0;

	// send high 4-bit
	(lcd.port)->OUTSET = lcd.en;
	(lcd.port)->OUTSET = hbits;          // set the corresponding bits based in hbits
	(lcd.port)->OUTCLR = ~hbits & 0xf0;  // reset the correspoinding bits based in ~hbits
	_delay_us(1);
	(lcd.port)->OUTCLR = lcd.en;
	_delay_us(1);

	// send lower 4-bit
	(lcd.port)->OUTSET = lcd.en;
	(lcd.port)->OUTSET = lbits << 4;          // set the corresponding bits based in hbits
	(lcd.port)->OUTCLR = (~lbits & 0x0f) << 4;  // reset the correspoinding bits based in ~hbits    
	_delay_us(1);
	(lcd.port)->OUTCLR = lcd.en;
	_delay_us(37); // at least more than 37uS
}

void lcd_clear() {
	lcd_write(LCD_CLEAR_DISPLAY, CMD);
	_delay_ms(2);
}

void lcd_init(uint8_t cursor, uint8_t blink) {
	lcd_write(LCD_FUNCTION_SET | LCD_8BIT_MODE, CMD);
	_delay_us(4100);
	lcd_write(LCD_FUNCTION_SET | LCD_8BIT_MODE, CMD);
	_delay_us(100);
	lcd_write(LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE | LCD_5x8_DOTS, CMD);
	lcd_write(LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON | cursor | blink, CMD);
	lcd_clear();
	lcd_write(LCD_ENTRY_MODE_SET | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_CURSOR, CMD);
}

void lcd_cursor(uint8_t row, uint8_t col) {
	if (row > 2) row = 2;
	if (col > 16) col = 16;
	lcd_write(LCD_SET_DDRAM_ADDR | ((row - 1) << 6) | (col - 1), CMD);
}

void lcd_print(char c) {
	lcd_write((uint8_t) c, DATA);
}

void lcd_print_str(char* str) {
	uint8_t n = 0;
	while(str[n]) {
		lcd_print(str[n]);
		n++;
	}
}

void lcd_create_char(uint8_t addr, const char* font) {
	lcd_write(LCD_SET_CGRAM_ADDR | (addr * 8), CMD);
	for (int i=0; i<8; i++) {
		lcd_write((uint8_t)font[i], DATA);
	}
}

void lcd_off() {
	lcd_write(LCD_DISPLAY_OFF, CMD);
}

#endif
