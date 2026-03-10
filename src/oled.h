#ifndef OLED_H
#define OLED_H

#include <U8g2lib.h>
#include <Wire.h>

// SSD1306 128x64, hardware I2C, 1-page buffer (~128 bytes RAM)
// I2C address: 0x3C (default, SA0 low)
// Drawing requires firstPage()/nextPage() loop instead of clearBuffer()/sendBuffer()
extern U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2;

void oled_init(void);

#endif // OLED_H
