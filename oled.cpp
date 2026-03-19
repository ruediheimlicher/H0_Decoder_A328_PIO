#include "oled.h"

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset= */ U8X8_PIN_NONE);

void oled_init(void)
{
    Wire.begin();
    u8g2.begin();
    // clear display: send empty pages
    u8g2.firstPage();
    do { } while (u8g2.nextPage());
}
