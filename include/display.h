#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>

/*
 * Mandatory x offset: 5 pixel
 * line 1: 20, line 2: 40, line 3: 60
 */
class Display {
    public:
        // Init
        Display();
        void init();
        void set_font_size(uint8_t size);

        // Simple OLED functions
        void clear();
        void print_text(int x, int y, const char *s);
        void draw_glyph(int x, int y, unsigned int glyph);
        void draw_rect(int x, int y, int w, int h, int color);
        int getLineY(int i);
        void firstPage();
        int nextPage();

        // Complex display
        void draw_live_status(char *buf_temperature, char *buf_pressure, char *buf_status);
        void draw_graph(const char *title, uint16_t *data, size_t datale, uint16_t min, uint16_t max);

    private:
        U8G2_SSD1306_128X64_NONAME_F_HW_I2C *tft;
};

#endif
