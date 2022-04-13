#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

class Display {
    public:
        Display();
        void init();
        void clear();
        void print_text(int x, int y, const char *s);
        void draw_rect(int x, int y, int w, int h, int color);
    private:
        Adafruit_ST7735 *tft;
};

#endif
