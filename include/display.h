#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include <Wire.h>
#include <U8glib.h>

/*
 * Mandatory x offset: 5 pixel
 * line 1: 20, line 2: 40, line 3: 60
 */
class Display {
    public:
        Display();
        void init();
        void clear();
        void print_text(int x, int y, const char *s);
        void draw_rect(int x, int y, int w, int h, int color);
        int getLineY(int i);
        void firstPage();
        int nextPage();
    private:
        U8GLIB_SSD1306_128X64 *tft;
};

#endif
