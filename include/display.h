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
        // Init
        Display();
        void init();
        void reset_font();

        // Simple OLED functions
        void clear();
        void print_text(int x, int y, const char *s);
        void draw_rect(int x, int y, int w, int h, int color);
        int getLineY(int i);
        void firstPage();
        int nextPage();

        // Complex display
        void draw_live_status(char *buf_temperature, char *buf_pressure, char *buf_status);
        void draw_graph(const char *title, uint16_t *data, size_t datale, uint16_t min, uint16_t max);

    private:
        U8GLIB_SSD1306_128X64 *tft;
};

#endif
