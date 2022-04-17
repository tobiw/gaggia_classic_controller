#include "display.h"

Display::Display()
{
}

void Display::init()
{
    tft = new U8GLIB_SSD1306_128X64(U8G_I2C_OPT_NONE);
    tft->setFont(u8g_font_unifont);
}

void Display::clear()
{
}

void Display::print_text(int x, int y, const char *s)
{
    tft->drawStr(x+4, y, s);
}

void Display::draw_rect(int x, int y, int w, int h, int color)
{
    tft->drawBox(x, y, w, h);
}

int Display::getLineY(int i) {
    return (i+1) * 20;
}

void Display::firstPage() {
    tft->firstPage();
}

int Display::nextPage() {
    return tft->nextPage();
}
