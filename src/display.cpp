#include "display.h"

Display::Display()
{
}

void Display::init()
{
    tft = new Adafruit_SSD1306(128, 32, &Wire, -1);

    tft->begin(SSD1306_SWITCHCAPVCC, 0x3C);

    tft->display();

    tft->setTextSize(1);
    tft->setTextColor(SSD1306_WHITE);
    tft->setTextWrap(false);
}

void Display::clear()
{
    tft->clearDisplay();
}

void Display::print_text(int x, int y, const char *s)
{
    tft->setCursor(x, y);
    tft->print(s);
    tft->display();
}

void Display::draw_rect(int x, int y, int w, int h)
{
    tft->fillRect(x, y, w, h, SSD1306_WHITE);
}
