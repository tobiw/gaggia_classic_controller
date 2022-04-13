#include "display.h"

#define PIN_TFT_CLK 18
#define PIN_TFT_SDA 23
#define PIN_TFT_RES 4
#define PIN_TFT_DC 2
#define PIN_TFT_CS 5

Display::Display()
{
}

void Display::init()
{
    tft = new Adafruit_ST7735(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RES);

    // Need to modify Adafruit_ST7735.cpp: remove MADCTL_MX to de-mirror text
    tft->initR(INITR_MINI160x80);
    tft->setRotation(3);
    tft->invertDisplay(true); // required
    tft->fillScreen(0);
}

void Display::print_text(int x, int y, const char *s)
{
    tft->setCursor(x, y);
    tft->setTextSize(2);
    tft->setTextColor(ST77XX_WHITE);
    tft->setTextWrap(false);
    tft->print(s);
}

void Display::draw_rect(int x, int y, int w, int h, int color)
{
    tft->fillScreen(0);
    tft->fillRect(x, y, w, h, color);
}
