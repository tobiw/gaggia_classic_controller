#include "display.h"

Display::Display()
{
}

void Display::reset_font()
{
    tft->setFont(u8g_font_unifont);
}

void Display::init()
{
    tft = new U8GLIB_SSD1306_128X64(U8G_I2C_OPT_NONE);
    reset_font();
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

void Display::draw_live_status(char *buf_temperature, char *buf_pressure, char *buf_status)
{
    tft->firstPage();
    do {
        print_text(0, getLineY(0), buf_temperature);
        print_text(0, getLineY(1), buf_pressure);
        print_text(0, getLineY(2), buf_status);
    } while(tft->nextPage());
}

void Display::draw_graph(const char *title, uint16_t *data, size_t datalen, uint16_t min, uint16_t max)
{
    const uint8_t graph_x_size = 120;
    const uint8_t graph_y_size = 44;
    const uint8_t graph_bottom = 10+graph_y_size;

    // Transform data x axis to graph dimensions
    const double x_factor = (double)graph_x_size / (double)datalen;

    // Transform data y axis to graph dimensions
    const int data_range = max - min;
    const double y_factor = (double)graph_y_size / (double)data_range;

    uint16_t max_value = data[0];
    char buf[8];

    tft->firstPage();
    do {
        tft->setFont(u8g_font_5x7);
        print_text(0, 8, title);

        tft->drawLine(5, graph_bottom, 5+graph_x_size, graph_bottom); // X
        tft->drawLine(5, 10, 5, graph_bottom); // Y

        for (unsigned int i = 1; i < datalen; i++) {
            if (data[i] > max_value) max_value = data[i];

            tft->drawLine(5+((i-1)*x_factor), graph_bottom-(int)(data[i-1]*y_factor), 5+(i*x_factor), graph_bottom-(int)(data[i]*y_factor));
        }

        sprintf(buf, "%u", max_value);
        print_text(100, 8, buf);
        sprintf(buf, "%u", datalen / 2);
        print_text(50, 62, buf);
        reset_font();
    } while(tft->nextPage());
}
