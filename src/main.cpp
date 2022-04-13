#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#define PIN_TFT_CLK 18
#define PIN_TFT_SDA 23
#define PIN_TFT_RES 4
#define PIN_TFT_DC 2
#define PIN_TFT_CS 5
Adafruit_ST7735 tft = Adafruit_ST7735(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RES);

#define PIN_BTN1 34
#define PIN_SSR 32
#define PIN_STATUS_INDICATOR 2 // built-in LED

enum heating_status_t {
    STATUS_COLD = 0,
    STATUS_HEATING,
    STATUS_NEAR_TARGET,
    STATUS_HOT
};

enum heater_action_t {
    HEATER_OFF = 0,
    HEATER_ON
} heater_action;

void init_tft() {
    // Need to modify Adafruit_ST7735.cpp: remove MADCTL_MX to de-mirror text
    tft.initR(INITR_MINI160x80);
    tft.setRotation(3);
    tft.invertDisplay(true); // required
}

volatile int last_btn = HIGH;
volatile bool btn1_pressed = false;
volatile unsigned long last_btn_press_time = 0;

// Note: cannot use delay() in ISR for debouncing
void IRAM_ATTR isr_btn1() {
    const int btn = digitalRead(PIN_BTN1);
    if (!btn1_pressed && digitalRead(PIN_BTN1) == LOW && last_btn == HIGH) {
        btn1_pressed = true;
        last_btn_press_time = millis();
    }
    last_btn = btn;
}

void setup() {
    Serial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(PIN_SSR, OUTPUT);
    digitalWrite(PIN_SSR, LOW);
    pinMode(PIN_BTN1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN1), isr_btn1, FALLING);
    init_tft();

    heater_action = HEATER_OFF;
}

void tft_text(int x, int y, const char *s) {
    tft.setCursor(x, y);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextWrap(false);
    tft.print(s);
}

void tft_draw_status(int color) {
    //tft.fillScreen(color);
    tft.fillScreen(0);
    tft.fillRect(10, 40, tft.width() - 10, tft.height() - 10, color);
}

void tft_set_status_color(heating_status_t s) {
    switch (s) {
        case STATUS_COLD:
            tft_draw_status(ST77XX_BLUE);
            break;
        case STATUS_HEATING:
            tft_draw_status(ST77XX_RED);
            break;
        case STATUS_NEAR_TARGET:
            tft_draw_status(ST77XX_ORANGE);
            break;
        case STATUS_HOT:
            tft_draw_status(ST77XX_GREEN);
            break;
    }
}

void set_ssr(heater_action_t a) {
    Serial.print("SSR set to ");
    if (a == HEATER_ON) {
        digitalWrite(PIN_SSR, HIGH);
        //digitalWrite(PIN_STATUS_INDICATOR, HIGH);
        Serial.println("ON");
    } else {
        digitalWrite(PIN_SSR, LOW);
        //digitalWrite(PIN_STATUS_INDICATOR, LOW);
        Serial.println("OFF");
    }
}

void loop() {
    static int t = 60;
    static char buf[32];
    const unsigned int t_target = 80;
    static heater_action_t heater_action = HEATER_OFF;
    static heating_status_t heating_status = STATUS_COLD;

    double td = t + 0.78;

    static unsigned long last_update_time = 0;
    const unsigned long m = millis();

    if ((unsigned int)td < (t_target - 5)) {
        if (heater_action == HEATER_ON) {
            t += 2; // rapid increase during heating phase
            heating_status = STATUS_HEATING;
        } else  {
            t = (t > 30 ? t - 1 : t); // cool to room temperature
            heating_status = STATUS_COLD;
        }
    } else if ((unsigned int)td < (t_target - 2)) {
        if (heater_action == HEATER_ON) t += 1; // slower increase when near target
        else t -= 2;
        heating_status = STATUS_NEAR_TARGET;
    } else if ((unsigned int)td >= (t_target - 2) || (unsigned int)td < (t_target + 2)) {
        if (heater_action == HEATER_ON) t += 0; // target reached, maintainer temperature
        else t -= 2;
        heating_status = STATUS_HOT;
    } else {
        heating_status = STATUS_COLD;
    }

    if (btn1_pressed && ((last_btn_press_time + 200) < m)) {
        heater_action = (heater_action == HEATER_OFF) ? HEATER_ON : HEATER_OFF;
        btn1_pressed = false;

        set_ssr(heater_action);
    }

    // Only update Serial and display once per second
    if ((last_update_time + 1000) < m) {
        Serial.print("LOOP: status ");
        Serial.print(heating_status);
        Serial.print(" heater ");
        Serial.println(heater_action);

        tft_set_status_color(heating_status);
        tft_text(10, 10, "T:");
        dtostrf(td, 3, 1, buf);
        tft_text(40, 10, buf);
        sprintf(buf, "(%u)", t_target);
        tft_text(90, 10, buf);
        tft_text(10, 20, heater_action == HEATER_OFF ? "OFF" : "ON");

        last_update_time = m;
    }
}
