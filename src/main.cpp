#include <Arduino.h>

#include "heater_controller.h"
#include "display.h"

#define PIN_SDA 2
#define PIN_SCL 3
#define PIN_BTN1 4
#define PIN_BTN2 7
#define PIN_SSR 5
#define PIN_ADC_PRESSURE A0
#define PIN_LED_R 19
#define PIN_LED_G 20
#define PIN_LED_B 21

#define DISPLAY_UPDATE_INTERVAL 500

enum heating_status_t {
    STATUS_COLD = 0,
    STATUS_HEATING,
    STATUS_NEAR_TARGET,
    STATUS_HOT
};

enum heater_action_t {
    HEATER_OFF = 0,
    HEATER_ON
} heater_action; // just what the user input is telling the device, not the actual SSR output

HeaterController heater_controller;

Display display;

int last_btn = HIGH;
unsigned long last_btn_press_time = 0;

void set_rgb_led(uint8_t r, uint8_t g, uint8_t b) {
    if (r + g + b <= 1) {
        digitalWrite(PIN_LED_R, r == 1 ? LOW : HIGH);
        digitalWrite(PIN_LED_G, g == 1 ? LOW : HIGH);
        digitalWrite(PIN_LED_B, b == 1 ? LOW : HIGH);
    } else { // flash to mix multiple colours
        // TODO: non-blocking
    }
}

void setup() {
    Serial.begin(115200);
    display.init();
    display.clear();
    display.print_text(0, 0, "T: 20.0");
    display.print_text(0, 19, "P: 1.0 bar");

    // RGB LEDs
#define LED_TEST_DELAY 200
    pinMode(PIN_LED_R, OUTPUT);
    pinMode(PIN_LED_G, OUTPUT);
    pinMode(PIN_LED_B, OUTPUT);
    set_rgb_led(1, 0, 0);
    delay(LED_TEST_DELAY);
    set_rgb_led(0, 1, 0);
    delay(LED_TEST_DELAY);
    set_rgb_led(0, 0, 1);
    delay(LED_TEST_DELAY);
    set_rgb_led(0, 0, 0);
    delay(LED_TEST_DELAY);

    pinMode(PIN_ADC_PRESSURE, INPUT);

    pinMode(PIN_SSR, OUTPUT);
    digitalWrite(PIN_SSR, LOW);
    pinMode(PIN_BTN1, INPUT_PULLUP);
    pinMode(PIN_BTN2, INPUT_PULLUP);

    heater_action = HEATER_OFF;
    heater_controller.set_target(80);
}

void display_set_status_color(heating_status_t s) {
    const int y = 24;
    // TODO: based on HeaterController status instead of user input
    /*switch (s) {
        case STATUS_COLD:
            display.print_text(0, y, "Cold");
            set_rgb_led(0, 0, 1);
            break;
        case STATUS_HEATING:
            display.print_text(0, y, "Heating");
            set_rgb_led(1, 0, 0);
            break;
        case STATUS_NEAR_TARGET:
            display.print_text(0, y, "Near");
            set_rgb_led(1, 0, 0);
            break;
        case STATUS_HOT:
            display.print_text(0, y, "Hot");
            set_rgb_led(0, 1, 0);
            break;
    }*/
    set_rgb_led(heater_action == HEATER_ON ? 1 : 0, 0, 0);
}

void set_ssr(heater_action_t a) {
    if (a == HEATER_ON) {
        digitalWrite(PIN_SSR, HIGH);
        set_rgb_led(1, 0, 0);
    } else {
        digitalWrite(PIN_SSR, LOW);
        set_rgb_led(0, 0, 0);
    }
}

void loop() {
    static double t = 20.3;
    static char buf[32];
    static heater_action_t heater_action = HEATER_OFF;
    static heater_action_t last_heater_action = HEATER_OFF;
    static heating_status_t heating_status = STATUS_COLD;
    const unsigned int t_target = 80;
    const unsigned int pressure_raw = analogRead(PIN_ADC_PRESSURE);

    static unsigned long last_update_time = 0;
    const unsigned long m = millis();

    static double last_temperature = t;
    static double last_pressure_bar = 1.0;

    // Handle button press -> turn heating on or off
    if (digitalRead(PIN_BTN1) == LOW && last_btn == HIGH) { // pressed
        if ((last_btn_press_time + 200) < m) { // debounce
            heater_action = (heater_action == HEATER_OFF) ? HEATER_ON : HEATER_OFF;
            last_btn_press_time = m;
            last_btn = LOW;
        }
    } else if (digitalRead(PIN_BTN1) == HIGH && last_btn == LOW) { // released
        last_btn = HIGH;
    }

    // Set SSR based on HeaterController, not direct from button inputs or measured temperature
    // TODO: Use HeaterController (PID)
    //heater_controller.update(t);
    //if (heater_action == HEATER_ON && heater_controller.get_output() == 1) set_ssr(HEATER_ON);
    if (heater_action != last_heater_action) {
        if (heater_action == HEATER_ON) set_ssr(HEATER_ON);
        else set_ssr(HEATER_OFF);
        last_heater_action = heater_action;
    }

    // Only update Serial and display once per second
    if ((last_update_time + DISPLAY_UPDATE_INTERVAL) < m) {
        // Pressure conversion
        // Max pressure of sensor is 1.2 Mpa = 174 psi = 12 bar
        // voltage range of sensor is 0.5-4.5V = 4V
        double pressure_V = pressure_raw * (5.0 / 1024.0); // factor is (maxV / maxDigitalValue)
        double pressure_bar = (pressure_V - 0.17) * 3.0; // subtract 1 bar pressure voltage and apply conversion factor

        // Serial monitor output
        Serial.print("LOOP: status ");
        Serial.print(heating_status);
        Serial.print(" heater ");
        Serial.print(heater_action);
        Serial.print(" ADC ");
        Serial.print(pressure_raw);
        Serial.print(" V ");
        Serial.print(pressure_V);
        Serial.print(" bar ");
        Serial.println(pressure_bar);

        // First line: temperature (update if changed by 0.5)
        if (t < (last_temperature - 0.5) || t > (last_temperature + 0.5)) {
            display.draw_rect(34, 0, 80, 18, SSD1306_BLACK);
            display.print_text(0, 0, "T:");
            dtostrf((double)t, 3, 1, buf);
            display.print_text(34, 0, buf);
            sprintf(buf, "(%u)", t_target);
            display.print_text(88, 0, buf);
            last_temperature = t;
        }
        // Second line: pressure (update if changed by 0.5)
        if (pressure_bar < (last_pressure_bar - 0.5) || pressure_bar > (last_pressure_bar + 0.5)) {
            display.draw_rect(28, 19, 80, 32, SSD1306_BLACK);
            display.print_text(0, 19, "P:");
            dtostrf(pressure_bar, 2, 1, buf);
            display.print_text(28, 19, buf);
            display.print_text(60, 19, " bar");
            last_pressure_bar = pressure_bar;
        }

        last_update_time = m;
    }

}
