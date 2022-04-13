#include <Arduino.h>

#include "heater_controller.h"
#include "display.h"

#define PIN_BTN1 A3
#define PIN_SSR A2
#define PIN_STATUS_INDICATOR 13 // built-in LED
#define PIN_ADC_PRESSURE A0

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

volatile int last_btn = HIGH;
volatile bool btn1_pressed = false;
volatile unsigned long last_btn_press_time = 0;

// Note: cannot use delay() in ISR for debouncing
void isr_btn1() {
    const int btn = digitalRead(PIN_BTN1);
    if (!btn1_pressed && digitalRead(PIN_BTN1) == LOW && last_btn == HIGH) {
        btn1_pressed = true;
        last_btn_press_time = millis();
    }
    last_btn = btn;
}

void setup() {
    Serial.begin(115200);
    display.init();

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(PIN_ADC_PRESSURE, INPUT);

    pinMode(PIN_SSR, OUTPUT);
    digitalWrite(PIN_SSR, LOW);
    pinMode(PIN_BTN1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN1), isr_btn1, FALLING);

    heater_action = HEATER_OFF;
    heater_controller.set_target(80);
}

void display_set_status_color(heating_status_t s) {
    switch (s) {
        case STATUS_COLD:
            display.draw_rect(10, 40, 100, 70, ST77XX_BLUE);
            break;
        case STATUS_HEATING:
            display.draw_rect(10, 40, 100, 70, ST77XX_RED);
            break;
        case STATUS_NEAR_TARGET:
            display.draw_rect(10, 40, 100, 70, ST77XX_ORANGE);
            break;
        case STATUS_HOT:
            display.draw_rect(10, 40, 100, 70, ST77XX_GREEN);
            break;
    }
}

void set_ssr(heater_action_t a) {
    if (a == HEATER_ON) {
        digitalWrite(PIN_SSR, HIGH);
        //digitalWrite(PIN_STATUS_INDICATOR, HIGH);
    } else {
        digitalWrite(PIN_SSR, LOW);
        //digitalWrite(PIN_STATUS_INDICATOR, LOW);
    }
}

void loop() {
    static int t = 60;
    static char buf[32];
    const unsigned int t_target = 80;
    static heater_action_t heater_action = HEATER_OFF;
    static heating_status_t heating_status = STATUS_COLD;
    const unsigned int pressure_raw = analogRead(PIN_ADC_PRESSURE);

    static unsigned long last_update_time = 0;
    const unsigned long m = millis();

    // Handle button press -> turn heating on or off
    if (btn1_pressed && ((last_btn_press_time + 200) < m)) {
        heater_action = (heater_action == HEATER_OFF) ? HEATER_ON : HEATER_OFF;
        btn1_pressed = false;
    }

    // Set SSR based on HeaterController, not direct from button inputs or measured temperature
    heater_controller.update(t);
    if (heater_action == HEATER_ON && heater_controller.get_output() == 1) set_ssr(HEATER_ON);
    else set_ssr(HEATER_OFF);

    // Only update Serial and display once per second
    if ((last_update_time + 1000) < m) {
        if (t < (t_target - 5)) {
            if (heater_action == HEATER_ON) {
                t += 2; // rapid increase during heating phase
                heating_status = STATUS_HEATING;
            } else  {
                t = (t > 30 ? t - 1 : t); // cool to room temperature
                heating_status = STATUS_COLD;
            }
        } else if (t < (t_target - 2)) {
            if (heater_action == HEATER_ON) t += 1; // slower increase when near target
            else t -= 2;
            heating_status = STATUS_NEAR_TARGET;
        } else if (t >= (t_target - 2) || t < (t_target + 2)) {
            if (heater_action == HEATER_ON) t += 0; // target reached, maintainer temperature
            else t -= 2;
            heating_status = STATUS_HOT;
        } else {
            heating_status = STATUS_COLD;
        }

        // Pressure conversion
        // Max pressure of sensor is 1.2 Mpa = 174 psi = 12 bar
        // voltage range of sensor is 4.5V
        // raw value is 0-4096 with Vref 1.1V?
        double pressure_V = pressure_raw * (3.3 / 4096.0); // factor is (maxV / maxDigitalValue)
        double pressure_bar = ((pressure_V - (480 * 3.3 / 4096.0)) * 3.0); // subtract 1 bar pressure voltage and apply conversion factor

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

        display.clear();
        display_set_status_color(heating_status);
        display.print_text(10, 10, "T:");
        dtostrf((double)t, 3, 1, buf);
        display.print_text(40, 10, buf);
        sprintf(buf, "(%u)", t_target);
        display.print_text(90, 10, buf);
        display.print_text(10, 20, heater_action == HEATER_OFF ? "OFF" : "ON");
        sprintf(buf, "P: %u", pressure_raw);
        display.print_text(80, 64, buf);

        last_update_time = m;
    }
}
