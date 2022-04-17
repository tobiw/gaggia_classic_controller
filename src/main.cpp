#include <Arduino.h>

#include "max6675.h"

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
    HEATER_ON,
    HEATER_50PCT,
    HEATER_25PCT,
    HEATER_12PCT
} heater_action; // just what the user input is telling the device, not the actual SSR output

HeaterController heater_controller;

Display display;

MAX6675 thermocouple(15, 10, 14); // SCK, CS, MISO

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
    /*display.clear();
    display.print_text(0, 0, "T: 20.0");
    display.print_text(0, 19, "1.0 bar");*/

    noInterrupts();
    TCCR1A = 0;
    TCNT1 = 57723; // default 0.5 s interval
    TCCR1B = _BV(CS10) | _BV(CS12);
    TIMSK1 |= _BV(TOIE1);
    interrupts();

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

volatile bool heater_pwm = false;
volatile bool current_ssr_output = false;

void set_ssr(heater_action_t a) {
    if (a == HEATER_ON) {
        heater_pwm = false;
        digitalWrite(PIN_SSR, HIGH);
        set_rgb_led(1, 0, 0);
    } else if (a == HEATER_OFF) {
        heater_pwm = false;
        digitalWrite(PIN_SSR, LOW);
        set_rgb_led(0, 0, 0);
    } else {
        heater_pwm = true;
        //set_rgb_led(0, 1, 0);
        // Driving of SSR occurs in TIMER1_OVF ISR
    }
}

volatile uint8_t isr_heater_action = 0;

ISR(TIMER1_OVF_vect)
{
    if (heater_pwm) {
        current_ssr_output = !current_ssr_output;
        digitalWrite(PIN_SSR, current_ssr_output ? HIGH : LOW);
        digitalWrite(PIN_LED_R, current_ssr_output ? LOW : HIGH);

        // Set 65535-x according to next PWM phase when duty cycle is < 50%
        // 50%: (16MHz / 1024 / (65535-15625) = 57723 = 1/2 s on and off
        // 25%: 61629 on, 53816 off
        // 12%: 63660 on, 51785 off
        if (isr_heater_action == 2) { // 50%
            TCNT1 = 57723;
        } else if (isr_heater_action == 3) { // 25%
            if (current_ssr_output) { // on cycle
                TCNT1 = 61629;
            } else { // off cycle
                TCNT1 = 53816;
            }
        } else if (isr_heater_action == 4) { // 12%
            if (current_ssr_output) { // on cycle
                TCNT1 = 63660;
            } else { // off cycle
                TCNT1 = 51785;
            }
        } else {
            TCNT1 = 57723;
        }
    } else {
        TCNT1 = 57723; // default 0.5 s interval
    }
}

void loop() {
    static char buf[32];
    static char buf_temperature[32], buf_pressure[32], buf_status[16];

    static heater_action_t heater_action = HEATER_OFF;
    static heater_action_t last_heater_action = HEATER_OFF;
    const unsigned int t_target = 80;

    static unsigned long last_update_time = 0;
    const unsigned long m = millis();

    const double temperature = thermocouple.readCelsius();
    const unsigned int pressure_raw = analogRead(PIN_ADC_PRESSURE);

    // Handle button press -> turn heating on or off
    if (digitalRead(PIN_BTN1) == LOW && last_btn == HIGH) { // pressed
        if ((last_btn_press_time + 200) < m) { // debounce
            switch (heater_action) {
                case HEATER_OFF:
                    heater_action = HEATER_ON;
                    isr_heater_action = 1;
                    break;
                case HEATER_ON:
                    heater_action = HEATER_50PCT;
                    isr_heater_action = 2;
                    break;
                case HEATER_50PCT:
                    heater_action = HEATER_25PCT;
                    isr_heater_action = 3;
                    break;
                case HEATER_25PCT:
                    heater_action = HEATER_12PCT;
                    isr_heater_action = 4;
                    break;
                case HEATER_12PCT:
                    heater_action = HEATER_OFF;
                    isr_heater_action = 0;
                    break;
            }

            last_btn_press_time = m;
            last_btn = LOW;
        }
    } else if (digitalRead(PIN_BTN1) == HIGH && last_btn == LOW) { // released
        last_btn = HIGH;
    }

    // Set SSR based on HeaterController, not direct from button inputs or measured temperature
    // TODO: Use HeaterController (PID)
    //heater_controller.update(temperature);
    //if (heater_action == HEATER_ON && heater_controller.get_output() == 1) set_ssr(HEATER_ON);
    if (heater_action != last_heater_action) {
        set_ssr(heater_action);
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
        Serial.print("LOOP: heater");
        Serial.print(heater_action);
        Serial.print(" ADC ");
        Serial.print(pressure_raw);
        Serial.print(" V ");
        Serial.print(pressure_V);
        Serial.print(" bar ");
        Serial.println(pressure_bar);

        // Start drawing display
        display.firstPage();
        do {
            // First line: temperature
            dtostrf(temperature, 3, 1, buf);
            sprintf(buf_temperature, "%sC (%uC)", buf, t_target);

            // Second line: pressure
            dtostrf(pressure_bar, 2, 1, buf);
            sprintf(buf_pressure, "%s bar", buf);

            // Third line: status
            switch (heater_action) {
                case HEATER_ON:
                    strcpy(buf_status, "ON");
                    break;
                case HEATER_50PCT:
                    strcpy(buf_status, "50%");
                    break;
                case HEATER_25PCT:
                    strcpy(buf_status, "25%");
                    break;
                case HEATER_12PCT:
                    strcpy(buf_status, "12%");
                    break;
                default:
                    strcpy(buf_status, "OFF");
                    break;
            }

            display.print_text(0, display.getLineY(0), buf_temperature);
            display.print_text(0, display.getLineY(1), buf_pressure);
            display.print_text(0, display.getLineY(2), buf_status);
        } while(display.nextPage());

        last_update_time = m;
    }
}
