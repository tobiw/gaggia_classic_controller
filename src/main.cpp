#include <Arduino.h>

//#include "max6675.h"
#include "SmartButton.h"

//#include "heater_controller.h"
#include "display.h"

#define PIN_SDA 2
#define PIN_SCL 3
#define PIN_BTN1 4
#define PIN_BREW_SWITCH 8
#define PIN_SSR 5
#define PIN_ADC_PRESSURE A0
#define PIN_LED_R 19
#define PIN_LED_G 20
#define PIN_LED_B 21

#define DISPLAY_UPDATE_INTERVAL 500
#define RECORD_LOG_SIZE 80

enum display_status_t {
    DISPLAY_WARMUP_TIMER = 0,
    DISPLAY_LIVE,
    DISPLAY_BREWING,
    DISPLAY_GRAPH_TEMPERATURE,
    DISPLAY_GRAPH_PRESSURE,
};

enum heater_action_t {
    HEATER_OFF = 0,
    HEATER_ON,
    HEATER_25PCT,
    HEATER_7PCT
}; // just what the user input is telling the device, not the actual SSR output

// Globals
display_status_t display_status = DISPLAY_WARMUP_TIMER;
heater_action_t heater_action = HEATER_ON;
volatile uint8_t isr_heater_action = 1;
unsigned int log_index = 0;
unsigned long brew_timer = 0;

//HeaterController heater_controller;

Display display;

//MAX6675 thermocouple(15, 10, 14); // SCK, CS, MISO

using namespace smartbutton;
SmartButton button(PIN_BTN1, SmartButton::InputType::NORMAL_HIGH);
bool button_pressed = false;
bool button_long_pressed = false;

void set_rgb_led(uint8_t r, uint8_t g, uint8_t b) {
    if (r + g + b <= 1) {
        digitalWrite(PIN_LED_R, r == 1 ? LOW : HIGH);
        digitalWrite(PIN_LED_G, g == 1 ? LOW : HIGH);
        digitalWrite(PIN_LED_B, b == 1 ? LOW : HIGH);
    } else { // flash to mix multiple colours
        // TODO: non-blocking
    }
}

void goto_next_heater_action() {
    switch (heater_action) {
        case HEATER_OFF:
            heater_action = HEATER_ON;
            isr_heater_action = 1;
            break;
        case HEATER_ON:
            heater_action = HEATER_25PCT;
            isr_heater_action = 2;
            break;
        case HEATER_25PCT:
            heater_action = HEATER_7PCT;
            isr_heater_action = 3;
            break;
        case HEATER_7PCT:
            heater_action = HEATER_OFF;
            isr_heater_action = 0;
            break;
    }
}

void goto_mode(display_status_t new_status) {
    display_status = new_status;
    switch (display_status) {
        case DISPLAY_LIVE:
            heater_action = HEATER_7PCT;
            isr_heater_action = 3;
            // Prepare logging and timer for brewing
            log_index = 0;
            brew_timer = 0;
            break;
        case DISPLAY_BREWING:
            heater_action = HEATER_25PCT;
            isr_heater_action = 2;
            break;
        case DISPLAY_GRAPH_TEMPERATURE:
            heater_action = HEATER_OFF;
            isr_heater_action = 0;
            break;
    }
}

void button_callback(SmartButton *b, SmartButton::Event event, int clicks) {
    if (event == SmartButton::Event::CLICK) {
        if (clicks == 1) {
            switch (display_status) {
                /*case DISPLAY_WARMUP_TIMER: // click on warmup screen advances to Live Status page
                    display_status = DISPLAY_LIVE;
                    break;*/
                case DISPLAY_WARMUP_TIMER: // TODO: remove once PID is active
                case DISPLAY_LIVE: // only switch heater action on Live Status or Brewing pages
                case DISPLAY_BREWING:
                    goto_next_heater_action();
                    break;
                case DISPLAY_GRAPH_TEMPERATURE: // switch between graphs
                    display_status = DISPLAY_GRAPH_PRESSURE;
                    break;
                case DISPLAY_GRAPH_PRESSURE: // switch between graphs
                    display_status = DISPLAY_GRAPH_TEMPERATURE;
                    break;
            }
        }
    } else if (event == SmartButton::Event::HOLD) {
        switch (display_status) {
            case DISPLAY_WARMUP_TIMER: // TODO: remove once PID is active
                goto_mode(DISPLAY_LIVE);
                break;
            case DISPLAY_LIVE:
                goto_mode(DISPLAY_BREWING);
                break;
            case DISPLAY_BREWING:
                goto_mode(DISPLAY_GRAPH_TEMPERATURE);
                break;
            case DISPLAY_GRAPH_TEMPERATURE:
            case DISPLAY_GRAPH_PRESSURE:
                goto_mode(DISPLAY_LIVE);
                break;
            //case DISPLAY_WARMUP_TIMER:
            default:
                break;
        }
    }
}

void setup() {
    Serial.begin(115200);
    display.init();

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
    button.begin(button_callback);

    pinMode(PIN_BREW_SWITCH, INPUT_PULLUP); // externally pulled-up

    //heater_controller.set_target(80);
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
        // Driving of SSR occurs in TIMER1_OVF ISR
    }
}

/*
 * Python program to calculate TCNT value:
 * def calc(p):
 *   timer_clock = cpu/prescale
 *   return (65535 - (p * timer_clock), 65535 - ((1-p) * timer_clock))
 */
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
        if (isr_heater_action == 2) { // 25%
            if (current_ssr_output) { // on cycle
                TCNT1 = 61629;
            } else { // off cycle
                TCNT1 = 53816;
            }
        } else if (isr_heater_action == 3) { // 7%
            if (current_ssr_output) { // on cycle
                TCNT1 = 64441;
            } else { // off cycle
                TCNT1 = 51004;
            }
        } else {
            TCNT1 = 57723;
        }
    } else {
        TCNT1 = 57723; // default 0.5 s interval
    }
}

void get_buf_temperature(char *buf_temperature, double temperature, unsigned int t_target) {
    char buf[8];
    dtostrf(temperature, 3, 1, buf);
    if (t_target == 0) {
        sprintf(buf_temperature, "%sC", buf);
    } else {
        sprintf(buf_temperature, "%sC (%uC)", buf, t_target);
    }
}

void get_buf_pressure(char *buf_pressure, double pressure_bar) {
    char buf[8];
    dtostrf(pressure_bar, 2, 1, buf);
    sprintf(buf_pressure, "%s bar", buf);
}

void get_buf_status(char *buf_status, heater_action_t heater_action) {
    char buf[8];
    switch (heater_action) {
        case HEATER_ON:
            strcpy(buf, " ON");
            break;
        case HEATER_25PCT:
            strcpy(buf, "25%");
            break;
        case HEATER_7PCT:
            strcpy(buf, " 7%");
            break;
        default:
            strcpy(buf, "OFF");
            break;
    }
    strcpy(buf_status, buf);
    /*if (display_status == DISPLAY_BREWING) {
        sprintf(buf_status, "%s  %s", buf, record_log ? "Rec" : "");
    } else {
        strcpy(buf_status, buf);
    }*/
}

void get_buf_timer(char *buf_timer, unsigned long m) {
    if (display_status == DISPLAY_WARMUP_TIMER) {
        unsigned int m_min = (unsigned int)((m / 1000) / 60);
        unsigned int m_sec = (unsigned int)((m / 1000) % 60);
        sprintf(buf_timer, "%02u:%02u", m_min, m_sec);
    } else { // Brew Status
        sprintf(buf_timer, "%02lu", m);
    }
}

void loop() {
    static char buf_temperature[16], buf_pressure[10], buf_status[16], buf_timer[8];

    static heater_action_t last_heater_action = HEATER_OFF;
    const unsigned int t_target = 80;

    static unsigned long last_update_time = 0;
    static unsigned long last_brew_timer_update = 0;
    const unsigned long m = millis();

    const double temperature = 20;//thermocouple.readCelsius();
    const unsigned int pressure_raw = analogRead(PIN_ADC_PRESSURE);

    // Record temperature (in C * 10) every 0.5 s
    static uint16_t temperature_log[RECORD_LOG_SIZE];

    // Record pressure (in bar * 10) every 0.5 s
    static uint16_t pressure_log[RECORD_LOG_SIZE];

    static unsigned long brew_switch_check_counter = 0;
    static bool brew_switch_activated = false;

    // Handle buttons and switches
    SmartButton::service();

    // Brew switch is a signal from an AC detection circuit (opto-isolated) so it
    // alternates between LOW and HIGH at the AC frequency. If switch is off, signal
    // is continuously HIGH.
    // At 50Hz mains the period is 20 ms; at 60Hz it is 16.7ms.
    // The main loop runs approx. every 1-2 ms so once the signal input is LOW it will stay
    // there for at least 10 loops.
    if (digitalRead(PIN_BREW_SWITCH) == LOW) { // any reading of LOW means switch is activated
        brew_switch_activated = true; // keep track that BREWING mode was entered because of brew switch
        brew_switch_check_counter = 0; // reset every time we see the switch LOW
        goto_mode(DISPLAY_BREWING);
    } else if (display_status == DISPLAY_BREWING && brew_switch_activated) { // only check for disabling the switch if currently brewing
        // Need to verify for more than one AC cycle that signal stays high.
        // Program loop frequency is about 1000Hz (1 ms per loop).
        // Need to read for at least 1 full mains cycle = 40 ms.
        // In 40 ms there are approx. 40 program loop cycles.
        // Count up to 120 as long as switch input is HIGH and only
        // deactivate brewing if it stayed HIGH the whole time.
        if (brew_switch_check_counter++ > 120) {
            brew_switch_check_counter = 0;
            goto_mode(DISPLAY_GRAPH_TEMPERATURE);
        }
    }

    // Set SSR based on HeaterController, not direct from button inputs or measured temperature
    // TODO: Use HeaterController (PID)
    //heater_controller.update(temperature);
    //if (heater_action == HEATER_ON && heater_controller.get_output() == 1) set_ssr(HEATER_ON);
    if (heater_action != last_heater_action) {
        set_ssr(heater_action);
        last_heater_action = heater_action;
    }

    // Automatically move from Warmup to Live after 15 minutes
    if (m > 900000) goto_mode(DISPLAY_LIVE);

    // Only update Serial and display once per second
    // Extra work in here takes approx. 60 ms (measured with oscilloscope)
    if ((last_update_time + DISPLAY_UPDATE_INTERVAL) < m) {
        // Increment brew timer by one second if in brewing mode
        if (display_status == DISPLAY_BREWING && (last_brew_timer_update + 1000) < m) {
            last_brew_timer_update = m;
            brew_timer++;
        }

        // Pressure conversion
        // Max pressure of sensor is 1.2 Mpa = 174 psi = 12 bar
        // voltage range of sensor is 0.5-4.5V = 4V
        double pressure_V = pressure_raw * (5.0 / 1024.0); // factor is (maxV / maxDigitalValue)
        double pressure_bar = (pressure_V - 0.17) * 3.0; // subtract 1 bar pressure voltage and apply conversion factor

        // Populate display buffers (TODO: only run what's required for current display mode)
        get_buf_temperature(buf_temperature, temperature, display_status == DISPLAY_WARMUP_TIMER ? 0 : t_target);
        get_buf_pressure(buf_pressure, pressure_bar);
        get_buf_status(buf_status, heater_action);
        get_buf_timer(buf_timer, display_status == DISPLAY_WARMUP_TIMER ? m : brew_timer);

        switch (display_status) {
            case DISPLAY_WARMUP_TIMER:
                display.draw_warmup_timer(buf_temperature, buf_timer, buf_status);
                break;
            case DISPLAY_LIVE:
                display.draw_live_status(buf_temperature, buf_pressure, buf_status);
                break;
            case DISPLAY_BREWING:
                display.draw_brew_status(buf_temperature, buf_pressure, buf_status, buf_timer);
                break;
            case DISPLAY_GRAPH_TEMPERATURE:
                display.draw_graph("Temp", temperature_log, log_index, 10, 120);
                break;
            case DISPLAY_GRAPH_PRESSURE:
                display.draw_graph("Pres", pressure_log, log_index, 0, 14);
                break;
        }

        // Record temperature and pressure to log
        if (display_status == DISPLAY_BREWING) {
            // Filter out values outside of 10C to 150C range
            if (temperature < 150 && temperature > 10)
                temperature_log[log_index] = (uint16_t)(temperature * 10.0);

            // Filter out values outside of 0 to 15 bar range
            if (pressure_bar < 15 && pressure_bar > 0)
                pressure_log[log_index] = (uint16_t)(pressure_bar * 10.0);

            log_index++;
            if (log_index >= RECORD_LOG_SIZE) { // 40 s
                goto_mode(DISPLAY_GRAPH_TEMPERATURE);
            }
        }

        last_update_time = m;
    }
}
