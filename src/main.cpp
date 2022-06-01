#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"


#include "SmartButton.h"

//#include "heater_controller.h"
#include "display.h"

/*
 * Pin definitions for inputs and outputs
 */
#define PIN_SDA 2
#define PIN_SCL 3
#define PIN_BTN1 4
#define PIN_BREW_SWITCH 8
#define PIN_SSR 5
#define PIN_ADC_PRESSURE A0
#define PIN_LED_R 19
#define PIN_LED_G 20
#define PIN_LED_B 21
#define PIN_SPI_CS 10
#define PIN_SPI_SCLK 15
#define PIN_SPI_MISO 14

/*
 * Global constants definitions
 */
#define DISPLAY_UPDATE_INTERVAL 500 // used in the main loop to only update the display periodically
#define RECORD_LOG_SIZE 100 // fixed size for the temperature and pressure logs
#define AUTO_ADVANCE_TIME_WARMUP_TO_LIVE 900000 // move from Warmup to Live Status after 15 minutes
#define AUTO_ADVANCE_TIME_LIVE_TO_BREWING 5000 // move from Live Status to Brewing after 5 seconds (if brew switch activated)

/*
 * The program's main modes. These are executed fairly sequentially in a typical brew workflow:
 * Powering up the machine enters the Warmup mode for 15 minutes before moving to the Live status/idle
 * screen (PID maintains set temperature). When the brew switch is activated the program moves to the
 * brewing mode which displays a shot timer and records temperature and pressure data. When the brew is
 * done the temperature and pressure graphs are displayed.
 */
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
    HEATER_4PCT
};

/*
 * Global variables
 */
display_status_t display_status = DISPLAY_WARMUP_TIMER; // current program mode
heater_action_t heater_action = HEATER_ON; // current heater setting (TODO: replace with PID controller)
volatile uint8_t isr_heater_action = 1; // heater setting for timer OVF ISR
unsigned int log_index = 0; // current log index when in Brewing mode
unsigned long brew_timer = 0; // current time in seconds since entering Brewing mode

//HeaterController heater_controller;

Display display;

Adafruit_MAX31855 thermocouple(PIN_SPI_SCLK, PIN_SPI_CS, PIN_SPI_MISO);

// Debounces button with click, double-click and long-press functionality
using namespace smartbutton;
SmartButton button(PIN_BTN1, SmartButton::InputType::NORMAL_HIGH);

/*
 * Set RGB LED output
 * red: heating
 * green: Ready for brewing
 * blue: inactive/cooling down
 */
void set_rgb_led(uint8_t r, uint8_t g, uint8_t b) {
    digitalWrite(PIN_LED_R, r == 1 ? LOW : HIGH);
    digitalWrite(PIN_LED_G, g == 1 ? LOW : HIGH);
    digitalWrite(PIN_LED_B, b == 1 ? LOW : HIGH);
}

void set_heater_action(heater_action_t h) {
    heater_action = h;
    switch (heater_action) {
        case HEATER_OFF:
            isr_heater_action = 0;
            break;
        case HEATER_ON:
            isr_heater_action = 1;
            break;
        case HEATER_25PCT:
            isr_heater_action = 2;
            break;
        case HEATER_4PCT:
            isr_heater_action = 3;
            break;
    }
}

// TODO: remove when using PID controller
void goto_next_heater_action() {
    switch (heater_action) {
        case HEATER_OFF:
            set_heater_action(HEATER_ON);
            break;
        case HEATER_ON:
            set_heater_action(HEATER_25PCT);
            break;
        case HEATER_25PCT:
            set_heater_action(HEATER_4PCT);
            break;
        case HEATER_4PCT:
            set_heater_action(HEATER_OFF);
            break;
    }
}

/*
 * Set new program mode and adjust global variables accordingly
 */
void goto_mode(display_status_t new_status) {
    display_status = new_status;
    switch (display_status) {
        case DISPLAY_LIVE:
            set_heater_action(HEATER_4PCT);
            // Prepare logging and timer for brewing
            log_index = 0;
            brew_timer = 0;
            break;
        case DISPLAY_BREWING:
            set_heater_action(HEATER_25PCT);
            break;
        case DISPLAY_GRAPH_TEMPERATURE:
            set_heater_action(HEATER_OFF);
            break;
        default:
            Serial.print("ERROR: Invalid new_status ");
            Serial.print((int)new_status);
            Serial.println(" passed to goto_mode");
            break;
    }
}

/*
 * Handle button actions:
 * click: set new heater (TODO: remove when using PID controller), and toggle between temperature and pressure graphs
 * long-press: go to next mode
 */
void button_callback(SmartButton *b, SmartButton::Event event, int clicks) {
    if (event == SmartButton::Event::CLICK) {
        if (clicks == 1) {
            switch (display_status) {
                case DISPLAY_WARMUP_TIMER: // TODO: advance to next mode once PID is active
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
    // Serial output and display
    Serial.begin(115200);
    display.init();

    // Configure timer 1, to be used for driving heater SSR
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

    // Sensors, inputs, and SSR output
    pinMode(PIN_ADC_PRESSURE, INPUT);
    pinMode(PIN_SSR, OUTPUT);
    digitalWrite(PIN_SSR, LOW);
    pinMode(PIN_BTN1, INPUT_PULLUP);
    button.begin(button_callback);
    pinMode(PIN_BREW_SWITCH, INPUT_PULLUP); // externally pulled-up

    if (!thermocouple.begin()) {
        Serial.println("ERROR thermocouple");
        while (1) {};
    }
    Serial.print("Get first thermocouple reading: ");
    Serial.println(thermocouple.readCelsius());
}

void set_ssr(heater_action_t a) {
    if (a == HEATER_ON) {
        digitalWrite(PIN_SSR, HIGH);
        set_rgb_led(1, 0, 0); // TODO: remove once heating is done by PID
    } else if (a == HEATER_OFF) {
        digitalWrite(PIN_SSR, LOW);
        set_rgb_led(0, 0, 0); // TODO: remove once heating is done by PID
    } else {
        // Driving of SSR occurs in TIMER1_OVF ISR
    }
}

volatile bool current_ssr_output = false;

/*
 * Timer 1 overflow interrupt service routine
 * Switches timer value (TCNT1) according to current on-duty and off-duty cycles for the heater.
 *
 * Python program to calculate TCNT value:
 * def calc(p):
 *   timer_clock = cpu/prescale
 *   return (65535 - (p * timer_clock), 65535 - ((1-p) * timer_clock))
 */
ISR(TIMER1_OVF_vect)
{
    if (isr_heater_action > 1) { // > 1 means PWM
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
        } else if (isr_heater_action == 3) { // 4%
            if (current_ssr_output) { // on cycle
                TCNT1 = 64910;
            } else { // off cycle
                TCNT1 = 50535;
            }
        } else {
            TCNT1 = 57723;
        }
    } else {
        TCNT1 = 57723; // default 0.5 s interval
    }
}

/*
 * Fill string buffer with temperature information for warmup, live status and brewing screens
 */
void get_buf_temperature(char *buf_temperature, double temperature) {
    char buf[8];
    dtostrf(temperature, 3, 1, buf);
    sprintf(buf_temperature, "%sC", buf);
}

/*
 * Fill string buffer with pressure information for live status and brewing screens
 */
void get_buf_pressure(char *buf_pressure, double pressure_bar) {
    char buf[8];
    dtostrf(pressure_bar, 2, 1, buf);
    sprintf(buf_pressure, "%s bar", buf);
}

/*
 * Fill string buffer with information for warmup, live status and brewing screens
 */
void get_buf_status(char *buf_status, heater_action_t heater_action) {
    const static char str_on[] = " ON";
    const static char str_25pct[] = "25%";
    const static char str_4pct[] = " 4%";
    const static char str_off[] = "OFF";

    switch (heater_action) {
        case HEATER_ON:
            strcpy(buf_status, str_on);
            break;
        case HEATER_25PCT:
            strcpy(buf_status, str_25pct);
            break;
        case HEATER_4PCT:
            strcpy(buf_status, str_4pct);
            break;
        default:
            strcpy(buf_status, str_off);
            break;
    }
}

/*
 * Fill string buffer with information for warmup and brewing screens
 */
void get_buf_timer(char *buf_timer, unsigned long m) {
    if (display_status == DISPLAY_WARMUP_TIMER) {
        unsigned int m_min = (unsigned int)((m / 1000) / 60);
        unsigned int m_sec = (unsigned int)((m / 1000) % 60);
        sprintf(buf_timer, "%02u:%02u", m_min, m_sec);
    } else { // Brew Status
        sprintf(buf_timer, "%02lu", m);
    }
}

/*
 * The program's main loop
 */
void loop() {
    static char buf_temperature[16], buf_pressure[10], buf_status[8], buf_timer[8];

    static heater_action_t last_heater_action = HEATER_OFF;

    static unsigned long last_update_time = 0;
    static unsigned long last_brew_timer_update = 0;
    const unsigned long m = millis();

    static double temperature = thermocouple.readCelsius();
    const unsigned int pressure_raw = analogRead(PIN_ADC_PRESSURE);

    // Record temperature (in C * 10) every 0.5 s
    static uint8_t temperature_log[RECORD_LOG_SIZE];

    // Record pressure (in bar * 10) every 0.5 s
    static uint8_t pressure_log[RECORD_LOG_SIZE];

    static uint16_t brew_switch_check_counter = 0;
    static bool brew_switch_activated = false;
    static unsigned long brew_switch_activated_time = 0;

    // Handle buttons and switches
    SmartButton::service();

    // Brew switch is a signal from an AC detection circuit (opto-isolated) so it
    // alternates between LOW and HIGH at the AC frequency. If switch is off, signal
    // is continuously HIGH.
    // At 50Hz mains the period is 20 ms; at 60Hz it is 16.7ms.
    if (digitalRead(PIN_BREW_SWITCH) == LOW) { // any reading of LOW means switch is activated
        if (!brew_switch_activated) {
            brew_switch_activated = true; // keep track that BREWING mode was entered because of brew switch
            brew_switch_activated_time = m; // keep track of time and only go to BREWING mode after 5 seconds (ignore flushing or pre-infusion)
        }
        brew_switch_check_counter = 0; // reset every time we see the switch LOW
    } else if (brew_switch_activated) {
        // Need to verify for more than one AC cycle that signal stays high.
        // Program loop frequency is about 1000Hz (1 ms per loop).
        // Need to read for at least 1 full mains cycle = 40 ms.
        // In 40 ms there are approx. 40 program loop cycles unless the code updating the display below is running.
        // Count up to 300 as long as switch input is HIGH and only deactivate brewing if it stayed HIGH the whole time.
        // (1000 to give a short delay when pump is stopped)
        if (brew_switch_check_counter++ > 1000) {
            brew_switch_check_counter = 0;
            brew_switch_activated = false;

            if (display_status == DISPLAY_BREWING) goto_mode(DISPLAY_GRAPH_TEMPERATURE);
        }
    }

    // Set SSR based on HeaterController, not direct from button inputs or measured temperature
    // TODO: Use HeaterController (PID)
    //heater_controller.update(temperature);
    if (heater_action != last_heater_action) {
        set_ssr(heater_action);
        last_heater_action = heater_action;
    }

    if (display_status == DISPLAY_WARMUP_TIMER) {
        // Automatically move from Warmup to Live after 15 minutes
        if (m > AUTO_ADVANCE_TIME_WARMUP_TO_LIVE) goto_mode(DISPLAY_LIVE);
    }

    // Move to Brewing mode 5 seconds after brewing switch got activated
    if (display_status == DISPLAY_WARMUP_TIMER || display_status == DISPLAY_LIVE) {
        if (brew_switch_activated && (m - brew_switch_activated_time) > AUTO_ADVANCE_TIME_LIVE_TO_BREWING) goto_mode(DISPLAY_BREWING);
    }

    // Only update Serial and display every 0.5 s
    // The extra work in here takes approx. 60 ms (measured with oscilloscope)
    if ((unsigned long)(m - last_update_time) > DISPLAY_UPDATE_INTERVAL) {
        // Increment brew timer by one second if in brewing mode
        if (display_status == DISPLAY_BREWING && ((m - last_brew_timer_update) > 1000)) {
            last_brew_timer_update = m;
            brew_timer++;
        }

        // Get temperature
        temperature = thermocouple.readCelsius();
        Serial.println(temperature);

        // PID here?
        // Basic temperature control
        if ((heater_action == HEATER_25PCT || heater_action == HEATER_ON) && temperature >= 98.0) {
            set_heater_action(HEATER_4PCT);
        } else if (heater_action == HEATER_4PCT && temperature < 90.0) {
            set_heater_action(HEATER_25PCT);
        }

        // Pressure conversion
        // Max pressure of sensor is 1.2 Mpa = 174 psi = 12 bar
        // voltage range of sensor is 0.5-4.5V = 4V
        double pressure_V = pressure_raw * (5.0 / 1024.0); // factor is (maxV / maxDigitalValue)
        double pressure_bar = (pressure_V - 0.17) * 3.0; // subtract 1 bar pressure voltage and apply conversion factor

        // Record temperature and pressure to log
        // Temperature: record 0.5C resolution with range from 70 to 120 => 50C range => 100 values
        //   Store as uint8_t. Decode: Treal = Tlog / 2 + 70
        // Pressure: record 0.5 bar resolution with range from 0 to 14 => 28 values
        //   Store as uint8_t. Decode: Preal = Plog / 2
        if (display_status == DISPLAY_BREWING) {
            // Filter out values outside of 10C to 150C range
            if (temperature < 150 && temperature > 10)
                temperature_log[log_index] = (uint8_t)(round(temperature * 2.0)) - 70; // Subtract 70C to normalize temperature range

            // Filter out values outside of 0 to 15 bar range
            if (pressure_bar < 15 && pressure_bar > 0)
                pressure_log[log_index] = (uint8_t)(round(pressure_bar * 2.0));

            log_index++;
            if (log_index >= RECORD_LOG_SIZE) { // 50 s
                goto_mode(DISPLAY_GRAPH_TEMPERATURE);
            }
        }

        // Populate string buffers and sent to display for drawing
        switch (display_status) {
            case DISPLAY_WARMUP_TIMER:
                get_buf_temperature(buf_temperature, temperature);
                get_buf_status(buf_status, heater_action);
                get_buf_timer(buf_timer, m);
                display.draw_warmup_timer(buf_temperature, buf_timer, buf_status);
                break;
            case DISPLAY_LIVE:
                get_buf_temperature(buf_temperature, temperature);
                get_buf_pressure(buf_pressure, pressure_bar);
                get_buf_status(buf_status, heater_action);
                display.draw_live_status(buf_temperature, buf_pressure, buf_status);
                break;
            case DISPLAY_BREWING:
                get_buf_temperature(buf_temperature, temperature);
                get_buf_pressure(buf_pressure, pressure_bar);
                get_buf_status(buf_status, heater_action);
                get_buf_timer(buf_timer, brew_timer);
                display.draw_brew_status(buf_temperature, buf_pressure, buf_status, buf_timer);
                break;
            case DISPLAY_GRAPH_TEMPERATURE:
                display.draw_graph("Temp", temperature_log, log_index, 70, 120);
                break;
            case DISPLAY_GRAPH_PRESSURE:
                display.draw_graph("Pres", pressure_log, log_index, 0, 14);
                break;
        }

        last_update_time = m;
    }
}
