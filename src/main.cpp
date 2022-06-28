#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "SmartButton.h"

#ifdef ESP32
#include <WiFi.h>
#include <WiFiUdp.h>
#include <gaggia_webserver.h>
#include <ESP32_PWM.h>
#endif

#include <application.h>

/*
 * Pin definitions for inputs and outputs
 */
#ifdef ATMEGA32
#include "pin_defines_atmega32.h"
#endif
#ifdef ESP32
#include "pin_defines_esp32.h"
#endif

/*
 * Global constants definitions
 */
#define DISPLAY_UPDATE_INTERVAL 500 // used in the main loop to only update the display periodically
#define RECORD_LOG_SIZE 120 // fixed size for the temperature and pressure logs
#define AUTO_ADVANCE_TIME_WARMUP_TO_LIVE 900000 // move from Warmup to Live Status after 15 minutes
#define AUTO_ADVANCE_TIME_LIVE_TO_BREWING 5000 // move from Live Status to Brewing after 5 seconds (if brew switch activated)

const char *ssid = "...";
const char *wpa_key = "...";


/*
 * Global variables
 */
unsigned int log_index = 0; // current log index when in Brewing mode
unsigned long brew_timer = 0; // current time in seconds since entering Brewing mode
uint8_t target_temperature = 94;
uint8_t temperature_overshoot_guard = 3;
double temperature = 0.0;

#ifdef ESP32
ESP32Timer timer1(1);
ESP32_PWM isr_pwm;
int isr_pwm_channel = -1;
#endif

#ifdef ATMEGA32
const bool RGB_LED_ACTIVE_LOW = true;
#else
const bool RGB_LED_ACTIVE_LOW = false;
#endif

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
    digitalWrite(PIN_LED_R, r == ((int)RGB_LED_ACTIVE_LOW) ? LOW : HIGH);
    digitalWrite(PIN_LED_G, g == ((int)RGB_LED_ACTIVE_LOW) ? LOW : HIGH);
    digitalWrite(PIN_LED_B, b == ((int)RGB_LED_ACTIVE_LOW) ? LOW : HIGH);
}

/*
 * Handle button actions:
 * click: set new target temperature, and toggle between temperature and pressure graphs
 * long-press: go to next mode
 */
void button_callback(SmartButton *b, SmartButton::Event event, int clicks) {
    if (event == SmartButton::Event::CLICK) {
        if (clicks == 1) {
            switch (app()->get_current_display_status()) {
                case DISPLAY_WARMUP_TIMER:
                    app()->goto_mode(DISPLAY_LIVE);
                    break;
                case DISPLAY_LIVE: // only change target temperature on Live Status or Brewing pages
                case DISPLAY_BREWING:
                    target_temperature += 2;
                    if (target_temperature > 99) target_temperature = 86;
                    app()->heatercontroller()->set_target(target_temperature);
                    break;
                case DISPLAY_GRAPH_TEMPERATURE:
                case DISPLAY_GRAPH_PRESSURE:
                    app()->goto_mode(DISPLAY_LIVE);
                    break;
            }
        }
    } else if (event == SmartButton::Event::HOLD) {
        switch (app()->get_current_display_status()) {
            case DISPLAY_WARMUP_TIMER:
                app()->goto_mode(DISPLAY_LIVE);
                break;
            case DISPLAY_LIVE:
                app()->goto_mode(DISPLAY_BREWING);
                break;
            case DISPLAY_BREWING:
                app()->goto_mode(DISPLAY_GRAPH_TEMPERATURE);
                break;
            case DISPLAY_GRAPH_TEMPERATURE:
            case DISPLAY_GRAPH_PRESSURE:
                app()->goto_mode(DISPLAY_LIVE);
                break;
            //case DISPLAY_WARMUP_TIMER:
            default:
                break;
        }
    }
}

char *convert_temperature_to_str(char *buf) {
    dtostrf(temperature, 3, 1, buf);
    return buf;
}

bool IRAM_ATTR TimerHandler(void *timerNo) {
    isr_pwm.run();
    return true;
}

void setup() {
    // Sensors, inputs, and SSR output
    pinMode(PIN_ADC_PRESSURE, INPUT);
    pinMode(PIN_SSR, OUTPUT);
    digitalWrite(PIN_SSR, LOW);
    pinMode(PIN_BTN1, INPUT_PULLUP);
    button.begin(button_callback);
    pinMode(PIN_BREW_SWITCH, INPUT_PULLUP); // externally pulled-up

    // Timer-driven PWM output for heater SSR control
#ifdef ESP32
    timer1.attachInterruptInterval(20 /* us */, TimerHandler);
#endif

    // Serial output and display
    Serial.begin(115200);
    app()->display()->init();  // TODO: move into app->init

#ifdef ESP32
    Serial.print("Connecting to WiFi ");
    WiFi.begin(ssid, wpa_key);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.print("WiFi connected: ");
    Serial.println(WiFi.localIP());

    app()->udp()->begin(WiFi.localIP(), 6777);  // TODO: move into app->init
    app()->send_udp_packet("\nSTART\n");

    // TODO: move into app->init
    app()->webserver()->begin(&temperature, &target_temperature, &temperature_overshoot_guard);
#endif

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

    if (!thermocouple.begin() || thermocouple.readCelsius() == 0.0) {
        Serial.println("ERROR thermocouple");
    }

    app()->heatercontroller()->set_target(target_temperature);
}

void set_ssr(uint8_t pwm_percent) {
#ifdef ESP32
    // Set PWM frequency to 10 Hz so that mains voltage can swing multiple times.
    // Set duty cycle according to how much to enable heater.
    if (pwm_percent < 10) {
        if (isr_pwm_channel > -1) {
            isr_pwm.deleteChannel(isr_pwm_channel);
            isr_pwm_channel = -1;
        }
        digitalWrite(PIN_SSR, LOW);
    } else if (isr_pwm_channel == -1) {
        isr_pwm_channel = isr_pwm.setPWM(PIN_SSR, 5, pwm_percent);
    } else {
        isr_pwm.modifyPWMChannel(isr_pwm_channel, PIN_SSR, 5, pwm_percent);
    }
#else
    digitalWrite(PIN_SSR, pwm_percent == 0 ? LOW : HIGH);
#endif
    set_rgb_led(pwm_percent == 0 ? 0 : 1, 0, 0);
}

/*
 * Fill string buffer with temperature information for warmup, live status and brewing screens
 */
void get_buf_temperature(char *buf_temperature, double temperature) {
    char buf[8];
    sprintf(buf_temperature, "%sC [%uC]", convert_temperature_to_str(buf), target_temperature);
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
 * Fill string buffer with information for warmup and brewing screens
 */
void get_buf_timer(char *buf_timer, unsigned long m) {
    if (app()->get_current_display_status() == DISPLAY_WARMUP_TIMER) {
        unsigned int m_min = (unsigned int)((m / 1000) / 60);
        unsigned int m_sec = (unsigned int)((m / 1000) % 60);
        sprintf(buf_timer, "%02u:%02u", m_min, m_sec);
    } else { // Brew Status
        sprintf(buf_timer, "%02lu", m);
    }
}

void get_buf_status(char *buf_status) {
    sprintf(buf_status, "PWM%%: %u", (uint8_t)app()->heatercontroller()->get_output());
}

void set_fake_temperature(double temp) {
    temperature = temp;
}

void set_target_temperature(double temp) {
    target_temperature = temp;
    app()->heatercontroller()->set_target(target_temperature); // TODO: store in EEPROM/SPIFFS
}

void set_pid_parameters(double p, double i, double d) {
    app()->heatercontroller()->set_pid_parameters(p, i, d); // TODO: store in EEPROM/SPIFFS
}

bool get_log(unsigned char i, char *buf) {
    return app()->systemlog()->get_log(i, buf);
}

/*
 * The program's main loop
 */
void loop() {
    static char buf_temperature[16], buf_pressure[10], buf_status[10], buf_timer[8];

    static unsigned long last_update_time = 0;
    static unsigned long last_brew_timer_update = 0;
    static unsigned long last_graph_switch_time = 0;
    const unsigned long m = millis();

    static double last_temperature = 0;
    const unsigned int pressure_raw = analogRead(PIN_ADC_PRESSURE);

    // Record temperature (in C * 10) every 0.5 s
    static uint8_t temperature_log[RECORD_LOG_SIZE];

    // Record pressure (in bar * 10) every 0.5 s
    static uint8_t pressure_log[RECORD_LOG_SIZE];
    static uint8_t update_cycle = 0;

    static uint16_t brew_switch_check_counter = 0;
    static bool brew_switch_activated = false;
    static unsigned long brew_switch_activated_time = 0;

    if (app()->get_current_display_status() == DISPLAY_ERROR) {
        while(1) {};
        return;
    }

    // Handle buttons and switches
    SmartButton::service();

#ifdef ESP32
    // Handle HTTP clients
    app()->webserver()->service();
#endif

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

            if (app()->get_current_display_status() == DISPLAY_BREWING) app()->goto_mode(DISPLAY_GRAPH_TEMPERATURE);
        }
    }

    // Set SSR based on HeaterController, not direct from button inputs or measured temperature
    if (temperature == 0) {
        temperature = thermocouple.readCelsius();
    }
    app()->heatercontroller()->update(temperature);

    if (app()->get_current_display_status() == DISPLAY_WARMUP_TIMER) {
        // Automatically move from Warmup to Live after 15 minutes
        if (m > AUTO_ADVANCE_TIME_WARMUP_TO_LIVE) app()->goto_mode(DISPLAY_LIVE);
    }

    // Move to Brewing mode 5 seconds after brewing switch got activated
    if (app()->get_current_display_status() == DISPLAY_WARMUP_TIMER || app()->get_current_display_status() == DISPLAY_LIVE) {
        if (brew_switch_activated && (m - brew_switch_activated_time) > AUTO_ADVANCE_TIME_LIVE_TO_BREWING) app()->goto_mode(DISPLAY_BREWING);
    }

    // Only update Serial and display every 0.5 s
    // The extra work in here takes approx. 60 ms (measured with oscilloscope)
    if ((unsigned long)(m - last_update_time) > DISPLAY_UPDATE_INTERVAL) {
        // Increment brew timer by one second if in brewing mode
        if (app()->get_current_display_status() == DISPLAY_BREWING && ((m - last_brew_timer_update) > 1000)) {
            last_brew_timer_update = m;
            brew_timer++;
        }

        set_ssr((uint8_t)app()->heatercontroller()->get_output());
        {
        char buf_udp[64];
        sprintf(buf_udp, "T:%d P:%u\n", (int)temperature, (uint8_t)app()->heatercontroller()->get_output());
        app()->send_udp_packet(buf_udp);
        }

        // Get temperature
        temperature = thermocouple.readCelsius();
        if (isnan(temperature) || temperature < 10.0) {
            delay(1);
            temperature = thermocouple.readCelsius();
            if (isnan(temperature) || temperature < 10.0) {
                temperature = last_temperature; // fall back to reusing last temperature reading
            }
        }

        bool temp_rising = last_temperature < temperature;

        last_temperature = temperature;

        // Log current state
        {
        char buf_serial[64], buf_temp[8];
        sprintf(buf_serial, "[%u] T: %sC [>%uC, -%uC, ^%u] PWM: %u",
                (unsigned int)(m/1000),
                convert_temperature_to_str(buf_temp), target_temperature, temperature_overshoot_guard, temp_rising ? 1 : 0, (uint8_t)app()->heatercontroller()->get_output());
        app()->systemlog()->log(buf_serial);
        }

        // Pressure conversion
        // Max pressure of sensor is 1.2 Mpa = 174 psi = 12 bar
        // voltage range of sensor is 0.5-4.5V = 4V
#ifdef ESP32
#define ADC_MAX_V 3.3
#else
#define ADC_MAX_V 5.0
#endif
        double pressure_V = pressure_raw * (ADC_MAX_V / 1024.0); // factor is (maxV / maxDigitalValue)
        double pressure_bar = (pressure_V - 0.17) * 3.0; // subtract 1 bar pressure voltage and apply conversion factor

        // Record temperature and pressure to log
        // Temperature: record 0.5C resolution with range from 70 to 120 => 50C range => 100 values
        //   Store as uint8_t. Decode: Treal = Tlog / 2 + 70
        // Pressure: record 0.5 bar resolution with range from 0 to 14 => 28 values
        //   Store as uint8_t. Decode: Preal = Plog / 2
        if (app()->get_current_display_status() == DISPLAY_BREWING && (update_cycle++) >= 1) {
            update_cycle = 0;

            // Filter out values outside of 60C to 150C range
            if (temperature < 150 && temperature > 60)
                temperature_log[log_index] = (uint8_t)(round((temperature - 70) * 2.0)); // Subtract 70C to normalize temperature range

            // Filter out values outside of 0 to 15 bar range
            if (pressure_bar < 15 && pressure_bar > 0)
                pressure_log[log_index] = (uint8_t)(round(pressure_bar * 2.0));

            log_index++;
            if (log_index >= RECORD_LOG_SIZE) { // 50 s
                app()->goto_mode(DISPLAY_GRAPH_TEMPERATURE);
            }
        }

        // Populate string buffers and sent to display for drawing
        switch (app()->get_current_display_status()) {
            case DISPLAY_WARMUP_TIMER:
                get_buf_temperature(buf_temperature, temperature);
                get_buf_timer(buf_timer, m);
                get_buf_status(buf_status);
                app()->display()->draw_warmup_timer(buf_temperature, buf_timer, buf_status);
                break;
            case DISPLAY_LIVE:
                get_buf_temperature(buf_temperature, temperature);
                get_buf_pressure(buf_pressure, pressure_bar);
                get_buf_status(buf_status);
                app()->display()->draw_live_status(buf_temperature, buf_pressure, buf_status);
                break;
            case DISPLAY_BREWING:
                get_buf_temperature(buf_temperature, temperature);
                get_buf_pressure(buf_pressure, pressure_bar);
                get_buf_timer(buf_timer, brew_timer);
                app()->display()->draw_brew_status(buf_temperature, buf_pressure, buf_status, buf_timer);
                break;
            case DISPLAY_GRAPH_TEMPERATURE:
                app()->display()->draw_graph("Temp", temperature_log, log_index, 70, 120);
                if ((m - last_graph_switch_time) > 3000) {
                    app()->goto_mode(DISPLAY_GRAPH_PRESSURE);
                    last_graph_switch_time = m;
                }
                break;
            case DISPLAY_GRAPH_PRESSURE:
                app()->display()->draw_graph("Pres", pressure_log, log_index, 0, 14);
                if ((m - last_graph_switch_time) > 3000) {
                    app()->goto_mode(DISPLAY_GRAPH_TEMPERATURE);
                    last_graph_switch_time = m;
                }
                break;
        }

        last_update_time = m;
    }
}
