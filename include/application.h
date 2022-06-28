#ifndef __APPLICATION_H__
#define __APPLICATION_H__

#include <display.h>
#include <gaggia_webserver.h>
#include <heater_controller.h>
#include <systemlog.h>

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
    DISPLAY_ERROR,
};

class Application {
    public:
        Application();

        void goto_mode(display_status_t new_status);
        display_status_t get_current_display_status() { return display_status; }

        Systemlog *systemlog() { return &_systemlog; }
        GaggiaWebServer *webserver() { return GaggiaWebServer::getInstance(); }
        HeaterController *heatercontroller() { return &_heater_controller; }
        Display *display() { return &_display; }

#ifdef ESP32
        WiFiUDP *udp() { return &_udp; }
        inline void send_udp_packet(char *data) { _udp.beginPacket("10.1.1.21", 6778); _udp.printf(data); _udp.endPacket(); }
#endif

    private:
        display_status_t display_status;

        Systemlog _systemlog;
        HeaterController _heater_controller;
        Display _display;

#ifdef ESP32
        WiFiUDP _udp;
        GaggiaWebServer *_server;
#endif

        unsigned int log_index; // current log index when in Brewing mode TODO: move to separate class for data log management
        unsigned long brew_timer; // current time in seconds since entering Brewing mode
};

Application *app();

#endif
