#ifndef __GAGGIA_WEBSERVER_H__
#define __GAGGIA_WEBSERVER_H__

#include <WebServer.h>
#include <systemlog.h>

class GaggiaWebServer {
    public:
        static GaggiaWebServer *getInstance();

        void begin(double *temperature, uint8_t *target_temperature, uint8_t *overshoot_guard);
        void service();
        void send(int code, char *type, char *payload);
        String get_arg(int i);
        void get_http_response_index();
        void get_http_response_api_temperature();
        void get_http_response_api_systemlog();
        void handle_http_post_set_pwm(int pwm);
        void handle_http_post_set_pid(int p, int i, int d);
        void handle_http_post_set_temperature(int temp);
        void handle_http_post_set_target_temperature(int temp);

        static GaggiaWebServer *server;

    private:
        GaggiaWebServer();

        WebServer *_server;
        double *_temperature;
        uint8_t *_target_temperature;
        uint8_t *_overshoot_guard;
};

#endif
