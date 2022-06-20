#ifndef __GAGGIA_WEBSERVER_H__
#define __GAGGIA_WEBSERVER_H__

#include <WebServer.h>

class GaggiaWebServer {
    public:
        static GaggiaWebServer *getInstance();

        void begin(double *temperature, uint8_t *target_temperature, uint8_t *overshoot_guard, char **systemlog);
        void service();
        void send(int code, char *type, char *payload);
        void get_http_response_index();
        void get_http_response_api_temperature();
        void get_http_response_api_systemlog();

        static GaggiaWebServer *server;

    private:
        GaggiaWebServer();

        WebServer *_server;
        double *_temperature;
        uint8_t *_target_temperature;
        uint8_t *_overshoot_guard;
        char **_systemlog;
};

#endif
