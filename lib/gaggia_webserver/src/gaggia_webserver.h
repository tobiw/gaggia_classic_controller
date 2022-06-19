#ifndef __GAGGIA_WEBSERVER_H__
#define __GAGGIA_WEBSERVER_H__

#include <WebServer.h>

class GaggiaWebServer {
    public:
        static GaggiaWebServer *getInstance();

        void begin(double *temperature);
        void service();
        void send(int code, char *type, char *payload);
        void get_http_response_index(char *buf_response);
        void get_http_response_api_temperature(char *buf_response);

        static GaggiaWebServer *server;

    private:
        GaggiaWebServer();

        WebServer *_server;
        double *_temperature;
};

#endif
