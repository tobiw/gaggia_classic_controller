#include <gaggia_webserver.h>

#define RESPONSE_SIZE 1500
static char buf_response[RESPONSE_SIZE];

GaggiaWebServer *GaggiaWebServer::server = NULL;

GaggiaWebServer* GaggiaWebServer::getInstance() {
    if (GaggiaWebServer::server == NULL) GaggiaWebServer::server = new GaggiaWebServer();
    return GaggiaWebServer::server;
}

static char *convert_temperature_to_str(double *t, char *buf) {
    dtostrf(*t, 3, 1, buf);
    return buf;
}

void http_index() {
    GaggiaWebServer::server->get_http_response_index();
    GaggiaWebServer::server->send(200, "text/html", buf_response);
}

void http_api_temperature() {
    GaggiaWebServer::server->get_http_response_api_temperature();
    GaggiaWebServer::server->send(200, "application/json", buf_response);
}

GaggiaWebServer::GaggiaWebServer() {
}

void GaggiaWebServer::begin(double *temperature, uint8_t *target_temperature, uint8_t *overshoot_guard, char **systemlog) {
    _server = new WebServer(80);
    _temperature = temperature;
    _target_temperature = target_temperature;
    _overshoot_guard = overshoot_guard;
    _systemlog = systemlog;
    _server->begin();
    _server->on("/", http_index);
    _server->on("/temperature", http_api_temperature);
}

void GaggiaWebServer::service() {
    _server->handleClient();
}

void GaggiaWebServer::send(int code, char *type, char *payload) {
    _server->send(code, type, payload);
}

void GaggiaWebServer::get_http_response_index() {
    char buf[8];
    sprintf(buf_response, "<html><head><title>Gaggia Classic</title></head><body><h1>Gaggia Classic</h1>Temperature: %s</body></html>",
            convert_temperature_to_str(_temperature, buf));
}

void GaggiaWebServer::get_http_response_api_temperature() {
    char buf[8];
    sprintf(buf_response, "{ \"temperature\": { \"current\": %s, \"target\": %u, \"overshoot_guard\": %u } }",
            convert_temperature_to_str(_temperature, buf), *_target_temperature, *_overshoot_guard);
}
