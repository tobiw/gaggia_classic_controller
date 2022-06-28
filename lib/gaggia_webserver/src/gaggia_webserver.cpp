#include <gaggia_webserver.h>

// TODO: replace with better design
void set_ssr(uint8_t pwm_percent);
void set_fake_temperature(double temp);
void set_target_temperature(double temp);
void set_pid_parameters(double p, double i, double d);
bool get_log(unsigned char i, char *buf);

#define RESPONSE_SIZE 1500
static char buf_response[RESPONSE_SIZE];

GaggiaWebServer *GaggiaWebServer::server = NULL;

GaggiaWebServer* GaggiaWebServer::getInstance() {
    if (GaggiaWebServer::server == NULL) GaggiaWebServer::server = new GaggiaWebServer();
    return GaggiaWebServer::server;
}

void http_index() {
    GaggiaWebServer::server->get_http_response_index();
    GaggiaWebServer::server->send(200, "text/html", buf_response);
}

void http_api_temperature() {
    GaggiaWebServer::server->get_http_response_api_temperature();
    GaggiaWebServer::server->send(200, "application/json", buf_response);
}

void http_api_systemlog() {
    GaggiaWebServer::server->get_http_response_api_systemlog();
    GaggiaWebServer::server->send(200, "text/html", buf_response);
}

void http_api_post_set_pwm() {
    GaggiaWebServer::server->handle_http_post_set_pwm(atoi(GaggiaWebServer::server->get_arg(0).c_str()));
    GaggiaWebServer::server->send(200, "text/plain", "OK");
}

void http_api_post_set_pid() {
    GaggiaWebServer::server->handle_http_post_set_pid(
        atoi(GaggiaWebServer::server->get_arg(0).c_str()),
        atoi(GaggiaWebServer::server->get_arg(1).c_str()),
        atoi(GaggiaWebServer::server->get_arg(2).c_str()));
    GaggiaWebServer::server->send(200, "text/plain", "OK");
}

void http_api_post_set_temperature() {
    GaggiaWebServer::server->handle_http_post_set_temperature(atoi(GaggiaWebServer::server->get_arg(0).c_str()));
    GaggiaWebServer::server->send(200, "text/plain", "OK");
}

void http_api_post_set_target_temperature() {
    GaggiaWebServer::server->handle_http_post_set_target_temperature(atoi(GaggiaWebServer::server->get_arg(0).c_str()));
    GaggiaWebServer::server->send(200, "text/plain", "OK");
}

GaggiaWebServer::GaggiaWebServer() {
}

void GaggiaWebServer::begin(double *temperature, uint8_t *target_temperature, uint8_t *overshoot_guard) {
    _server = new WebServer(80);
    _temperature = temperature;
    _target_temperature = target_temperature;
    _overshoot_guard = overshoot_guard;
    _server->begin();
    _server->on("/", http_index);
    _server->on("/temperature", http_api_temperature);
    _server->on("/systemlog", http_api_systemlog);
    _server->on("/set_pwm", HTTP_POST, http_api_post_set_pwm);
    _server->on("/set_pid", HTTP_POST, http_api_post_set_pid);
    _server->on("/set_temp", HTTP_POST, http_api_post_set_temperature);
    _server->on("/set_target_temp", HTTP_POST, http_api_post_set_target_temperature);
}

void GaggiaWebServer::service() {
    _server->handleClient();
}

void GaggiaWebServer::send(int code, char *type, char *payload) {
    _server->send(code, type, payload);
}

String GaggiaWebServer::get_arg(int i) {
    return _server->arg(i);
}

void GaggiaWebServer::get_http_response_index() {
    char buf[8];
    dtostrf(*_temperature, 3, 1, buf);
    sprintf(buf_response, "<html><head><title>Gaggia Classic</title></head><body><h1>Gaggia Classic</h1>Temperature: %s</body></html>", buf);
}

void GaggiaWebServer::get_http_response_api_temperature() {
    char buf[8];
    dtostrf(*_temperature, 3, 1, buf);
    sprintf(buf_response, "{ \"temperature\": { \"current\": %s, \"target\": %u, \"overshoot_guard\": %u } }",
            buf, *_target_temperature, *_overshoot_guard);
}

void GaggiaWebServer::get_http_response_api_systemlog() {
    char buf[SYSTEMLOG_ENTRY_SIZE];
    uint8_t i;

    strcpy(buf_response, "<html><head></head><body><h1>Log</h1><ul>");
    while (get_log(i++, buf)) {
        strcat(buf_response, "<li>");
        strcat(buf_response, buf);
        strcat(buf_response, "</li>");
    }
    strcat(buf_response, "</ul></body></html>");
}

void GaggiaWebServer::handle_http_post_set_pwm(int pwm) {
    set_ssr(pwm);
}

void GaggiaWebServer::handle_http_post_set_pid(int p, int i, int d) {
    set_pid_parameters(p / 10.0, i / 10.0, d / 10.0);
}

void GaggiaWebServer::handle_http_post_set_temperature(int temp) {
    set_fake_temperature(temp / 10.0);
}

void GaggiaWebServer::handle_http_post_set_target_temperature(int temp) {
    set_target_temperature(temp / 10.0);
}
