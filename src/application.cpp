#include <application.h>

Application *app_instance = NULL;

Application *app() {
    if (app_instance == NULL) {
        app_instance = new Application();
    }
    return app_instance;
}

Application::Application() {
    display_status = DISPLAY_WARMUP_TIMER;
    log_index = 0;
    brew_timer = 0;
}

/*
 * Set new program mode and adjust global variables accordingly
 */
void Application::goto_mode(display_status_t new_status) {
    display_status = new_status;
    switch (display_status) {
        case DISPLAY_LIVE:
            // Prepare logging and timer for brewing
            log_index = 0;
            brew_timer = 0;
            break;
        case DISPLAY_BREWING:
            break;
        case DISPLAY_GRAPH_TEMPERATURE:
        case DISPLAY_GRAPH_PRESSURE:
            break;
        default:
            Serial.print("ERROR: Invalid new_status ");
            Serial.print((int)new_status);
            Serial.println(" passed to goto_mode");
            break;
    }
}
