#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
//#include "esp_system.h"
//#include "sdkconfig.h"

//#include "pid_controller.h"

//PidController pid;

#define GPIO_SSR GPIO_NUM_2

void set_ssr(int enable) {
    const int output = enable == 1 ? 1 : 0;
    printf("Setting SSR to %d (GPIO to %d)\n", enable, output);
    gpio_set_level(GPIO_SSR, output);
}

extern "C" void app_main() {
    gpio_pad_select_gpio(GPIO_SSR);
    gpio_set_direction(GPIO_SSR, GPIO_MODE_OUTPUT);

    while (1) {
        printf("Turning on\n");
        set_ssr(1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("Turning off\n");
        set_ssr(0);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
