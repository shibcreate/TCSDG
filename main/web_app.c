#include "web_app.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MAIN_TASK_PERIOD (5000)

void task_web_server(void *pv) {
    while (1) {
        vTaskDelay(MAIN_TASK_PERIOD/portTICK_PERIOD_MS);
    }
}