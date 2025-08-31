#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_app.h"
#include "uart_app.h"
#include "lora_app.h"
#include "web_app.h"

void app_main(void) {
    // Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_app_start();
    init_uart();

    if (init_lora() != 0) {
        ESP_LOGE("MAIN", "LoRa module not recognized!");
        while (1) vTaskDelay(1);
    }

    // Start tasks
    xTaskCreatePinnedToCore(&task_lora, "LORA_TASK", 4096, NULL, 7, NULL, 1);
    xTaskCreatePinnedToCore(&task_web_server, "WEB_SERVER_TASK", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(&task_uart, "UART_TASK", 4096, NULL, 6, NULL, 0);
}