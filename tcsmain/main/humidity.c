// humidity.c
#include "humidity.h"
#include "dht.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "telemetry.h"   

// Hardcoded sensor type
#define SENSOR_TYPE DHT_TYPE_AM2301

// Hardcoded GPIO pin for the data line
#define DHT_GPIO 45  

static void dht_task(void *pvParameters)
{
    float temperature = 0.0f;
    float humidity = 0.0f;

    while (1) {
        if (dht_read_float_data(SENSOR_TYPE, DHT_GPIO, &humidity, &temperature) == ESP_OK) {
            printf("Humidity: %.1f%% Temp: %.1fC\n", humidity, temperature);
            update_telemetry_value_by_name("Temperature", temperature);
            update_telemetry_value_by_name("Humidity", humidity);
        } else {
            printf("Could not read data from sensor\n");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void humidity_start_task(void)
{
    xTaskCreatePinnedToCore(
        dht_task,
        "dht_task",
        configMINIMAL_STACK_SIZE * 3,
        NULL,
        6,
        NULL,
        1
    );
}
