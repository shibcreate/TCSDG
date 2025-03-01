#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "ra01s.h"

#define BUTTON_GPIO 34

static const char *TAG = "TELEMETRY_MASTER";
static TaskHandle_t task_master_handle = NULL;
static TaskHandle_t task_lora_receive_handle = NULL;

// Function to send a message with a specific label and value
void send_lora_message(const char *label, int value) {
    uint8_t txData[256];
    int txLen = snprintf((char *)txData, sizeof(txData), "%s: %d", label, value);
    
    ESP_LOGI(TAG, "Sending: %s", txData);
    LoRaSend(txData, txLen, SX126x_TXMODE_SYNC);
}

// Example telemetry messages
void send_telemetry_data() {
    send_lora_message("EMeter_Current", rand() % 100);
    send_lora_message("EMeter_Voltage", rand() % 100);
    send_lora_message("MCM_Motor_Speed", rand() % 5000);
    send_lora_message("MCM_DC_Bus_Current", rand() % 200);
    send_lora_message("MCM_Torque_Feedback", rand() % 400);
    send_lora_message("MCM_Commanded_Torque", rand() % 400);
    send_lora_message("TPS0ThrottlePercent0FF", rand() % 100);
    send_lora_message("VCU_WSS_FL", rand() % 300);
    send_lora_message("VCU_WSS_FR", rand() % 300);
    send_lora_message("VCU_WSS_RR", rand() % 300);
    send_lora_message("VCU_WSS_RL", rand() % 300);
    send_lora_message("Steering_Angle", rand() % 360);
    send_lora_message("Pack_Voltage", rand() % 600);
}

// Compact VCU and BMS faults/warnings
void send_fault_warnings() {
    uint8_t vcu_faults = rand() & 0xFF;
    uint8_t bms_faults = rand() & 0xFF;
    send_lora_message("VCU_FAULTS", vcu_faults);
    send_lora_message("BMS_FAULTS", bms_faults);
}

// Task for sending telemetry data
void task_master(void *pvParameters) {
    ESP_LOGI(TAG, "Master Task Started");
    while (1) {
        send_telemetry_data();
        send_fault_warnings();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task for receiving LoRa messages
void task_lora_receive(void *pvParameters) {
    ESP_LOGI(TAG, "Listening for LoRa Messages");
    while (1) {
        uint8_t rxData[256];
        uint8_t rxLen = LoRaReceive(rxData, sizeof(rxData));
        if (rxLen > 0) {
            ESP_LOGI(TAG, "Received %d bytes: [%.*s]", rxLen, rxLen, rxData);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Button monitor task
void task_button_monitor(void *pvParameters) {
    gpio_reset_pin(BUTTON_GPIO);
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);

    bool last_state = gpio_get_level(BUTTON_GPIO);
    while (1) {
        bool state = gpio_get_level(BUTTON_GPIO);
        if (state == 0 && last_state == 1) { // Button pressed
            ESP_LOGI(TAG, "Button Pressed: Entering Receive Mode");
            vTaskSuspend(task_master_handle);
            xTaskCreate(&task_lora_receive, "LORA_RECEIVE", 4096, NULL, 5, &task_lora_receive_handle);
        } else if (state == 1 && last_state == 0) { // Button released
            ESP_LOGI(TAG, "Button Released: Entering Send Mode");
            if (task_lora_receive_handle) {
                vTaskDelete(task_lora_receive_handle);
                task_lora_receive_handle = NULL;
            }
            vTaskResume(task_master_handle);
        }
        last_state = state;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main() {
    LoRaInit();
    int8_t txPowerInDbm = 22;
    uint32_t frequencyInHz = 915000000;
    float tcxoVoltage = 3.3;
    bool useRegulatorLDO = true;

    ESP_LOGI(TAG, "Initializing LoRa at 915MHz");
    if (LoRaBegin(frequencyInHz, txPowerInDbm, tcxoVoltage, useRegulatorLDO) != 0) {
        ESP_LOGE(TAG, "LoRa module not recognized");
        while (1) {
            vTaskDelay(1);
        }
    }
    LoRaConfig(7, 4, 1, 8, 0, true, false);

    // Start tasks
    xTaskCreate(&task_master, "MASTER", 4096, NULL, 5, &task_master_handle);
    xTaskCreate(&task_button_monitor, "BUTTON_MONITOR", 4096, NULL, 5, NULL);
}