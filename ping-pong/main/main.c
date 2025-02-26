#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ra01s.h"

static const char *TAG = "TELEMETRY_MASTER";

// Function to send a message with a specific label and value
void send_lora_message(const char *label, int value) {
    uint8_t txData[256];
    int txLen = snprintf((char *)txData, sizeof(txData), "%s: %d", label, value);
    
    ESP_LOGI(TAG, "Sending: %s", txData);
    LoRaSend(txData, txLen, SX126x_TXMODE_SYNC);
}

// Function to send MCM_Voltage_Info
void send_mcm_voltage_info() {
    int voltage = rand() % 100; // Example: Voltage between 0-99
    send_lora_message("MCM_Voltage_Info", voltage);
}

// Function to send Ground_Speed
void send_ground_speed() {
    int speed = rand() % 200;   // Example: Speed between 0-199
    send_lora_message("Ground_Speed", speed);
}

// Main task to handle telemetry data and communication
void task_master(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start");

    while (1) {
        // **Listen for incoming messages**
        uint8_t rxData[256];
        uint8_t rxLen = LoRaReceive(rxData, sizeof(rxData));
        if (rxLen > 0) {
            ESP_LOGI(TAG, "Received %d byte packet: [%.*s]", rxLen, rxLen, rxData);
        }

        // Send telemetry data
        send_mcm_voltage_info();  // Send MCM_Voltage_Info
        vTaskDelay(pdMS_TO_TICKS(500)); // Small delay between messages

        send_ground_speed();  // Send Ground_Speed
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay before next iteration
    }
}

void app_main()
{
    // Initialize LoRa
    LoRaInit();
    int8_t txPowerInDbm = 22;
    uint32_t frequencyInHz = 915000000; // Default to 915 MHz

    ESP_LOGI(TAG, "Frequency is 915MHz");

    float tcxoVoltage = 3.3;  // Enable TCXO
    bool useRegulatorLDO = true;

    if (LoRaBegin(frequencyInHz, txPowerInDbm, tcxoVoltage, useRegulatorLDO) != 0) {
        ESP_LOGE(TAG, "LoRa module not recognized");
        while (1) {
            vTaskDelay(1);
        }
    }

    LoRaConfig(7, 4, 1, 8, 0, true, false);

    // Start master task
    xTaskCreate(&task_master, "MASTER", 1024 * 4, NULL, 5, NULL);
}