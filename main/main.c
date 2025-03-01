#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_app.h"
#include "ra01s.h"
#include "driver/uart.h"

#define MAIN_TASK_PERIOD (5000)
#define UART_PORT_NUM      (UART_NUM_0)  // UART port for USB-C (UART0 is usually default)
#define UART_BAUD_RATE     (115200)
#define UART_BUF_SIZE      (1024)

// Private Variables
static uint8_t Ground_Speed = 0u;
static uint8_t MCM_Voltage_Info = 0u;

static const char *TAG_MAIN = "MAIN";
static const char *TAG_SECONDARY = "SECONDARY";

// Function to send DRS message with the label and value
void send_drs_message(int drsMode) {
    uint8_t txData[256]; // Max Payload size
    int txLen = sprintf((char *)txData, "DRS: %d", drsMode);

    if (LoRaSend(txData, txLen, SX126x_TXMODE_SYNC)) {
        ESP_LOGI(TAG_SECONDARY, "Sent DRS: %d", drsMode);
    } else {
        ESP_LOGE(TAG_SECONDARY, "Failed to send DRS");
    }
}

// Function to parse LoRa messages and update variables
void parse_lora_message(uint8_t *rxData, uint8_t rxLen) {
    char message[256];
    strncpy(message, (char *)rxData, rxLen);
    message[rxLen] = '\0';  // Null-terminate the string

    // Check if the message contains MCM_Voltage_Info
    if (strstr(message, "Pack_Voltage") != NULL) {
        int value = 0;
        if (sscanf(message, "Pack_Voltage: %d", &value) == 1) {
            MCM_Voltage_Info = value;  // Update MCM_Voltage_Info with the extracted value
            ESP_LOGI(TAG_SECONDARY, "Updated MCM_Voltage_Info to %d", MCM_Voltage_Info);
        }
    }
    // Check if the message contains Ground_Speed
    else if (strstr(message, "MCM_Motor_Speed") != NULL) {
        int value = 0;
        if (sscanf(message, "MCM_Motor_Speed: %d", &value) == 1) {
            Ground_Speed = value;  // Update Ground_Speed with the extracted value
            ESP_LOGI(TAG_SECONDARY, "Updated Ground_Speed to %d", Ground_Speed);
        }
    }
}

// Function to initialize UART for USB-C serial communication
void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Configure UART with the above settings
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));
}

// UART task to read input and send LoRa messages
// Add this global flag
static bool is_transmitting = false;

void task_uart(void *pvParameters) {
    uint8_t data[128];

    while (1) {
        int length = uart_read_bytes(UART_PORT_NUM, data, sizeof(data), 20 / portTICK_PERIOD_MS);

        if (length > 0) {
            data[length] = '\0';
            ESP_LOGI(TAG_MAIN, "Received data: %s", data);

            if (data[0] == '1') {
                is_transmitting = true;  // Enable continuous transmission
                ESP_LOGI(TAG_MAIN, "Switched to continuous transmission mode.");
            } else if (data[0] == '0') {
                is_transmitting = false; // Re-enable receiving mode
                ESP_LOGI(TAG_MAIN, "Switched back to receive mode.");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Small delay
    }
}

void task_lora(void *pvParameters) {
    ESP_LOGI(TAG_SECONDARY, "Listening for LoRa messages...");

    uint8_t rxData[256];

    while (1) {
        if (is_transmitting) {
            // Continuous LoRa transmission
            uint8_t txData[256];
            int txLen = sprintf((char *)txData, "Continuous LoRa message");

            if (LoRaSend(txData, txLen, SX126x_TXMODE_SYNC)) {
                ESP_LOGI(TAG_SECONDARY, "Sent: %s", txData);
            } else {
                ESP_LOGE(TAG_SECONDARY, "Failed to send LoRa message");
            }

            vTaskDelay(pdMS_TO_TICKS(1000));  // Send every second
        } else {
            // Only receive if not transmitting
            uint8_t rxLen = LoRaReceive(rxData, sizeof(rxData));
            if (rxLen > 0) {
                ESP_LOGI(TAG_SECONDARY, "Received %d byte packet: [%.*s]", rxLen, rxLen, rxData);
                parse_lora_message(rxData, rxLen);
            }

            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// Main sensor handling task (Web server related tasks)
void task_web_server(void *pvParameters) {
    ESP_LOGI(TAG_MAIN, "Web server related tasks will be here...");

    while (true) {
        ESP_LOGI(TAG_MAIN, "MCM_Voltage_Info:%d, Ground_Speed:%d", MCM_Voltage_Info, Ground_Speed);
        // Add web server code and other non-LoRa related operations here
        vTaskDelay(MAIN_TASK_PERIOD / portTICK_PERIOD_MS);  // Periodic task delay
    }
}

void app_main(void) {
    // Initialize NVS (Non-Volatile Storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Start WiFi (if needed)
    wifi_app_start();

    // Initialize UART (USB-C)
    init_uart();

    // Initialize LoRa
    ESP_LOGI(TAG_SECONDARY, "Initializing LoRa...");
    LoRaInit();
    uint32_t frequencyInHz = 915000000;  // 915MHz for LoRa
    int8_t txPowerInDbm = 22;            // Transmission power in dBm
    float tcxoVoltage = 3.3;             // Enable TCXO with 3.3V
    bool useRegulatorLDO = true;         // Use DCDC + LDO for power regulation

    if (LoRaBegin(frequencyInHz, txPowerInDbm, tcxoVoltage, useRegulatorLDO) != 0) {
        ESP_LOGE(TAG_SECONDARY, "LoRa module not recognized!");
        while (1) {
            vTaskDelay(1);
        }
    }

    LoRaConfig(7, 4, 1, 8, 0, true, false);  // LoRa config settings

    // Start tasks on different cores
    xTaskCreatePinnedToCore(&task_lora, "LORA_TASK", 1024 * 4, NULL, 5, NULL, 1);  // Core 1 for LoRa task
    xTaskCreatePinnedToCore(&task_web_server, "WEB_SERVER_TASK", 1024 * 4, NULL, 5, NULL, 0); // Core 0 for Web server task
    xTaskCreatePinnedToCore(&task_uart, "UART_TASK", 1024 * 4, NULL, 5, NULL, 0); // Core 0 for UART task
}

// Public Function Definitions
uint8_t get_temperature(void) {
    return MCM_Voltage_Info;
}

uint8_t get_humidity(void) {
    return Ground_Speed;
}