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
static uint8_t Pack_Voltage = 0u;
static uint8_t MCM_Motor_Speed = 0u;
static uint8_t VCU_Faults = 0u;
static uint8_t BMS_Faults = 0u;

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

    // Check if the message contains Pack_Voltage
    if (strstr(message, "Pack_Voltage") != NULL) {
        int value = 0;
        if (sscanf(message, "Pack_Voltage: %d", &value) == 1) {
            Pack_Voltage = value;  // Update Pack_Voltage
            //ESP_LOGI(TAG_SECONDARY, "Updated Pack_Voltage to %d", Pack_Voltage);
        }
    }
    // Check if the message contains MCM_Motor_Speed
    else if (strstr(message, "MCM_Motor_Speed") != NULL) {
        int value = 0;
        if (sscanf(message, "MCM_Motor_Speed: %d", &value) == 1) {
            MCM_Motor_Speed = value;  // Update MCM_Motor_Speed
            //ESP_LOGI(TAG_SECONDARY, "Updated MCM_Motor_Speed to %d", MCM_Motor_Speed);
        }
    }
    // Check if the message contains VCU_Faults
    else if (strstr(message, "VCU_FAULTS") != NULL) {
        int value = 0;
        if (sscanf(message, "VCU_FAULTS: %d", &value) == 1) {
            VCU_Faults = value;  // Update VCU_Faults
            //ESP_LOGI(TAG_SECONDARY, "Updated VCU_Faults to %d", VCU_Faults);
        }
    }
    // Check if the message contains BMS_Faults
    else if (strstr(message, "BMS_FAULTS") != NULL) {
        int value = 0;
        if (sscanf(message, "BMS_FAULTS: %d", &value) == 1) {
            BMS_Faults = value;  // Update BMS_Faults
            //ESP_LOGI(TAG_SECONDARY, "Updated BMS_Faults to %d", BMS_Faults);
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
// Add this global flag and the transmission timeout duration
// UART task to read input and send LoRa messages
static bool is_transmitting = false;
static TickType_t transmit_end_time = 0; // Stores when the transmission should stop
static int drsMode = 0;  // Variable to store the current DRS mode

static bool is_pl_transmitting = false;
static TickType_t pl_transmit_end_time = 0;
static int plMode = -1;  // -1 means no PL mode active

static bool is_torque_transmitting = false;
static TickType_t torque_transmit_end_time = 0;
static int torqueLimit = -1;  // -1 means no active TorqueLimit transmission

void task_uart(void *pvParameters) {
    uint8_t data[128];

    while (1) {
        int length = uart_read_bytes(UART_PORT_NUM, data, sizeof(data), 20 / portTICK_PERIOD_MS);

        if (length > 0) {
            data[length] = '\0';
            //ESP_LOGI(TAG_MAIN, "Received data: %s", data);

            if (data[0] == '1') {
                drsMode = 1;
                is_transmitting = true;
                transmit_end_time = xTaskGetTickCount() + pdMS_TO_TICKS(15000);
                //ESP_LOGI(TAG_MAIN, "Switched to DRS: Manual mode for 15 seconds.");
            } else if (data[0] == '0') {
                drsMode = 0;
                is_transmitting = true;
                transmit_end_time = xTaskGetTickCount() + pdMS_TO_TICKS(15000);
                //ESP_LOGI(TAG_MAIN, "Switched to DRS: Auto mode for 15 seconds.");
            } else if (data[0] == '2') {
                plMode = 1;
                is_pl_transmitting = true;
                pl_transmit_end_time = xTaskGetTickCount() + pdMS_TO_TICKS(15000);
                //ESP_LOGI(TAG_MAIN, "Switched to PL: Mode 1 for 15 seconds.");
            } else if (data[0] == '3') {
                plMode = 2;
                is_pl_transmitting = true;
                pl_transmit_end_time = xTaskGetTickCount() + pdMS_TO_TICKS(15000);
                //ESP_LOGI(TAG_MAIN, "Switched to PL: Mode 2 for 15 seconds.");
            } else if (data[0] == '4') {
                torqueLimit = 200;
                is_torque_transmitting = true;
                torque_transmit_end_time = xTaskGetTickCount() + pdMS_TO_TICKS(15000);
                //ESP_LOGI(TAG_MAIN, "Switched to TorqueLimit: 200 for 15 seconds.");
            } else if (data[0] == '5') {
                torqueLimit = 150;
                is_torque_transmitting = true;
                torque_transmit_end_time = xTaskGetTickCount() + pdMS_TO_TICKS(15000);
                //ESP_LOGI(TAG_MAIN, "Switched to TorqueLimit: 150 for 15 seconds.");
            } else {
                is_transmitting = false;
                is_pl_transmitting = false;
                is_torque_transmitting = false;
                //ESP_LOGI(TAG_MAIN, "Invalid input, stopped transmission.");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void task_lora(void *pvParameters) {
    //ESP_LOGI(TAG_SECONDARY, "Listening for LoRa messages...");

    uint8_t rxData[256];

    while (1) {
        if (is_transmitting) {
            if (xTaskGetTickCount() >= transmit_end_time) {
                is_transmitting = false;
                //ESP_LOGI(TAG_MAIN, "DRS transmission stopped after 15 seconds.");
            } else {
                uint8_t txData[256];
                const char *drsMessage = (drsMode == 1) ? "DRS: Manual" : "DRS: Auto";
                int txLen = sprintf((char *)txData, "%s", drsMessage);

                if (LoRaSend(txData, txLen, SX126x_TXMODE_SYNC)) {
                    ESP_LOGI(TAG_SECONDARY, "Sent: %s", txData);
                } else {
                    ESP_LOGE(TAG_SECONDARY, "Failed to send LoRa message");
                }

                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        } 
        else if (is_pl_transmitting) {
            if (xTaskGetTickCount() >= pl_transmit_end_time) {
                is_pl_transmitting = false;
                //ESP_LOGI(TAG_MAIN, "PL transmission stopped after 15 seconds.");
            } else {
                uint8_t txData[256];
                const char *plMessage = (plMode == 1) ? "PL: Mode 1" : "PL: Mode 2";
                int txLen = sprintf((char *)txData, "%s", plMessage);

                if (LoRaSend(txData, txLen, SX126x_TXMODE_SYNC)) {
                    ESP_LOGI(TAG_SECONDARY, "Sent: %s", txData);
                } else {
                    ESP_LOGE(TAG_SECONDARY, "Failed to send PL message");
                }

                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        } 
        else if (is_torque_transmitting) {
            if (xTaskGetTickCount() >= torque_transmit_end_time) {
                is_torque_transmitting = false;
                //ESP_LOGI(TAG_MAIN, "TorqueLimit transmission stopped after 15 seconds.");
            } else {
                uint8_t txData[256];
                char torqueMessage[50];
                sprintf(torqueMessage, "TorqueLimit: %d", torqueLimit);
                int txLen = sprintf((char *)txData, "%s", torqueMessage);

                if (LoRaSend(txData, txLen, SX126x_TXMODE_SYNC)) {
                    ESP_LOGI(TAG_SECONDARY, "Sent: %s", txData);
                } else {
                    ESP_LOGE(TAG_SECONDARY, "Failed to send TorqueLimit message");
                }

                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        } 
        else {
            uint8_t rxLen = LoRaReceive(rxData, sizeof(rxData));
            if (rxLen > 0) {
                //ESP_LOGI(TAG_SECONDARY, "Received %d byte packet: [%.*s]", rxLen, rxLen, rxData);
                printf("%.*s\n", rxLen, rxData);  // This prints the raw message
                parse_lora_message(rxData, rxLen);
            }

            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// Main sensor handling task (Web server related tasks)
void task_web_server(void *pvParameters) {
    //ESP_LOGI(TAG_MAIN, "Web server related tasks will be here...");

    while (true) {
        //ESP_LOGI(TAG_MAIN, "Pack_Voltage:%d, MCM_Motor_Speed:%d, VCU_Faults:%d, BMS_Faults:%d", Pack_Voltage, MCM_Motor_Speed, VCU_Faults, BMS_Faults);

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
    //ESP_LOGI(TAG_SECONDARY, "Initializing LoRa...");
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

    LoRaConfig(9, 4, 1, 8, 0, true, false);  // LoRa config settings

    // Start tasks on different cores
    xTaskCreatePinnedToCore(&task_lora, "LORA_TASK", 1024 * 4, NULL, 5, NULL, 1);  // Core 1 for LoRa task
    xTaskCreatePinnedToCore(&task_web_server, "WEB_SERVER_TASK", 1024 * 4, NULL, 5, NULL, 0); // Core 0 for Web server task
    xTaskCreatePinnedToCore(&task_uart, "UART_TASK", 1024 * 4, NULL, 5, NULL, 0); // Core 0 for UART task
}

// Getter Functions for Web Server
uint8_t get_pack_voltage(void) {
    return Pack_Voltage;
}

uint8_t get_mcm_motor_speed(void) {
    return MCM_Motor_Speed;
}

uint8_t get_vcu_faults(void) {
    return VCU_Faults;
}

uint8_t get_bms_faults(void) {
    return BMS_Faults;
}