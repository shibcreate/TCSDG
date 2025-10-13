#include "lora_handler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdlib.h>
#include "ra01s.h"
#include "main.h"  // For currentState and saved_mode

static const char *TAG = "LORA_HANDLER";

// Task handles
static TaskHandle_t task_master_handle = NULL;
static TaskHandle_t task_lora_receive_handle = NULL;

// --- LoRa message helper ---
static void send_lora_message(const char *label, int value) {
    uint8_t txData[256];
    int txLen = snprintf((char *)txData, sizeof(txData), "%s: %d", label, value);
    ESP_LOGI(TAG, "Sending: %s", txData);
    LoRaSend(txData, txLen, SX126x_TXMODE_SYNC);
}

// --- Telemetry ---
void lora_send_telemetry_data(void) {
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

// --- Faults ---
void lora_send_fault_warnings(void) {
    send_lora_message("VCU_FAULTS", rand() & 0xFF);
    send_lora_message("BMS_FAULTS", rand() & 0xFF);
}

// --- Tasks ---
static void task_master(void *pvParameters) {
    ESP_LOGI(TAG, "LoRa Master Task Started");

    while (1) {
        lora_send_telemetry_data();
        lora_send_fault_warnings();
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1s delay
    }
}

static void task_lora_receive(void *pvParameters) {
    ESP_LOGI(TAG, "LoRa Receive Task Started");

    while (1) {
        uint8_t rxData[256];
        uint8_t rxLen = LoRaReceive(rxData, sizeof(rxData));
        if (rxLen > 0) {
            ESP_LOGI(TAG, "Received %d bytes: [%.*s]", rxLen, rxLen, rxData);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// --- State monitor ---
static void task_state_monitor(void *pvParameters) {
    FINITE_STATES lastState = currentState;

    while (1) {
        if (currentState != lastState) {
            ESP_LOGI(TAG, "State changed: %d -> %d", lastState, currentState);

            switch (currentState) {
                case SENDING_STATE:
                    if (task_master_handle) vTaskResume(task_master_handle);
                    if (task_lora_receive_handle) {
                        vTaskDelete(task_lora_receive_handle);
                        task_lora_receive_handle = NULL;
                    }
                    break;

                case RECEIVING_STATE:
                    if (task_master_handle) vTaskSuspend(task_master_handle);
                    if (!task_lora_receive_handle) {
                        xTaskCreatePinnedToCore(&task_lora_receive, "LORA_RECEIVE", 4096, NULL, 5, &task_lora_receive_handle, 0);
                    }
                    break;

                case SLEEP_STATE:
                    if (task_master_handle) vTaskSuspend(task_master_handle);
                    if (task_lora_receive_handle) {
                        vTaskDelete(task_lora_receive_handle);
                        task_lora_receive_handle = NULL;
                    }
                    break;
            }

            lastState = currentState;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// --- Public functions ---
void lora_handler_init(void) {
    LoRaInit();
    int8_t txPowerInDbm = 22;
    uint32_t frequencyInHz = 915000000;
    float tcxoVoltage = 3.3;
    bool useRegulatorLDO = true;

    ESP_LOGI(TAG, "Initializing LoRa at 915MHz");
    if (LoRaBegin(frequencyInHz, txPowerInDbm, tcxoVoltage, useRegulatorLDO) != 0) {
        ESP_LOGE(TAG, "LoRa module not recognized");
        while (1) { vTaskDelay(1); }
    }

    LoRaConfig(9, 4, 1, 8, 0, true, false);
}

void lora_handler_start(void) {
    xTaskCreatePinnedToCore(&task_master, "LORA_MASTER", 4096, NULL, 10, &task_master_handle, 0);
    xTaskCreatePinnedToCore(&task_state_monitor, "STATE_MONITOR", 2048, NULL, 10, NULL, 0);
}

void lora_handler_stop(void) {
    if (task_master_handle) {
        vTaskDelete(task_master_handle);
        task_master_handle = NULL;
    }
    if (task_lora_receive_handle) {
        vTaskDelete(task_lora_receive_handle);
        task_lora_receive_handle = NULL;
    }
}