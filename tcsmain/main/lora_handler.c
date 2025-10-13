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

// --- Telemetry Batches ---
void lora_send_telemetry_data(void) {
    ESP_LOGI(TAG, "Sending telemetry data in batches...");
    
    // Batch 1: Critical Motor Parameters (5 values)
    send_lora_message("MCM_Motor_Speed", get_telemetry_value_by_name("Motor_Speed"));
    send_lora_message("MCM_DC_Bus_Current", get_telemetry_value_by_name("MCM_DC_Bus_Current"));
    send_lora_message("MCM_DC_Bus_Voltage", get_telemetry_value_by_name("MCM_DC_Bus_Voltage"));
    send_lora_message("MCM_Torque_Feedback", get_telemetry_value_by_name("MCM_Torque_Feedback"));
    send_lora_message("MCM_Commanded_Torque", get_telemetry_value_by_name("MCM_Commanded_Torque"));
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Batch 2: Battery Critical Parameters (5 values)
    send_lora_message("BMS_Pack_Voltage", get_telemetry_value_by_name("Pack_Voltage"));
    send_lora_message("Lowest_Cell_Voltage", get_telemetry_value_by_name("Lowest_Cell_Voltage"));
    send_lora_message("Highest_Cell_Voltage", get_telemetry_value_by_name("Highest_Cell_Voltage"));
    send_lora_message("Lowest_Cell_Temperature", get_telemetry_value_by_name("Lowest_Cell_Temperature"));
    send_lora_message("Highest_Cell_Temperature", get_telemetry_value_by_name("Highest_Cell_Temperature"));
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Batch 3: Vehicle Dynamics (5 values)
    send_lora_message("Speed_KPH", get_telemetry_value_by_name("Speed_KPH"));
    send_lora_message("Steering_Angle", get_telemetry_value_by_name("Steering_Angle"));
    send_lora_message("VCU_WSS_FL_S", get_telemetry_value_by_name("VCU_WSS_FL_S"));
    send_lora_message("VCU_WSS_FR_S", get_telemetry_value_by_name("VCU_WSS_FR_S"));
    send_lora_message("VCU_WSS_RL_S", get_telemetry_value_by_name("VCU_WSS_RL_S"));
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Batch 4: Environmental Sensors (5 values)
    send_lora_message("Temperature", get_telemetry_value_by_name("Temperature"));
    send_lora_message("Humidity", get_telemetry_value_by_name("Humidity"));
    send_lora_message("EMeter_Current", get_telemetry_value_by_name("EMeter_Current"));
    send_lora_message("EMeter_Voltage", get_telemetry_value_by_name("EMeter_Voltage"));
    send_lora_message("Raw_IMU_Reading", get_telemetry_value_by_name("Raw_IMU_Reading"));
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Batch 5: Additional Motor Parameters (5 values)
    send_lora_message("MCM_Torque_Limit_Command", get_telemetry_value_by_name("MCM_Torque_Limit_Command"));
    send_lora_message("MCM_Speed_Mode_Enable", get_telemetry_value_by_name("MCM_Speed_Mode_Enable"));
    send_lora_message("MCM_Int_Invert_Enable_State", get_telemetry_value_by_name("MCM_Int_Invert_Enable_State"));
    send_lora_message("MCM_Int_Inverter_State", get_telemetry_value_by_name("MCM_Int_Inverter_State"));
    send_lora_message("VCU_WSS_RR_S", get_telemetry_value_by_name("VCU_WSS_RR_S"));
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Batch 6: Throttle and Launch Control (5 values)
    send_lora_message("TPS0_Calib_Min", get_telemetry_value_by_name("TPS0_Calib_Min"));
    send_lora_message("TPS0_Calib_Max", get_telemetry_value_by_name("TPS0_Calib_Max"));
    send_lora_message("LC_Ready", get_telemetry_value_by_name("LC_Ready"));
    send_lora_message("LC_Status", get_telemetry_value_by_name("LC_Status"));
    send_lora_message("Torque", get_telemetry_value_by_name("Torque"));
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Batch 7: DRS and Additional (5 values)
    send_lora_message("Slip_Ratio", get_telemetry_value_by_name("Slip_Ratio"));
    send_lora_message("Start_Torque", get_telemetry_value_by_name("Start_Torque"));
    send_lora_message("DRS_Enable", get_telemetry_value_by_name("DRS_Enable"));
    send_lora_message("DRS_Mode", get_telemetry_value_by_name("DRS_Mode"));
    send_lora_message("BMS_Balancing_State", get_telemetry_value_by_name("BMS_Balancing_State"));
    vTaskDelay(pdMS_TO_TICKS(200));
    
    ESP_LOGI(TAG, "All telemetry batches sent");
}


// --- Faults ---
void lora_send_fault_warnings(void) {
    // VCU Faults - sum all VCU fault values
    const char* vcu_faults[] = {
        "VCU_FAULT_TPS_OutOfRange",
        "VCU_FAULT_BPS_OutOfRange",
        "VCU_FAULT_TPS_PowerFailure",
        "VCU_FAULT_BPS_PowerFailure",
        "VCU_FAULT_TPS_SignalFailure", 
        "VCU_FAULT_BPS_SignalFailure",
        "VCU_FAULT_TPS_NotCalibrated",
        "VCU_FAULT_BPS_NotCalibrated",
        "VCU_FAULT_TPS_OutOfSync",
        "VCU_FAULT_TPSBPS_Implausible",
        "VCU_FAULT_BSPD_SoftFault",
        "VCU_FAULT_LVS_BatteryEmpty",
        "VCU_WARNING_LVS_BatteryLow",
        "VCU_NOTICE_HVIL_TermSenseLost"
    };
    
    // BMS Faults - sum all BMS fault values
    const char* bms_faults[] = {
        "BMS_Pack_High_Volt_Warning",
        "BMS_Pack_Low_Volt_Warning",
        "BMS_Cell_Low_Volt_Warning",
        "BMS_Cell_High_Volt_Warning",
        "BMS_Cell_High_Temp_Warning",
        "BMS_Cell_Low_Temp_Warning",
        "BMS_Cell_Volt_Imbalance_Warning",
        "BMS_Pack_High_Volt_Fault",
        "BMS_Pack_Low_Volt_Fault", 
        "BMS_Cell_Low_Volt_Fault",
        "BMS_Cell_High_Volt_Fault",
        "BMS_Cell_High_Temp_Fault",
        "BMS_Cell_Volt_Imbalance_Fault",
        "BMS_Balacing_End_Fault"
    };
    
    // Calculate VCU fault sum
    uint32_t vcu_fault_sum = 0;
    for (int i = 0; i < sizeof(vcu_faults) / sizeof(vcu_faults[0]); i++) {
        float value = get_telemetry_value_by_name(vcu_faults[i]);
        vcu_fault_sum += (uint32_t)value; // Add each fault value (should be 0 or 1)
    }
    
    // Calculate BMS fault sum
    uint32_t bms_fault_sum = 0;
    for (int i = 0; i < sizeof(bms_faults) / sizeof(bms_faults[0]); i++) {
        float value = get_telemetry_value_by_name(bms_faults[i]);
        bms_fault_sum += (uint32_t)value; // Add each fault value (should be 0 or 1)
    }
    
    // Send the summed fault counts
    send_lora_message("VCU_FAULTS", vcu_fault_sum & 0xFF);
    send_lora_message("BMS_FAULTS", bms_fault_sum & 0xFF);
    
    //ESP_LOGI(TAG, "VCU Faults: %d, BMS Faults: %d", vcu_fault_sum, bms_fault_sum);
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