#include "lora_handler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>
#include "ra01s.h"
#include "main.h"  // For currentState and saved_mode
#include "telemetry.h"
#include "can_lora.h"

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
    send_lora_message("MCM_Motor_Speed", get_telemetry_value_by_name("MCM_Motor_Speed"));
    send_lora_message("MCM_DCBus_Current", get_telemetry_value_by_name("MCM_DCBus_Current"));
    send_lora_message("MCM_DCBus_Voltage", get_telemetry_value_by_name("MCM_DCBus_Voltage"));
    send_lora_message("MCM_Torque_Feedback", get_telemetry_value_by_name("MCM_Torque_Feedback"));
    send_lora_message("MCM_Commanded_Torque", get_telemetry_value_by_name("MCM_Commanded_Torque"));
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Batch 2: Battery Critical Parameters (5 values)
    send_lora_message("Pack_Voltage", get_telemetry_value_by_name("Pack_Voltage"));
    send_lora_message("BMS_State_Of_Charge", get_telemetry_value_by_name("BMS_State_Of_Charge"));
    send_lora_message("BMS_Highest_Cell_Temperature", get_telemetry_value_by_name("BMS_Highest_Cell_Temperature"));
    send_lora_message("Battery_Low_Voltage", get_telemetry_value_by_name("Battery_Low_Voltage"));
    send_lora_message("VCU_BMS_HighestCellVoltage", get_telemetry_value_by_name("VCU_BMS_HighestCellVoltage"));
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Batch 3: Vehicle Dynamics (5 values)
    send_lora_message("Speed_KPH", get_telemetry_value_by_name("Speed_KPH"));
    send_lora_message("Steering_Angle", get_telemetry_value_by_name("Steering_Angle"));
    send_lora_message("VCU_WSS_FL_S", get_telemetry_value_by_name("VCU_WSS_FL_S"));
    send_lora_message("VCU_WSS_FR_S", get_telemetry_value_by_name("VCU_WSS_FR_S"));
    send_lora_message("VCU_WSS_RL_S", get_telemetry_value_by_name("VCU_WSS_RL_S"));
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Batch 4: Environmental and Power (5 values)
    send_lora_message("Temperature", get_telemetry_value_by_name("Temperature"));
    send_lora_message("Humidity", get_telemetry_value_by_name("Humidity"));
    send_lora_message("EMeter_Current", get_telemetry_value_by_name("EMeter_Current"));
    send_lora_message("EMeter_Voltage", get_telemetry_value_by_name("EMeter_Voltage"));
    send_lora_message("TPS1_Throttle_Percent", get_telemetry_value_by_name("TPS1_Throttle_Percent"));
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Batch 5: Additional Motor Parameters (5 values)
    send_lora_message("MCM_Torque_Command", get_telemetry_value_by_name("MCM_Torque_Command"));
    send_lora_message("MCM_Speed_Command", get_telemetry_value_by_name("MCM_Speed_Command"));
    send_lora_message("MCM_Speed_Mode_Enable", get_telemetry_value_by_name("MCM_Speed_Mode_Enable"));
    send_lora_message("MCM_IntInvert_EnableState", get_telemetry_value_by_name("MCM_IntInvert_EnableState"));
    send_lora_message("MCM_IntInverter_State", get_telemetry_value_by_name("MCM_IntInverter_State"));
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Batch 6: VCU Faults and Safety (5 values)
    send_lora_message("VCU_Fault_TPS_OutOfRange", get_telemetry_value_by_name("VCU_Fault_TPS_OutOfRange"));
    send_lora_message("VCU_Fault_BPS_OutOfRange", get_telemetry_value_by_name("VCU_Fault_BPS_OutOfRange"));
    send_lora_message("VCU_FAULT_TPS_OutOfSync", get_telemetry_value_by_name("VCU_FAULT_TPS_OutOfSync"));
    send_lora_message("VCU_FAULT_TPSBPS_Implausible", get_telemetry_value_by_name("VCU_FAULT_TPSBPS_Implausible"));
    send_lora_message("VCU_FAULT_BSPD_SoftFault", get_telemetry_value_by_name("VCU_FAULT_BSPD_SoftFault"));
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Batch 7: BMS Faults and Status (5 values)
    send_lora_message("BMS_Imminent_Contactor_Opening_Warning", get_telemetry_value_by_name("BMS_Imminent_Contactor_Opening_Warning"));
    send_lora_message("BMS_Cell_Under_Voltage_Fault", get_telemetry_value_by_name("BMS_Cell_Under_Voltage_Fault"));
    send_lora_message("BMS_Cell_Over_Temperature_Fault", get_telemetry_value_by_name("BMS_Cell_Over_Temperature_Fault"));
    send_lora_message("BMS_Pack_Under_Voltage_Fault", get_telemetry_value_by_name("BMS_Pack_Under_Voltage_Fault"));
    send_lora_message("BMS_Isolation_Leakage_Fault", get_telemetry_value_by_name("BMS_Isolation_Leakage_Fault"));
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Batch 8: Additional BMS and VCU (5 values)
    send_lora_message("BMS_Precharge_Fault", get_telemetry_value_by_name("BMS_Precharge_Fault"));
    send_lora_message("BMS_Failed_Thermistor_Fault", get_telemetry_value_by_name("BMS_Failed_Thermistor_Fault"));
    send_lora_message("BMS_Current_State", get_telemetry_value_by_name("BMS_Current_State"));
    send_lora_message("BMS_Main_Contactor_Positive_Closed", get_telemetry_value_by_name("BMS_Main_Contactor_Positive_Closed"));
    send_lora_message("BMS_Main_Contactor_Negative_Closed", get_telemetry_value_by_name("BMS_Main_Contactor_Negative_Closed"));
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Batch 9: VCU Features and Wheel Speed (5 values)
    send_lora_message("VCU_WSS_RR_S", get_telemetry_value_by_name("VCU_WSS_RR_S"));
    send_lora_message("BPS0_Brake_Percent", get_telemetry_value_by_name("BPS0_Brake_Percent"));
    send_lora_message("VCU_MCM_RegenMode", get_telemetry_value_by_name("VCU_MCM_RegenMode"));
    send_lora_message("VCU_MCM_Regen_MaxTorqueNm", get_telemetry_value_by_name("VCU_MCM_Regen_MaxTorqueNm"));
    send_lora_message("VCU_FAULT_LVS_BatteryLow", get_telemetry_value_by_name("VCU_FAULT_LVS_BatteryLow"));
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Batch 10: VCU Debug and Launch Control (5 values)
    send_lora_message("VCU_NOTICE_HVIL_TermSenseLost", get_telemetry_value_by_name("VCU_NOTICE_HVIL_TermSenseLost"));
    send_lora_message("VCU_BMS_HighestCellTemp", get_telemetry_value_by_name("VCU_BMS_HighestCellTemp"));
    send_lora_message("VCU_BMS_LowestCellVoltage", get_telemetry_value_by_name("VCU_BMS_LowestCellVoltage"));
    send_lora_message("VCU_BMS_HighestCellTemp_dC", get_telemetry_value_by_name("VCU_BMS_HighestCellTemp_dC"));
    send_lora_message("VCU_LaunchControl_getTorqueCommand_Nm", get_telemetry_value_by_name("VCU_LaunchControl_getTorqueCommand_Nm"));
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Batch 11: Power Limit and Launch Control PID (5 values)
    send_lora_message("VCU_LaunchControl_getSlipRatioScaled", get_telemetry_value_by_name("VCU_LaunchControl_getSlipRatioScaled"));
    send_lora_message("VCU_LaunchControl_getPidOutput", get_telemetry_value_by_name("VCU_LaunchControl_getPidOutput"));
    send_lora_message("VCU_LaunchControl_PID_Proportional", get_telemetry_value_by_name("VCU_LaunchControl_PID_Proportional"));
    send_lora_message("VCU_LaunchControl_PID_Integral", get_telemetry_value_by_name("VCU_LaunchControl_PID_Integral"));
    send_lora_message("VCU_LaunchControl_PID_TotalError", get_telemetry_value_by_name("VCU_LaunchControl_PID_TotalError"));
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Batch 12: Power Limit Status (5 values)
    send_lora_message("VCU_POWERLIMIT_PID_getTotalError", get_telemetry_value_by_name("VCU_POWERLIMIT_PID_getTotalError"));
    send_lora_message("VCU_POWERLIMIT_PID_getProportional", get_telemetry_value_by_name("VCU_POWERLIMIT_PID_getProportional"));
    send_lora_message("VCU_POWERLIMIT_PID_getIntegral", get_telemetry_value_by_name("VCU_POWERLIMIT_PID_getIntegral"));
    send_lora_message("VCU_POWERLIMIT_getTorqueCommand_Nm", get_telemetry_value_by_name("VCU_POWERLIMIT_getTorqueCommand_Nm"));
    
    ESP_LOGI(TAG, "All telemetry batches sent");
    vTaskDelay(pdMS_TO_TICKS(50));
}


// --- Faults ---
void lora_send_fault_warnings(void) {
    // VCU Faults - only the ones that exist in your telemetry data
    const char* vcu_faults[] = {
        "VCU_Fault_TPS_OutOfRange",
        "VCU_Fault_BPS_OutOfRange",
        "VCU_FAULT_TPS_OutOfSync",
        "VCU_FAULT_TPSBPS_Implausible",
        "VCU_FAULT_BSPD_SoftFault",
        "VCU_FAULT_LVS_BatteryLow",
        "VCU_NOTICE_HVIL_TermSenseLost"
    };
    
    // BMS Faults - only the ones that exist in your telemetry data
    const char* bms_faults[] = {
        "BMS_Imminent_Contactor_Opening_Warning",
        "BMS_Cell_Under_Voltage_Fault",
        "BMS_Cell_Over_Temperature_Fault",
        "BMS_Pack_Under_Voltage_Fault",
        "BMS_Isolation_Leakage_Fault",
        "BMS_Precharge_Fault",
        "BMS_Failed_Thermistor_Fault"
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
        //vTaskDelay(pdMS_TO_TICKS(1000));  // 1s delay
    }
}

static void task_lora_receive(void *pvParameters) {
    ESP_LOGI("LoRa", "LoRa Receive Task Started");

    char key[32];
    int value;

    while (1) {
        uint8_t rxData[256];
        uint8_t rxLen = LoRaReceive(rxData, sizeof(rxData));

        if (rxLen > 0) {
            rxData[rxLen] = '\0';
            ESP_LOGI("LoRa", "Received: %s", rxData);

            // Accept "PL: Mode 1", "Regen: Mode 2", etc.
            if (sscanf((char*)rxData, "%31[^:]: Mode %d", key, &value) == 2) {
                strncpy(lora_cmd.key, key, sizeof(lora_cmd.key)-1);
                lora_cmd.value = value;
                lora_cmd.new_command = true;
                ESP_LOGI("LoRa", "Parsed key=%s, value=%d", key, value);
            } else {
                ESP_LOGW("LoRa", "Failed to parse: %s", rxData);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Yield to avoid watchdog
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