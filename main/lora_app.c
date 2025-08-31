#include "lora_app.h"
#include "ra01s.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

// --- LoRa state variables ---
int drsMode = 0;
int plMode = -1;
int torqueLimit = -1;

static uint8_t Pack_Voltage = 0;
static uint8_t MCM_Motor_Speed = 0;
static uint8_t VCU_Faults = 0;
static uint8_t BMS_Faults = 0;

// Flags + timers
bool is_drs = false, is_pl = false, is_torque = false;
TickType_t drs_end = 0, pl_end = 0, torque_end = 0;

static const char *TAG = "LORA";

// --- Send functions ---
static void send_drs(void) {
    uint8_t tx[64];
    const char *msg = (drsMode == 1) ? "DRS: Manual" : "DRS: Auto";
    int len = sprintf((char*)tx, "%s", msg);
    LoRaSend(tx, len, SX126x_TXMODE_SYNC);
    ESP_LOGI(TAG, "Sent: %s", tx);
}

static void send_pl(void) {
    uint8_t tx[64];
    const char *msg = (plMode == 1) ? "PL: Mode 1" : "PL: Mode 2";
    int len = sprintf((char*)tx, "%s", msg);
    LoRaSend(tx, len, SX126x_TXMODE_SYNC);
    ESP_LOGI(TAG, "Sent: %s", tx);
}

static void send_torque(void) {
    uint8_t tx[64];
    int len = sprintf((char*)tx, "TorqueLimit: %d", torqueLimit);
    LoRaSend(tx, len, SX126x_TXMODE_SYNC);
    ESP_LOGI(TAG, "Sent: %s", tx);
}

// --- Mode table ---
static lora_mode_t modes[] = {
    { "DRS",       &is_drs,    &drs_end,    send_drs },
    { "PL",        &is_pl,     &pl_end,     send_pl },
    { "TorqueLim", &is_torque, &torque_end, send_torque }
};
#define NUM_MODES (sizeof(modes)/sizeof(modes[0]))

lora_mode_t *get_lora_modes(int *count) {
    *count = NUM_MODES;
    return modes;
}

// --- Init ---
int init_lora(void) {
    LoRaInit();
    if (LoRaBegin(915000000, 22, 3.3, true) != 0) return -1;
    LoRaConfig(9, 4, 1, 8, 0, true, false);
    return 0;
}

// --- Parsing incoming messages ---
void parse_lora_message(uint8_t *rx, uint8_t len) {
    char msg[256];
    strncpy(msg, (char*)rx, len);
    msg[len] = '\0';

    int v=0;
    if (sscanf(msg, "Pack_Voltage: %d", &v) == 1) Pack_Voltage = v;
    else if (sscanf(msg, "MCM_Motor_Speed: %d", &v) == 1) MCM_Motor_Speed = v;
    else if (sscanf(msg, "VCU_FAULTS: %d", &v) == 1) VCU_Faults = v;
    else if (sscanf(msg, "BMS_FAULTS: %d", &v) == 1) BMS_Faults = v;
}

// --- Task ---
void task_lora(void *pv) {
    uint8_t rx[256];
    int count;
    lora_mode_t *m = get_lora_modes(&count);

    while (1) {
        bool did_send = false;

        for (int i=0;i<count;i++) {
            if (*(m[i].is_active)) {
                if (xTaskGetTickCount() >= *(m[i].end_time)) {
                    *(m[i].is_active) = false;
                    ESP_LOGI(TAG, "%s stopped", m[i].name);
                } else {
                    m[i].send_func();
                    did_send = true;
                }
            }
        }

        if (!did_send) {
            int len = LoRaReceive(rx, sizeof(rx));
            if (len > 0) {
                ESP_LOGI(TAG, "Received: %.*s", len, rx);
                parse_lora_message(rx, len);
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

// --- Getters for web ---
uint8_t get_pack_voltage(void){return Pack_Voltage;}
uint8_t get_mcm_motor_speed(void){return MCM_Motor_Speed;}
uint8_t get_vcu_faults(void){return VCU_Faults;}
uint8_t get_bms_faults(void){return BMS_Faults;}