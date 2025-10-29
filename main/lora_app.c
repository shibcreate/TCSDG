#include "lora_app.h"
#include "ra01s.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include <stdio.h>
#include <string.h>

// --- LoRa state variables ---
int PLTargetPower = -1;     // 1, 2, 3
int RegenMode = -1;         // 1, 2
int EfficiencyMode = -1;    // 1, 2, 3

static uint8_t Pack_Voltage = 0;
static uint8_t MCM_Motor_Speed = 0;
static uint8_t VCU_Faults = 0;
static uint8_t BMS_Faults = 0;

// Flags + timers
bool is_pl = false, is_regen = false, is_eff = false;
TickType_t pl_end = 0, regen_end = 0, eff_end = 0;

static const char *TAG = "LORA";

// --- Button Handler ---
#define BUTTON_GPIO 34
static volatile bool use_fsk = false;

static void IRAM_ATTR button_isr_handler(void* arg) {
    use_fsk = !use_fsk;
}

// --- Send functions ---
static void send_pl(void) {
    uint8_t tx[64];
    const char *msg = "PL: Unknown";  // default
    switch (PLTargetPower) {
        case 1: msg = "PL: Mode 1"; break;
        case 2: msg = "PL: Mode 2"; break;
        case 3: msg = "PL: Mode 3"; break;
    }
    int len = sprintf((char*)tx, "%s", msg);
    LoRaSend(tx, len, SX126x_TXMODE_SYNC);
    ESP_LOGI(TAG, "Sent: %s", tx);
}

static void send_regen(void) {
    uint8_t tx[64];
    const char *msg = "Regen: Unknown";
    if (RegenMode == 1) msg = "Regen: Mode 1";
    else if (RegenMode == 2) msg = "Regen: Mode 2";
    int len = sprintf((char*)tx, "%s", msg);
    LoRaSend(tx, len, SX126x_TXMODE_SYNC);
    ESP_LOGI(TAG, "Sent: %s", tx);
}

static void send_eff(void) {
    uint8_t tx[64];
    const char *msg = "Efficiency: Unknown";  // default
    switch (EfficiencyMode) {
        case 1: msg = "Efficiency: Mode 1"; break;
        case 2: msg = "Efficiency: Mode 2"; break;
        case 3: msg = "Efficiency: Mode 3"; break;
    }
    int len = sprintf((char*)tx, "%s", msg);
    LoRaSend(tx, len, SX126x_TXMODE_SYNC);
    ESP_LOGI(TAG, "Sent: %s", tx);
}

// --- Mode table ---
static lora_mode_t modes[] = {
    { "PLTargetPower", &is_pl,    &pl_end,    send_pl },
    { "RegenMode",     &is_regen, &regen_end, send_regen },
    { "EfficiencyMode",&is_eff,   &eff_end,   send_eff }
};
#define NUM_MODES (sizeof(modes)/sizeof(modes[0]))

lora_mode_t *get_lora_modes(int *count) {
    *count = NUM_MODES;
    return modes;
}

// --- Button Init ---
static void init_button(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);
}

// --- Init ---
int init_lora(void) {
    init_button();

    if (use_fsk) {
        ESP_LOGI(TAG, "Using FSK mode");
        return 0;
    } else {
        ESP_LOGI(TAG, "Using LoRa mode");
        LoRaInit();
        if (LoRaBegin(915000000, 22, 3.3, true) != 0) return -1;
        LoRaConfig(9, 4, 1, 8, 0, true, false);
        return 0;
    }
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
        if (use_fsk) {
            int fsk_len = 0;
            uint8_t fsk_rx[256];
            if (fsk_len > 0) {
                ESP_LOGI(TAG, "FSK Received: %.*s", fsk_len, fsk_rx);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            bool did_send = false;
            for (int i = 0; i < count; i++) {
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
                vTaskDelay(pdMS_TO_TICKS(250));
            }
        }
    }
}

// --- Getters for web ---
uint8_t get_pack_voltage(void){return Pack_Voltage;}
uint8_t get_mcm_motor_speed(void){return MCM_Motor_Speed;}
uint8_t get_vcu_faults(void){return VCU_Faults;}
uint8_t get_bms_faults(void){return BMS_Faults;}