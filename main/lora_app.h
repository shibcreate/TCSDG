#pragma once
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Mode struct
typedef struct {
    const char *name;
    bool *is_active;
    TickType_t *end_time;
    void (*send_func)(void);
} lora_mode_t;

// State variables
extern int PLTargetPower;    // 1, 2, 3
extern int RegenMode;        // 1, 2
extern int EfficiencyMode;   // 1, 2, 3

extern bool is_pl;
extern bool is_regen;
extern bool is_eff;

extern TickType_t pl_end;
extern TickType_t regen_end;
extern TickType_t eff_end;

// API
int init_lora(void);
void task_lora(void *pv);
void parse_lora_message(uint8_t *rx, uint8_t len);
lora_mode_t *get_lora_modes(int *count);

// Getters
uint8_t get_pack_voltage(void);
uint8_t get_mcm_motor_speed(void);
uint8_t get_vcu_faults(void);
uint8_t get_bms_faults(void);