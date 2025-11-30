#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern TaskHandle_t imuTaskHandle;

// RTC timer function
time_t get_relative_time(void);
extern time_t start_time;

// Tag for logging IMU info
extern const char *TAG_IMU;

// I2C port and address config for IMU
#define PORT 0

#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_GND)
#define I2C_ADDR ICM42670_I2C_ADDR_GND
#endif

#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_VCC)
#define I2C_ADDR ICM42670_I2C_ADDR_VCC
#endif

// Crash data record structure
typedef struct {
    float g_force;
    float accel[3];
    uint32_t vcu_can_id;
    uint8_t vcu_can_dlc;
    uint8_t vcu_can_data[8];
    uint32_t bms_can_id;
    uint8_t bms_can_dlc;
    uint8_t bms_can_data[8];
    bool vcu_captured;
    bool bms_captured;
} crash_record_t;

// Crash handling functions
void print_crash_timestamp();
//CRASH STUFF
void vcu_save_crash_record(crash_record_t *record);
void bms_save_crash_record(crash_record_t *record);
void vcu_read_crash_record();
void bms_read_crash_record();
void vcu_erase_crash_record();
void bms_erase_crash_record();
void handle_crash_event();
void start_imu_task(void);

// IMU test tasks
void icm42670_test(void *pvParameters);

// IMU crash state flags and capture status exposed as global if needed
extern uint8_t imu_crash_event;
extern bool vcu_can_captured;
extern bool bms_can_captured;
