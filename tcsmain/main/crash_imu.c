#include "crash_imu.h"
#include "esp_timer.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "nvs_flash.h"
#include <icm42670.h>
#include "freertos/task.h"
#include "esp_log.h"

const char *TAG_IMU = "IMU";

TaskHandle_t imuTaskHandle = NULL;

// RTC start time
time_t start_time = 1700286000; // Update as needed

// Globals
uint8_t imu_crash_event = 2;      // 1: Write, 2: Read, 3: Erase
bool vcu_can_captured = false;
bool bms_can_captured = false;

// Returns time relative to start_time
time_t get_relative_time(void) {
    int64_t usec = esp_timer_get_time();
    time_t elapsed_sec = usec / 1000000;
    return start_time + elapsed_sec;
}

void print_crash_timestamp() {
    time_t curr_time = get_relative_time();
    struct tm timeinfo;
    localtime_r(&curr_time, &timeinfo);

    ESP_LOGI(TAG_IMU, "Crash detected at relative time: %04d-%02d-%02d %02d:%02d:%02d",
        timeinfo.tm_year + 1900,
        timeinfo.tm_mon + 1,
        timeinfo.tm_mday,
        timeinfo.tm_hour,
        timeinfo.tm_min,
        timeinfo.tm_sec);
}

void vcu_save_crash_record(crash_record_t *record) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("v_crash_log", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        printf("VCU NVS open failed: %s\n", esp_err_to_name(err));
        return;
    }

    err = nvs_set_u8(nvs, "v_crash_flag", 1);
    if (err != ESP_OK) printf("VCU flag set failed: %s\n", esp_err_to_name(err));

    err = nvs_set_blob(nvs, "v_crash_record", record, sizeof(crash_record_t));
    if (err != ESP_OK) printf("VCU blob set failed: %s\n", esp_err_to_name(err));

    err = nvs_commit(nvs);
    if (err != ESP_OK) printf("VCU commit failed: %s\n", esp_err_to_name(err));
    else printf("VCU crash record saved successfully.\n");

    nvs_close(nvs);
}

void bms_save_crash_record(crash_record_t *record) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("b_crash_log", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        printf("BMS NVS open failed: %s\n", esp_err_to_name(err));
        return;
    }

    err = nvs_set_u8(nvs, "b_crash_flag", 1);
    if (err != ESP_OK) printf("BMS flag set failed: %s\n", esp_err_to_name(err));

    err = nvs_set_blob(nvs, "b_crash_record", record, sizeof(crash_record_t));
    if (err != ESP_OK) printf("BMS blob set failed: %s\n", esp_err_to_name(err));

    err = nvs_commit(nvs);
    if (err != ESP_OK) printf("BMS commit failed: %s\n", esp_err_to_name(err));
    else printf("BMS crash record saved successfully.\n");

    nvs_close(nvs);
}

void vcu_read_record() {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("v_crash_log", NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        printf("Failed to open VCU NVS namespace: %s\n", esp_err_to_name(err));
        return;
    }

    uint8_t flag = 0;
    err = nvs_get_u8(nvs, "v_crash_flag", &flag);
    if (err == ESP_ERR_NVS_NOT_FOUND || flag != 1) {
        printf("No VCU crash record found.\n");
        nvs_close(nvs);
        return;
    } else if (err != ESP_OK) {
        printf("Error reading VCU crash flag: %s\n", esp_err_to_name(err));
        nvs_close(nvs);
        return;
    }

    crash_record_t record;
    size_t size = sizeof(record);
    err = nvs_get_blob(nvs, "v_crash_record", &record, &size);
    if (err == ESP_OK) {
        printf("=== VCU CRASH ===\n");
        printf("G-Force: %.2f g\n", record.g_force);
        printf("Accel: [%.2f, %.2f, %.2f]\n", record.accel[0], record.accel[1], record.accel[2]);
        printf("CAN Msg ID: 0x%lX, DLC: %d\n", record.vcu_can_id, record.vcu_can_dlc);
        printf("Data: ");
        for (int i = 0; i < record.vcu_can_dlc; i++) printf("%02X ", record.vcu_can_data[i]);
        printf("\n");
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("VCU crash record not found in NVS.\n");
    } else if (err == ESP_ERR_NVS_INVALID_LENGTH) {
        printf("VCU crash record size mismatch (invalid length).\n");
    } else {
        printf("Failed to read VCU crash record: %s\n", esp_err_to_name(err));
    }
    nvs_close(nvs);
}

void bms_read_record() {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("b_crash_log", NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        printf("Failed to open BMS NVS namespace: %s\n", esp_err_to_name(err));
        return;
    }

    uint8_t flag = 0;
    err = nvs_get_u8(nvs, "b_crash_flag", &flag);
    if (err == ESP_ERR_NVS_NOT_FOUND || flag != 1) {
        printf("No BMS crash record found.\n");
        nvs_close(nvs);
        return;
    } else if (err != ESP_OK) {
        printf("Error reading BMS crash flag: %s\n", esp_err_to_name(err));
        nvs_close(nvs);
        return;
    }

    crash_record_t record;
    size_t size = sizeof(record);
    err = nvs_get_blob(nvs, "b_crash_record", &record, &size);
    if (err == ESP_OK) {
        printf("=== BMS CRASH ===\n");
        printf("G-Force: %.2f g\n", record.g_force);
        printf("Accel: [%.2f, %.2f, %.2f]\n", record.accel[0], record.accel[1], record.accel[2]);
        printf("CAN Msg ID: 0x%lX, DLC: %d\n", record.bms_can_id, record.bms_can_dlc);
        printf("Data: ");
        for (int i = 0; i < record.bms_can_dlc; i++) printf("%02X ", record.bms_can_data[i]);
        printf("\n");
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("BMS crash record not found in NVS.\n");
    } else if (err == ESP_ERR_NVS_INVALID_LENGTH) {
        printf("BMS crash record size mismatch (invalid length).\n");
    } else {
        printf("Failed to read BMS crash record: %s\n", esp_err_to_name(err));
    }
    nvs_close(nvs);
}

void vcu_erase_record() {
    nvs_handle_t nvs;
    if (nvs_open("v_crash_log", NVS_READWRITE, &nvs) != ESP_OK) return;
    nvs_erase_key(nvs, "v_crash_flag");
    nvs_erase_key(nvs, "v_crash_record");
    nvs_commit(nvs);
    nvs_close(nvs);
    vcu_can_captured = false;
    printf("VCU Crash record erased.\n");
}

void bms_erase_record() {
    nvs_handle_t nvs;
    if (nvs_open("b_crash_log", NVS_READWRITE, &nvs) != ESP_OK) return;

    nvs_erase_key(nvs, "b_crash_flag");
    nvs_erase_key(nvs, "b_crash_record");
    nvs_commit(nvs);
    nvs_close(nvs);
    bms_can_captured = false;
    printf("BMS Crash record erased.\n");
}

void handle_crash_event() {
    if (imu_crash_event == 2) {
		vcu_read_record();
		bms_read_record();
	}
    else if (imu_crash_event == 3){
		vcu_erase_record();
		bms_erase_record();
	} 
}

void icm42670_test(void *pvParameters)
{
    // init device descriptor and device
    icm42670_t dev = { 0 };
    ESP_ERROR_CHECK(
        icm42670_init_desc(&dev, I2C_ADDR, PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(icm42670_init(&dev));

    // enable accelerometer and gyro in low-noise (LN) mode
    ESP_ERROR_CHECK(icm42670_set_gyro_pwr_mode(&dev, ICM42670_GYRO_ENABLE_LN_MODE));
    ESP_ERROR_CHECK(icm42670_set_accel_pwr_mode(&dev, ICM42670_ACCEL_ENABLE_LN_MODE));

    /* OPTIONAL */
    // enable low-pass-filters on accelerometer and gyro
    ESP_ERROR_CHECK(icm42670_set_accel_lpf(&dev, ICM42670_ACCEL_LFP_53HZ));
    ESP_ERROR_CHECK(icm42670_set_gyro_lpf(&dev, ICM42670_GYRO_LFP_53HZ));
    // set output data rate (ODR)
    ESP_ERROR_CHECK(icm42670_set_accel_odr(&dev, ICM42670_ACCEL_ODR_200HZ));
    ESP_ERROR_CHECK(icm42670_set_gyro_odr(&dev, ICM42670_GYRO_ODR_200HZ));
    // set full scale range (FSR)
    ESP_ERROR_CHECK(icm42670_set_accel_fsr(&dev, ICM42670_ACCEL_RANGE_16G));
    ESP_ERROR_CHECK(icm42670_set_gyro_fsr(&dev, ICM42670_GYRO_RANGE_2000DPS));

    // read temperature sensor value once
    float temperature;
    ESP_ERROR_CHECK(icm42670_read_temperature(&dev, &temperature));
    ESP_LOGI(TAG_IMU, "Temperature reading: %f", temperature);

    int16_t raw_reading;
    uint8_t data_register;
    uint8_t CRASH_THRESHOLD = 18;
    /* select which acceleration or gyro value should be read: */
    // data_register = ICM42670_REG_ACCEL_DATA_X1;
    // data_register = ICM42670_REG_ACCEL_DATA_Y1;
    // data_register = ICM42670_REG_ACCEL_DATA_Z1;
    data_register = ICM42670_REG_GYRO_DATA_X1;
    // data_register = ICM42670_REG_GYRO_DATA_Y1;
    // data_register = ICM42670_REG_GYRO_DATA_Z1;

    // now poll selected accelerometer or gyro raw value directly from registers
    while (1)
    {
        ESP_ERROR_CHECK(icm42670_read_raw_data(&dev, data_register, &raw_reading));

        ESP_LOGI(TAG_IMU, "Raw accelerometer / gyro reading: %d", raw_reading);

        if (raw_reading > CRASH_THRESHOLD) {
            if (imu_crash_event != 1) {  // prevent repeated writes
                imu_crash_event = 1; // Mark crash happened
                vTaskDelay(pdMS_TO_TICKS(1000));
                print_crash_timestamp();
                printf("I am in IMU deepsleep...\n");
                handleLightSleepState();
            }
        }
    }
}

void start_imu_task(void){
    xTaskCreatePinnedToCore(
        icm42670_test, 
        "icm42670_test", 
        configMINIMAL_STACK_SIZE * 8, 
        NULL, 
        5, 
        &imuTaskHandle, 
        1
    );
}