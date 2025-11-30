#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_flash.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include <inttypes.h>
#include "sdkconfig.h"
#include "lora_handler.h"
#include "esp_sleep.h"
#include "esp_attr.h"
#include <time.h>
#include "driver/twai.h"                        
#include <icm42670.h>           
#include "nvs_flash.h"
//new refactored .h
#include <rgb_ledc_controller.h>
#include "humidity.h"
#include "telemetry.h"
#include "can_lora.h"
#include "crash_imu.h"

#define BUTTON_PIN 1
#define BUF_SIZE 128

static const char *TAG_RGB = "rainbow_flash";

/* GPIOs for RGB LED */
#define GPIO_LED_RED   38
#define GPIO_LED_GREEN 39
#define GPIO_LED_BLUE  40

rgb_led_t led1;
const uint32_t RED = 0xFF0000;
const uint32_t BLUE = 0x0000FF;
const uint32_t YELLOW = 0xFFFF00;

// Global var for current mode
FINITE_STATES currentState = 0;
RTC_DATA_ATTR FINITE_STATES saved_mode;

void IRAM_ATTR button_isr(void *arg) {
    if (gpio_get_level(BUTTON_PIN) == 0) {
        currentState = RECEIVING_STATE;  // pressed and hold
    } else {
        currentState = SENDING_STATE;  // not pressed
    }
}

static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

#if CONFIG_CAN_BITRATE_25
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_25KBITS();
#define BITRATE "Bitrate is 25 Kbit/s"
#elif CONFIG_CAN_BITRATE_50
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_50KBITS();
#define BITRATE "Bitrate is 50 Kbit/s"
#elif CONFIG_CAN_BITRATE_100
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_100KBITS();
#define BITRATE "Bitrate is 100 Kbit/s"
#elif CONFIG_CAN_BITRATE_125
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
#define BITRATE "Bitrate is 125 Kbit/s"
#elif CONFIG_CAN_BITRATE_250
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
#define BITRATE "Bitrate is 250 Kbit/s"
#elif CONFIG_CAN_BITRATE_500
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
#define BITRATE "Bitrate is 500 Kbit/s"
#elif CONFIG_CAN_BITRATE_800
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_800KBITS();
#define BITRATE "Bitrate is 800 Kbit/s"
#elif CONFIG_CAN_BITRATE_1000
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
#define BITRATE "Bitrate is 1 Mbit/s"
#endif

static TaskHandle_t stateManager = NULL;
void stateManagerTask(void* parameter);
void handleSendState(void);
void handleReceiveState(void);
void handleLightSleepState(void);

static const twai_general_config_t g_config =
	TWAI_GENERAL_CONFIG_DEFAULT(CONFIG_CTX_GPIO, CONFIG_CRX_GPIO, TWAI_MODE_NORMAL);

void stateManagerTask(void* parameter){
    currentState = SENDING_STATE;
    FINITE_STATES lastState = currentState;
    for(;;){

        if (currentState != lastState) {
            // State transition detected
            if (currentState == SENDING_STATE) {
                if (imuTaskHandle != NULL) {
                    vTaskResume(imuTaskHandle);
                }
            } else if (currentState == RECEIVING_STATE) {
                if (imuTaskHandle != NULL) {
                    vTaskSuspend(imuTaskHandle);
                }
            }
            lastState = currentState;
        }

        switch (currentState)
        {
        case SENDING_STATE: //Master sends to lora and receives from can
            
            rgb_led_set_color(&led1, BLUE);
            
            break;
        case RECEIVING_STATE: //Master receives from lora and sends to can
        
            rgb_led_set_color(&led1, RED);
            
            break;
        case SLEEP_STATE:
            handleLightSleepState();
            break;
        default:
            printf("Default\n");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));

    }
}

void handleLightSleepState(){
    esp_err_t ret;
    ret = esp_sleep_enable_ext0_wakeup(BUTTON_PIN, 0);

    printf("\nentering deepsleep\n");
    fflush(stdout);
    esp_deep_sleep_start();
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(CAN_LORA_TAG, "System starting, init start_time");
    start_time = 1700286000;

	handle_crash_event();   
    init_telemetry_data();

    ESP_LOGI(CAN_LORA_TAG, "%s",BITRATE);
	ESP_LOGI(CAN_LORA_TAG, "CTX_GPIO=%d",CONFIG_CTX_GPIO);
	ESP_LOGI(CAN_LORA_TAG, "CRX_GPIO=%d",CONFIG_CRX_GPIO);

	ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
	ESP_LOGI(CAN_LORA_TAG, "Driver installed");
	ESP_ERROR_CHECK(twai_start());
	ESP_LOGI(CAN_LORA_TAG, "Driver started");

	ESP_ERROR_CHECK(i2cdev_init());

	//BUTTON STUFF
	gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON_PIN);
    gpio_pulldown_dis(BUTTON_PIN);
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr, NULL);
	//BUTTON STUFF

    // Create and initialize RGB LED instance
    led1 = rgb_led_new(GPIO_LED_RED, GPIO_LED_GREEN, GPIO_LED_BLUE,
                                 LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2);
    ESP_ERROR_CHECK(rgb_led_init(&led1));

    // Initialize LoRa
    lora_handler_init();
    lora_handler_start();


	xTaskCreate(stateManagerTask, "stateManager", 4096, NULL, 5, &stateManager);

  	start_can_task();
    start_imu_task();
    humidity_start_task();

    vTaskSuspend(NULL);

}
