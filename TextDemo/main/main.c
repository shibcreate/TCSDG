#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ssd1306.h"
#include "font8x8_basic.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_sleep.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "nvs_flash.h"


#define BUTTON_PIN 11
#define tag "SSD1306"

#define BUF_SIZE 128

typedef enum {
    SENDING_STATE,
    RECEIVING_STATE,
    SLEEP_STATE
} FINITE_STATES;

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

static TaskHandle_t stateManager = NULL;
SSD1306_t dev;
void handleSendState(void);
void handleReceiveState(void);
void handleLightSleepState(void);
void setup_i2c_and_ssd1306();

void stateManagerTask(void* parameter){
    currentState = SENDING_STATE;
    TickType_t lastActivityTime = xTaskGetTickCount();
    for(;;){
        switch (currentState)
        {
        case SENDING_STATE:
            handleSendState();
            break;
        case RECEIVING_STATE:
            handleReceiveState();
            break;
        case SLEEP_STATE:
            handleLightSleepState();
            break;
        default:
            printf("Default\n");
            break;
        }
        if (xTaskGetTickCount() - lastActivityTime > pdMS_TO_TICKS(10000)){
            currentState = SLEEP_STATE;
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
}

void handleSendState(){
    printf("M1: Sending\n");
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 2, "Mode: Sending", 13, false);
}
void handleReceiveState(){
    printf("M1: Receiving\n");
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 2, "Mode: Receiving", 15, false);
}

void handleLightSleepState(){
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 2, "Mode: Sleep", 11, false);

    esp_err_t ret;
    ret = esp_sleep_enable_ext0_wakeup(BUTTON_PIN, 0);

    printf("\nentering deepsleep\n");
    fflush(stdout);
    esp_deep_sleep_start();

}

void setup_i2c_and_ssd1306() {
    #if CONFIG_I2C_INTERFACE
        ESP_LOGI(tag, "INTERFACE is i2c");
        ESP_LOGI(tag, "CONFIG_SDA_GPIO=%d", CONFIG_SDA_GPIO);
        ESP_LOGI(tag, "CONFIG_SCL_GPIO=%d", CONFIG_SCL_GPIO);
        ESP_LOGI(tag, "CONFIG_RESET_GPIO=%d", CONFIG_RESET_GPIO);
        i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    #endif // CONFIG_I2C_INTERFACE
    
    #if CONFIG_FLIP
        dev._flip = true;
        ESP_LOGW(tag, "Flip upside down");
    #endif
    
    #if CONFIG_SSD1306_128x64
        ESP_LOGI(tag, "Panel is 128x64");
        ssd1306_init(&dev, 128, 64);
    #endif // CONFIG_SSD1306_128x64
    }


int64_t mode_start_time = 0;
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    setup_i2c_and_ssd1306();

	int center, top, bottom;

	top = 2;
	center = 3; 
	bottom = 8;

	//BUTTON STUFF
	gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON_PIN);
    gpio_pulldown_dis(BUTTON_PIN);
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr, NULL);
	//BUTTON STUFF

	ssd1306_clear_screen(&dev, false);
	xTaskCreate(stateManagerTask, "stateManager", 4096, NULL, 10, &stateManager);
    vTaskSuspend(NULL);
}