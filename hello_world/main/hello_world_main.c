/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"

#define BUTTON_PIN 4
void app_main(void)
{
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON_PIN);
    while(1){
        if(gpio_get_level(BUTTON_PIN)==0){
            for (int i = 0; i<10; i++){
            printf("Ground_Speed: %02d\n",i+60);
        }
        printf("\n----------------------------------------------\n\n");
        for (int i = 0; i<10; i++){
            printf("MCM_Voltage_Info: %02d\n",i+21);
        }
        }
        else{
        }
    }

}
