/*	TWAI Network simple sending data Example

	This example code is in the Public Domain (or CC0 licensed, at your option.)

	Unless required by applicable law or agreed to in writing, this
	software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
	CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include <time.h>
#include "driver/twai.h" // Update from V4.2


static const char *TAG = "SENDING";

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

static const twai_general_config_t g_config =
	TWAI_GENERAL_CONFIG_DEFAULT(CONFIG_CTX_GPIO, CONFIG_CRX_GPIO, TWAI_MODE_NORMAL);

void fill_random_data(twai_message_t *msg) {
    for (int i = 0; i < msg->data_length_code; i++) {
        msg->data[i] = rand() % 256;
    }
}

	static void twai_sending_task(void *arg)
	{
		ESP_LOGI(pcTaskGetName(0), "task start");
	
		twai_message_t messages[] = {
			{ .identifier = 0x100, .extd = 0, .data_length_code = 8, .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x0, 0x00, 0x01} },
			{ .identifier = 0xA5,  .extd = 0, .data_length_code = 8, .data = {0x00, 0x00, 0xD2, 0x04, 0x00, 0x00, 0x00, 0x00} },
			{ .identifier = 0xA6,  .extd = 0, .data_length_code = 8, .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7B, 0x00} },
			{ .identifier = 0xA7,  .extd = 0, .data_length_code = 8, .data = {0xED, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },
			{ .identifier = 0xAA,  .extd = 0, .data_length_code = 8, .data = {0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x01, 0x00} },
			{ .identifier = 0xAC,  .extd = 0, .data_length_code = 8, .data = {0xB0, 0x04, 0x7E, 0x04, 0x00, 0x00, 0x00, 0x00} },
			{ .identifier = 0xC0,  .extd = 0, .data_length_code = 8, .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x84, 0x03} },
			{ .identifier = 0x501, .extd = 0, .data_length_code = 8, .data = {0x00, 0x00, 0x00, 0x00, 0xB0, 0x04, 0xC0, 0x12} },
			{ .identifier = 0x502, .extd = 0, .data_length_code = 8, .data = {0xD2, 0x04, 0xD3, 0x04, 0xD4, 0x04, 0xD5, 0x04} },
			{ .identifier = 0x505, .extd = 0, .data_length_code = 8, .data = {0xD2, 0x04, 0xD3, 0x04, 0xD4, 0x04, 0xD5, 0x04} },
			{ .identifier = 0x506, .extd = 0, .data_length_code = 8, .data = {0x11111111,0b00010101,0x00000001, 0x00,0x00000001,0x00, 0x00000001, 0x00} },
			{ .identifier = 0x507, .extd = 0, .data_length_code = 8, .data = {0xD2, 0x04, 0xD3, 0x04, 0xD4, 0x04, 0xD5, 0x04} },
			{ .identifier = 0x508, .extd = 0, .data_length_code = 8, .data = {0xD2, 0x04, 0xD3, 0x04, 0xD4, 0x04, 0xD5, 0x04} },
			{ .identifier = 0x50A, .extd = 0, .data_length_code = 8, .data = {0x58, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },
			{ .identifier = 0x50B, .extd = 0, .data_length_code = 8, .data = {0x01, 0x01, 0xD0, 0x07, 0x7D, 0x00, 0xFF, 0x00} },
			{ .identifier = 0x50C, .extd = 0, .data_length_code = 8, .data = {0xF1, 0xFF, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00} },
			{ .identifier = 0x50E, .extd = 0, .data_length_code = 8, .data = {0xD2, 0x04, 0xD3, 0x04, 0xD4, 0x04, 0xD5, 0x04} },
			{ .identifier = 0x50F, .extd = 0, .data_length_code = 8, .data = {0xD2, 0x04, 0xD3, 0x04, 0xD4, 0x04, 0xD5, 0x04} },
			{ .identifier = 0x511, .extd = 0, .data_length_code = 8, .data = {0xD2, 0x04, 0xD3, 0x04, 0xD4, 0x04, 0xD5, 0x04} },
			{ .identifier = 0x512, .extd = 0, .data_length_code = 8, .data = {0xD2, 0x04, 0xD3, 0x04, 0xD4, 0x04, 0xD5, 0x04} },
			{ .identifier = 0x513, .extd = 0, .data_length_code = 8, .data = {0xD2, 0x04, 0xD3, 0x04, 0xD4, 0x04, 0xD5, 0x04} },
			{ .identifier = 0x602, .extd = 0, .data_length_code = 8, .data = {0xD2, 0x04, 0xD3, 0x04, 0xD4, 0x04, 0xD5, 0x04} },
			{ .identifier = 0x610, .extd = 0, .data_length_code = 8, .data = {0xD2, 0x04, 0xD3, 0x04, 0xD4, 0x04, 0xD5, 0x04} },
			{ .identifier = 0x620, .extd = 0, .data_length_code = 8, .data = {0b01111110,0b11111110,0x80,0x00,0xA0, 0x86, 0x01, 0x00} },
			{ .identifier = 0x621, .extd = 0, .data_length_code = 8, .data = {0xD2, 0x04, 0xD3, 0x04, 0xD4, 0x04, 0xD5, 0x04} },
			{ .identifier = 0x623, .extd = 0, .data_length_code = 8, .data = {0x10, 0x27,  0x20, 0x4E,  0x1E, 0x00,  0x14, 0x00} },
		};
	
		size_t num_msgs = sizeof(messages) / sizeof(messages[0]);
	
		while (1) {
			for (int i = 0; i < num_msgs; i++) {
				fill_random_data(&messages[i]);
				twai_status_info_t status_info;
				twai_get_status_info(&status_info);
				if (status_info.state != TWAI_STATE_RUNNING) {
					ESP_LOGE(TAG, "TWAI not running");
					continue;
				}
	
				esp_err_t ret = twai_transmit(&messages[i], pdMS_TO_TICKS(100));
				if (ret == ESP_OK) {
					ESP_LOGI(TAG, "Sent ID 0x%03" PRIX32, messages[i].identifier);
				} else {
					ESP_LOGE(TAG, "Send failed: %s", esp_err_to_name(ret));
				}
	
				//vTaskDelay(pdMS_TO_TICKS(500)); // Wait between messages
			}
		}
	
		vTaskDelete(NULL);
	}
	

void app_main()
{
	srand(time(NULL));
	// Install and start TWAI driver
	ESP_LOGI(TAG, "%s",BITRATE);
	ESP_LOGI(TAG, "CTX_GPIO=%d",CONFIG_CTX_GPIO);
	ESP_LOGI(TAG, "CRX_GPIO=%d",CONFIG_CRX_GPIO);

	ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
	ESP_LOGI(TAG, "Driver installed");
	ESP_ERROR_CHECK(twai_start());
	ESP_LOGI(TAG, "Driver started");

	// Start task
	xTaskCreate(twai_sending_task, "TWAI", 1024*4, NULL, 2, NULL);
}