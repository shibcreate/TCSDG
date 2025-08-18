#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void init_uart(void);
void task_uart(void *pvParameters);