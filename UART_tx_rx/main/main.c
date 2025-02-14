#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#define TAG "UART_SERIAL"
#define UART_NUM UART_NUM_0
#define BUF_SIZE 128

void app_main() {
    
    printf("%s: UART Serial Example Started\n", TAG);
    uint8_t buffer[BUF_SIZE];
    int len;
    int counter = 0;  // Counter to track time for logging

    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    printf("%s: Open PuTTY, connect to the correct COM port, type your message, and press Enter to send.\n", TAG);
    while (1) {
        int number = rand()%100;
        // Read data from UART
        len = uart_read_bytes(UART_NUM, buffer, BUF_SIZE - 1, 10 / portTICK_PERIOD_MS);
        if (len > 0) {
            buffer[len] = '\0'; // Null-terminate received data
            printf("%s: Received: %s\n", TAG, buffer);

            // Echo data back
            uart_write_bytes(UART_NUM, (const char*) buffer, len);
        }

        // Print periodic message every 5 seconds
        if (counter >= 10) {  // 500 10ms = 5000ms (5 seconds)
            printf("%s: Ground_Speed: %d\n", TAG, number);
            printf("%s: MCM_Voltage_Info: %d\n", TAG, number);
            counter = 0;  // Reset counter
        }

        counter++;  // Increment counter every loop iteration
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Delay to avoid busy-waiting
    }
}

/*

printf("Ground_Speed: %02d\n",i+60);

printf("MCM_Voltage_Info: %02d\n",i+21);

*/