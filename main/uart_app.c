#include "uart_app.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "lora_app.h"

#define UART_PORT_NUM      (UART_NUM_0)
#define UART_BAUD_RATE     (115200)
#define UART_BUF_SIZE      (1024)

static const char *TAG = "UART";

typedef struct {
    char cmd;
    int *param;             // optional (mode selector)
    int param_val;
    bool *flag;
    TickType_t *end_time;
} uart_cmd_t;

// External state from lora_app
extern int drsMode, plMode, torqueLimit;
extern bool is_drs, is_pl, is_torque;
extern TickType_t drs_end, pl_end, torque_end;

static uart_cmd_t commands[] = {
    { '1', &drsMode, 1, &is_drs, &drs_end },
    { '0', &drsMode, 0, &is_drs, &drs_end },
    { '2', &plMode,  1, &is_pl,  &pl_end },
    { '3', &plMode,  2, &is_pl,  &pl_end },
    { '4', &torqueLimit, 200, &is_torque, &torque_end },
    { '5', &torqueLimit, 150, &is_torque, &torque_end }
};
#define NUM_CMDS (sizeof(commands)/sizeof(commands[0]))

void init_uart(void) {
    uart_config_t cfg = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &cfg));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));
}

void task_uart(void *pv) {
    uint8_t buf[128];
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, buf, sizeof(buf), 20/portTICK_PERIOD_MS);
        if (len > 0) {
            buf[len] = '\0';
            char c = buf[0];

            bool found = false;
            for (int i=0;i<NUM_CMDS;i++) {
                if (commands[i].cmd == c) {
                    *(commands[i].param) = commands[i].param_val;
                    *(commands[i].flag) = true;
                    *(commands[i].end_time) = xTaskGetTickCount() + pdMS_TO_TICKS(40000);
                    ESP_LOGI(TAG, "Activated cmd %c", c);
                    found = true;
                    break;
                }
            }
            if (!found) {
                ESP_LOGW(TAG, "Unknown cmd: %c", c);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}