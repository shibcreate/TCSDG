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
    int *param;
    int param_val;
    bool *flag;
    TickType_t *end_time;
} uart_cmd_t;

// External state
extern int PLTargetPower, RegenMode, EfficiencyMode;
extern bool is_pl, is_regen, is_eff;
extern TickType_t pl_end, regen_end, eff_end;

// Command mapping
static uart_cmd_t commands[] = {
    // PLTargetPower
    { '1', &PLTargetPower, 1, &is_pl, &pl_end },
    { '2', &PLTargetPower, 2, &is_pl, &pl_end },
    { '3', &PLTargetPower, 3, &is_pl, &pl_end },

    // RegenMode
    { '4', &RegenMode, 1, &is_regen, &regen_end },
    { '5', &RegenMode, 2, &is_regen, &regen_end },

    // EfficiencyMode
    { '6', &EfficiencyMode, 1, &is_eff, &eff_end },
    { '7', &EfficiencyMode, 2, &is_eff, &eff_end },
    { '8', &EfficiencyMode, 3, &is_eff, &eff_end }
};
#define NUM_CMDS (sizeof(commands)/sizeof(commands[0]))

// Queue handle for UART events
static QueueHandle_t uart_queue;

void init_uart(void) {
    uart_config_t cfg = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &cfg));

    // Install driver with event queue enabled
    ESP_ERROR_CHECK(uart_driver_install(
        UART_PORT_NUM,
        UART_BUF_SIZE, UART_BUF_SIZE,
        20, &uart_queue, 0
    ));
}

void task_uart(void *pv) {
    uart_event_t event;
    uint8_t buf[128];

    while (1) {
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA: {
                    int len = uart_read_bytes(UART_PORT_NUM, buf, sizeof(buf)-1, 0);
                    if (len > 0) {
                        buf[len] = '\0';
                        char c = buf[0];

                        bool found = false;
                        for (int i = 0; i < NUM_CMDS; i++) {
                            if (commands[i].cmd == c) {
                                *(commands[i].param) = commands[i].param_val;
                                *(commands[i].flag) = true;
                                *(commands[i].end_time) = xTaskGetTickCount() + pdMS_TO_TICKS(10000); // 40s
                                ESP_LOGI(TAG, "Activated cmd %c", c);
                                found = true;
                                break;
                            }
                        }
                        if (!found) {
                            ESP_LOGW(TAG, "Unknown cmd: %c", c);
                        }
                    }
                    break;
                }

                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "HW FIFO overflow");
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart_queue);
                    break;

                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "Ring buffer full");
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart_queue);
                    break;

                default:
                    ESP_LOGD(TAG, "UART event type: %d", event.type);
                    break;
            }
        }
    }
}