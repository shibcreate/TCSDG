#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_types.h"
#include "esp_lcd_io_i2c.h"
#include "esp_lcd_panel_ssd1306.h"

// Define I2C parameters
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA      GPIO_NUM_10  // Change to your SDA pin
#define I2C_MASTER_SCL      GPIO_NUM_9   // Change to your SCL pin
#define I2C_FREQ_HZ         400000       // 400kHz I2C speed
#define SSD1306_I2C_ADDR    0x3C         // Default SSD1306 I2C address

static const char *TAG = "SSD1306_LCD";

i2c_master_bus_handle_t i2c_bus = NULL;
esp_lcd_panel_io_handle_t io_handle = NULL;
esp_lcd_panel_handle_t panel_handle = NULL;

// Initialize I2C bus
void init_i2c_bus() {
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));
    ESP_LOGI(TAG, "I2C bus initialized successfully");
}

// Initialize LCD IO interface
void init_lcd_io() {
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = SSD1306_I2C_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
        .control_phase_bytes = 1,  // As per SSD1306 datasheet
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_bit_offset = 6,  // Data/Command bit offset in address (SSD1306-specific)
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));
    ESP_LOGI(TAG, "LCD IO handle created successfully");
}
//esp_lcd_new_panel_ssd1306()
//esp_lcd_panel_dev_config_t panel_config
// Initialize LCD panel
void init_lcd_panel() {
    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = 64,  // Height of the display
    };

    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,         // Monochrome display
        .reset_gpio_num = -1,        // No reset pin used
        .vendor_config = &ssd1306_config, // Pass SSD1306-specific configuration
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    // Perform basic initialization and turn on the display
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "LCD panel initialized successfully");
}

// Display text (requires bitmap conversion or LVGL)
void lcd_display_text(const char *text) {
    // NOTE: Text rendering requires converting text to a bitmap or using a library like LVGL.
    ESP_LOGI(TAG, "Text rendering is not directly supported in this example.");
}

// Main application
void app_main() {
    init_i2c_bus();
    init_lcd_io();
    init_lcd_panel();

    // Example: Clear screen (fill with black)
    uint8_t clear_screen[128 * 64 / 8] = {0};  // Monochrome buffer (1 bit per pixel)
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 128, 64, clear_screen));

    ESP_LOGI(TAG, "Screen cleared successfully.");
}
