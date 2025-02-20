#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_types.h"
#include "esp_lcd_io_i2c.h"
#include "esp_lcd_panel_ssd1306.h"
#include "lvgl/lvgl.h"
#include "driver/gpio.h"

// Define I2C parameters
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA      GPIO_NUM_10 // Change to your SDA pin
#define I2C_MASTER_SCL      GPIO_NUM_9   // Change to your SCL pin
#define I2C_FREQ_HZ         100000       // 400kHz I2C speed
#define SSD1306_I2C_ADDR    0x3C         // Default SSD1306 I2C address

static const char *TAG = "SSD1306_LCD";

i2c_master_bus_handle_t i2c_handle = NULL;
esp_lcd_panel_io_handle_t io_handle = NULL;
esp_lcd_panel_handle_t panel_handle = NULL;

// LVGL display buffers
static lv_color_t buf1[128 * 8];  // 1/8th of 128x64 screen
static lv_color_t buf2[128 * 8];  // Optional second buffer

void gpio_configuration() {
    gpio_set_pull_mode(I2C_MASTER_SDA, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(I2C_MASTER_SCL, GPIO_PULLUP_ONLY);
}


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
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_handle));
    ESP_LOGI(TAG, "I2C bus initialized successfully");
}

i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,  // Use 7-bit addressing
    .device_address = 0x3C,                // MPU-6050 default address
    .scl_speed_hz = 100000,                // Set clock speed to 100 kHz
};

// Handle for the MPU-6050 device
i2c_master_dev_handle_t dev_handle;

// Initialize LCD IO interface
void init_lcd_io() {
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = SSD1306_I2C_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
        .control_phase_bytes = 1,  
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_bit_offset = 6,  
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_handle, &io_config, &io_handle));
    ESP_LOGI(TAG, "LCD IO handle created successfully");
}

// LVGL flush callback function
void lvgl_display_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);

    if (!panel) {
        ESP_LOGE(TAG, "Panel handle is NULL!");
        lv_display_flush_ready(disp);
        return;
    }

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map));

    lv_display_flush_ready(disp);  // Notify LVGL flushing is done
}

// Setup LVGL Display for v9.0.0+
void setup_lvgl_display() {
    lv_display_t *disp = lv_display_create(128, 64);  // Create LVGL display instance

    if (!disp) {
        ESP_LOGE(TAG, "Failed to create LVGL display!");
        return;
    }

    // Set flush callback
    lv_display_set_flush_cb(disp, lvgl_display_flush);

    // Set display buffers
    lv_display_set_buffers(disp, buf1, buf2, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);

    // Associate the LVGL display with our LCD panel
    lv_display_set_user_data(disp, panel_handle);

    ESP_LOGI(TAG, "LVGL display initialized successfully.");
}

// Initialize LCD panel and LVGL
void init_lcd_panel() {
    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = 64,  
    };

    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,         
        .reset_gpio_num = -1,       
        .vendor_config = &ssd1306_config, 
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    // Perform basic initialization
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "LCD panel initialized successfully");

    // Initialize LVGL
    lv_init();

    // Setup LVGL display
    setup_lvgl_display();
}

// Display text using LVGL
void lcd_display_text(const char *text) {
    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, text);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    uint8_t buffer1 = 0xA4;
    i2c_master_transmit(dev_handle, &buffer1,1,100);
}

// Main application
void app_main() {
    
    init_i2c_bus();
    init_lcd_io();
    init_lcd_panel();
    gpio_configuration();
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_handle, &dev_cfg, &dev_handle));
    lcd_display_text("Hello, World!");

    while (1) {
        lv_task_handler();  // Process LVGL events
        vTaskDelay(pdMS_TO_TICKS(10));  // Delay 10ms
    }
}
