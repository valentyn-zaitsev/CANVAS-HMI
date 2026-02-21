#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_types.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_st7796.h"
#include "esp_lcd_touch_gt911.h"
#include "can_manager.h"
#include "vehicle_data.h"

static const char *TAG = "CYD_35";

// ========================================
// LCD Pin Configuration
// ========================================
#define LCD_HOST        SPI2_HOST
#define PIN_NUM_SCLK    14
#define PIN_NUM_MOSI    13
#define PIN_NUM_MISO    12
#define PIN_NUM_LCD_CS  15
#define PIN_NUM_LCD_DC  2
#define PIN_NUM_BK_LIGHT 27

// ========================================
// Touch Pin Configuration
// ========================================
#define I2C_MASTER_SCL_IO 32
#define I2C_MASTER_SDA_IO 33
#define PIN_NUM_TOUCH_INT 21
#define PIN_NUM_TOUCH_RST 25


void blink_task(void* arg) {
    // Use GPIO 26 instead of GPIO 4 (GPIO 4 is used for CAN_RX)
    gpio_reset_pin(26);
    gpio_set_direction(26, GPIO_MODE_OUTPUT);
    while(1) {
        gpio_set_level(26, 0); // Turn on LED
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(26, 1); // Turn off LED
        vTaskDelay(pdMS_TO_TICKS(500));
        //ESP_LOGI(TAG, "Blink!"); // Log to console
    }
}

static lv_color_t get_dynamic_color(int value) {
    return lv_color_mix(lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_LIME), (value * 255) / 100);
}

static void arc_loader_event_cb(lv_event_t * e) {
    lv_obj_t * arc = lv_event_get_target(e);
    lv_obj_t * label = (lv_obj_t *)lv_event_get_user_data(e);

    int val = (int)lv_arc_get_value(arc);
    lv_label_set_text_fmt(label, "%d", val);

    // OPTIMIZATION: Update style only when value changes significantly (step > 1)
    static int last_val = -1;
    if(abs(val - last_val) > 1) { 
        lv_color_t dyn_color = get_dynamic_color(val);
        lv_obj_set_style_arc_color(arc, dyn_color, LV_PART_INDICATOR);
        lv_obj_set_style_border_color(arc, dyn_color, LV_PART_KNOB);
        last_val = val;
    }
}

void draw_pro_dashboard(void) {
    if (lvgl_port_lock(100)) {
        lv_obj_t * scr = lv_screen_active();
        lv_obj_clean(scr);
        lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

        // Create 6 color test rectangles in 3x2 grid (3 columns, 2 rows)
        const int box_w = 160;
        const int box_h = 160;

        // Row 1
        // RED
        lv_obj_t * red_box = lv_obj_create(scr);
        lv_obj_set_size(red_box, box_w, box_h);
        lv_obj_set_pos(red_box, 0, 0);
        lv_obj_set_style_bg_color(red_box, lv_palette_main(LV_PALETTE_RED), 0);
        lv_obj_set_style_border_width(red_box, 0, 0);
        lv_obj_t * red_label = lv_label_create(red_box);
        lv_label_set_text(red_label, "RED");
        lv_obj_set_style_text_color(red_label, lv_color_white(), 0);
        lv_obj_center(red_label);

        // GREEN
        lv_obj_t * green_box = lv_obj_create(scr);
        lv_obj_set_size(green_box, box_w, box_h);
        lv_obj_set_pos(green_box, box_w, 0);
        lv_obj_set_style_bg_color(green_box, lv_palette_main(LV_PALETTE_GREEN), 0);
        lv_obj_set_style_border_width(green_box, 0, 0);
        lv_obj_t * green_label = lv_label_create(green_box);
        lv_label_set_text(green_label, "GREEN");
        lv_obj_set_style_text_color(green_label, lv_color_black(), 0);
        lv_obj_center(green_label);

        // BLUE
        lv_obj_t * blue_box = lv_obj_create(scr);
        lv_obj_set_size(blue_box, box_w, box_h);
        lv_obj_set_pos(blue_box, box_w * 2, 0);
        lv_obj_set_style_bg_color(blue_box, lv_palette_main(LV_PALETTE_BLUE), 0);
        lv_obj_set_style_border_width(blue_box, 0, 0);
        lv_obj_t * blue_label = lv_label_create(blue_box);
        lv_label_set_text(blue_label, "BLUE");
        lv_obj_set_style_text_color(blue_label, lv_color_white(), 0);
        lv_obj_center(blue_label);

        // Row 2
        // YELLOW
        lv_obj_t * yellow_box = lv_obj_create(scr);
        lv_obj_set_size(yellow_box, box_w, box_h);
        lv_obj_set_pos(yellow_box, 0, box_h);
        lv_obj_set_style_bg_color(yellow_box, lv_palette_main(LV_PALETTE_YELLOW), 0);
        lv_obj_set_style_border_width(yellow_box, 0, 0);
        lv_obj_t * yellow_label = lv_label_create(yellow_box);
        lv_label_set_text(yellow_label, "YELLOW");
        lv_obj_set_style_text_color(yellow_label, lv_color_black(), 0);
        lv_obj_center(yellow_label);

        // CYAN
        lv_obj_t * cyan_box = lv_obj_create(scr);
        lv_obj_set_size(cyan_box, box_w, box_h);
        lv_obj_set_pos(cyan_box, box_w, box_h);
        lv_obj_set_style_bg_color(cyan_box, lv_palette_main(LV_PALETTE_CYAN), 0);
        lv_obj_set_style_border_width(cyan_box, 0, 0);
        lv_obj_t * cyan_label = lv_label_create(cyan_box);
        lv_label_set_text(cyan_label, "CYAN");
        lv_obj_set_style_text_color(cyan_label, lv_color_black(), 0);
        lv_obj_center(cyan_label);

        // MAGENTA
        lv_obj_t * magenta_box = lv_obj_create(scr);
        lv_obj_set_size(magenta_box, box_w, box_h);
        lv_obj_set_pos(magenta_box, box_w * 2, box_h);
        lv_obj_set_style_bg_color(magenta_box, lv_palette_main(LV_PALETTE_PINK), 0);
        lv_obj_set_style_border_width(magenta_box, 0, 0);
        lv_obj_t * magenta_label = lv_label_create(magenta_box);
        lv_label_set_text(magenta_label, "MAGENTA");
        lv_obj_set_style_text_color(magenta_label, lv_color_white(), 0);
        lv_obj_center(magenta_label);

        lvgl_port_unlock();
    }
}

void app_main(void) {

    xTaskCreate(blink_task, "blink", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "Starting initialization...");

    esp_log_level_set("esp_lvgl_port", ESP_LOG_DEBUG);

    gpio_config_t bk_gpio_config = { 
        .mode = GPIO_MODE_OUTPUT, 
        .pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT 
    };
    gpio_config(&bk_gpio_config);
    gpio_set_level(PIN_NUM_BK_LIGHT, 1);
    gpio_config(&bk_gpio_config);
    gpio_set_level(PIN_NUM_BK_LIGHT, 1);

    // ========================================
    // Step 1: Enable LCD Backlight
    // ========================================
    gpio_config(&bk_gpio_config);
    gpio_set_level(PIN_NUM_BK_LIGHT, 1);

    // ========================================
    // Step 2: Initialize SPI Bus
    // ========================================
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK, .mosi_io_num = PIN_NUM_MOSI, .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 480 * 320 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // ========================================
    // Step 3: Initialize ST7796 Display Driver
    // ========================================
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_LCD_DC, .cs_gpio_num = PIN_NUM_LCD_CS,
        .pclk_hz = 40 * 1000 * 1000, .lcd_cmd_bits = 8, .lcd_param_bits = 8, .spi_mode = 0, .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,
        .bits_per_pixel = 16,
        .rgb_endian = LCD_RGB_ENDIAN_BGR,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7796(io_handle, &panel_config, &panel_handle));
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);

    esp_lcd_panel_invert_color(panel_handle, false);
    esp_lcd_panel_disp_on_off(panel_handle, true);

    // ========================================
    // Step 4: Initialize I2C for Touch Controller
    // ========================================
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER, 
        .sda_io_num = I2C_MASTER_SDA_IO, 
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, 
        .scl_pullup_en = GPIO_PULLUP_ENABLE, 
        .master.clk_speed = 400000,
    };
    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0);

    esp_lcd_touch_handle_t touch_handle = NULL;
    esp_lcd_touch_config_t touch_config = {
        .x_max = 320,
        .y_max = 480,
        .rst_gpio_num = 25,
        .int_gpio_num = -1,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = true, 
            .mirror_x = false, 
            .mirror_y = true 
        },
    };

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_NUM_0, &tp_io_config, &tp_io_handle);
    esp_lcd_touch_new_i2c_gt911(tp_io_handle, &touch_config, &touch_handle);

    // ========================================
    // Step 5: Initialize LVGL Port
    // ========================================
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    // Display configuration with optimized buffer settings
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = 480 * 20,          // Smaller buffer for better performance
        .double_buffer = false,           // Single buffer mode
        .hres = 480,
        .vres = 320,
        .monochrome = false,
        // Note: In newer versions, rotation is a structure requiring additional braces
        .rotation = {
            .swap_xy = true,
            .mirror_x = true,
            .mirror_y = true,
        },
        .color_format = LV_COLOR_FORMAT_RGB565,
        .flags = {
            .buff_dma = true,             // Enable DMA buffer
            .swap_bytes = true,           // Color byte order fix
        }
    };

    lv_display_t * disp_handle = lvgl_port_add_disp(&disp_cfg);

    // ========================================
    // Step 6: Initialize LVGL Touch Input
    // ========================================
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp_handle,
        .handle = touch_handle
    };
    lvgl_port_add_touch(&touch_cfg);


    // Initialize CAN Manager
    ESP_LOGI(TAG, "Initializing CAN Manager...");
    if (can_manager_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CAN Manager");
    } else {
        if (can_manager_start() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start CAN Manager");
        } else {
            ESP_LOGI(TAG, "CAN Manager started successfully");
        }
    }

    draw_pro_dashboard();

    // ========================================
    // Main Loop
    // ========================================
    while (1) {
        // Touch handling example (currently disabled):
        // esp_lcd_touch_point_data_t data;  // Structure for single touch point data
        // uint8_t touch_cnt = 0;            // Variable to store touch count

        // // 1. Poll the touch controller chip
        // esp_lcd_touch_read_data(touch_handle);

        // // 2. Get touch data (requires exactly 4 arguments)
        // esp_err_t err = esp_lcd_touch_get_data(touch_handle, &data, &touch_cnt, 1);

        // // 3. If reading is successful and at least one touch point exists
        // if (err == ESP_OK && touch_cnt > 0) {
        //     ESP_LOGI("TOUCH_DEBUG", "X: %d, Y: %d", data.x, data.y);
        // }

        // Alternative touch reading method:
        // esp_lcd_touch_read_data(touch_handle);  // Read from hardware

        // esp_lcd_touch_point_data_t point;
        // uint8_t touch_cnt = 0;

        // // Get data into structure
        // if (esp_lcd_touch_get_data(touch_handle, &point, &touch_cnt, 1) == ESP_OK) {
        //     if (touch_cnt > 0) {
        //         // Coordinates are now available in point.x and point.y
        //         printf("X: %u, Y: %u\n", point.x, point.y);
        //     }
        // }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}