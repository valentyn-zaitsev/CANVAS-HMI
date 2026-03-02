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
#include "can_driver.h"
#include "can_sniffer.h"
#include "mercedes_decode.h"
#include "vehicle_data.h"
#include "obd2_pids.h"
#include <inttypes.h>

static const char *TAG = "CYD_35";

// LCD Pin Configuration
#define LCD_HOST        SPI2_HOST
#define PIN_NUM_SCLK    14
#define PIN_NUM_MOSI    13
#define PIN_NUM_MISO    12
#define PIN_NUM_LCD_CS  15
#define PIN_NUM_LCD_DC  2
#define PIN_NUM_BK_LIGHT 27

// Touch Pin Configuration
#define I2C_MASTER_SCL_IO 32
#define I2C_MASTER_SDA_IO 33
#define PIN_NUM_TOUCH_INT 21
#define PIN_NUM_TOUCH_RST 25

// Screen dimensions
#define SCREEN_W 480
#define SCREEN_H 320

// Navigation zones
#define NAV_ZONE_W 60

// Number of screens
#define NUM_SCREENS 5
static int current_screen = 0;

// Screen containers
static lv_obj_t *screen_gauges = NULL;
static lv_obj_t *screen_sensors = NULL;
static lv_obj_t *screen_diag = NULL;
static lv_obj_t *screen_debug = NULL;
static lv_obj_t *screen_sniffer = NULL;
static lv_obj_t *screens[NUM_SCREENS];

// Screen 1 (Gauges) labels
static lv_obj_t *rpm_value_label = NULL;
static lv_obj_t *speed_value_label = NULL;
static lv_obj_t *temp_value_label = NULL;
static lv_obj_t *fuel_value_label = NULL;
static lv_obj_t *status_dot = NULL;

// Screen 2 (Sensors) labels
static lv_obj_t *sensor_labels[10] = {NULL};

// Screen 3 (Diagnostics) labels
static lv_obj_t *diag_status_label = NULL;
static lv_obj_t *diag_stats_label = NULL;
static lv_obj_t *diag_detail_label = NULL;

// Screen 4 (CAN Debug) labels
static lv_obj_t *debug_bus_label = NULL;
static lv_obj_t *debug_counters_label = NULL;
static lv_obj_t *debug_errors_label = NULL;

// Screen 5 (Sniffer) labels
static lv_obj_t *sniffer_summary_label = NULL;
#define SNIFFER_DISPLAY_ROWS 10
static lv_obj_t *sniffer_row_labels[SNIFFER_DISPLAY_ROWS] = {NULL};
static int sniffer_scroll_offset = 0;

// Navigation dots and buttons
static lv_obj_t *nav_dots[NUM_SCREENS] = {NULL};
static lv_obj_t *btn_prev = NULL;
static lv_obj_t *btn_next = NULL;
static lv_obj_t *screen_title_label = NULL;

// ============================================================================
// Blink task (heartbeat LED on GPIO 26)
// ============================================================================
void blink_task(void *arg) {
    gpio_reset_pin(26);
    gpio_set_direction(26, GPIO_MODE_OUTPUT);
    while (1) {
        gpio_set_level(26, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(26, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ============================================================================
// Navigation
// ============================================================================
static const char *screen_titles[] = {"GAUGES", "SENSORS", "DIAGNOSTICS", "CAN DEBUG", "SNIFFER"};

static void update_nav_ui(void) {
    // Update dots
    for (int i = 0; i < NUM_SCREENS; i++) {
        if (nav_dots[i]) {
            lv_obj_set_style_bg_color(nav_dots[i], lv_color_white(), 0);
            lv_obj_set_style_bg_opa(nav_dots[i],
                i == current_screen ? LV_OPA_COVER : LV_OPA_30, 0);
        }
    }
    // Buttons always visible (circular navigation)
    if (btn_prev) lv_obj_clear_flag(btn_prev, LV_OBJ_FLAG_HIDDEN);
    if (btn_next) lv_obj_clear_flag(btn_next, LV_OBJ_FLAG_HIDDEN);
    // Update title
    if (screen_title_label) {
        lv_label_set_text(screen_title_label, screen_titles[current_screen]);
    }
}

static void switch_screen(int new_screen) {
    // Wrap around for circular navigation
    if (new_screen < 0) new_screen = NUM_SCREENS - 1;
    if (new_screen >= NUM_SCREENS) new_screen = 0;
    if (new_screen == current_screen) return;
    lv_obj_add_flag(screens[current_screen], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(screens[new_screen], LV_OBJ_FLAG_HIDDEN);
    current_screen = new_screen;
    update_nav_ui();
}

static void btn_prev_cb(lv_event_t *e) {
    switch_screen(current_screen - 1);
}

static void btn_next_cb(lv_event_t *e) {
    switch_screen(current_screen + 1);
}

// ============================================================================
// Screen 1: Main Gauges
// ============================================================================
static void build_gauges_screen(lv_obj_t *parent) {
    // RPM - top left, large
    lv_obj_t *rpm_box = lv_obj_create(parent);
    lv_obj_set_size(rpm_box, 225, 125);
    lv_obj_set_pos(rpm_box, 5, 5);
    lv_obj_set_style_bg_color(rpm_box, lv_color_make(40, 10, 10), 0);
    lv_obj_set_style_border_color(rpm_box, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_border_width(rpm_box, 2, 0);
    lv_obj_set_style_radius(rpm_box, 8, 0);
    lv_obj_set_style_pad_all(rpm_box, 5, 0);
    lv_obj_clear_flag(rpm_box, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *rpm_title = lv_label_create(rpm_box);
    lv_label_set_text(rpm_title, "RPM");
    lv_obj_set_style_text_color(rpm_title, lv_palette_lighten(LV_PALETTE_RED, 2), 0);
    lv_obj_set_style_text_font(rpm_title, &lv_font_montserrat_14, 0);
    lv_obj_align(rpm_title, LV_ALIGN_TOP_LEFT, 0, 0);

    rpm_value_label = lv_label_create(rpm_box);
    lv_label_set_text(rpm_value_label, "---");
    lv_obj_set_style_text_color(rpm_value_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(rpm_value_label, &lv_font_montserrat_40, 0);
    lv_obj_align(rpm_value_label, LV_ALIGN_CENTER, 0, 10);

    // Speed - top right, large
    lv_obj_t *speed_box = lv_obj_create(parent);
    lv_obj_set_size(speed_box, 225, 125);
    lv_obj_set_pos(speed_box, 250, 5);
    lv_obj_set_style_bg_color(speed_box, lv_color_make(10, 40, 10), 0);
    lv_obj_set_style_border_color(speed_box, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_border_width(speed_box, 2, 0);
    lv_obj_set_style_radius(speed_box, 8, 0);
    lv_obj_set_style_pad_all(speed_box, 5, 0);
    lv_obj_clear_flag(speed_box, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *speed_title = lv_label_create(speed_box);
    lv_label_set_text(speed_title, "km/h");
    lv_obj_set_style_text_color(speed_title, lv_palette_lighten(LV_PALETTE_GREEN, 2), 0);
    lv_obj_set_style_text_font(speed_title, &lv_font_montserrat_14, 0);
    lv_obj_align(speed_title, LV_ALIGN_TOP_LEFT, 0, 0);

    speed_value_label = lv_label_create(speed_box);
    lv_label_set_text(speed_value_label, "---");
    lv_obj_set_style_text_color(speed_value_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(speed_value_label, &lv_font_montserrat_40, 0);
    lv_obj_align(speed_value_label, LV_ALIGN_CENTER, 0, 10);

    // Coolant temp - bottom left
    lv_obj_t *temp_box = lv_obj_create(parent);
    lv_obj_set_size(temp_box, 225, 100);
    lv_obj_set_pos(temp_box, 5, 140);
    lv_obj_set_style_bg_color(temp_box, lv_color_make(10, 10, 40), 0);
    lv_obj_set_style_border_color(temp_box, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_set_style_border_width(temp_box, 2, 0);
    lv_obj_set_style_radius(temp_box, 8, 0);
    lv_obj_set_style_pad_all(temp_box, 5, 0);
    lv_obj_clear_flag(temp_box, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *temp_title = lv_label_create(temp_box);
    lv_label_set_text(temp_title, "STEER");
    lv_obj_set_style_text_color(temp_title, lv_palette_lighten(LV_PALETTE_BLUE, 2), 0);
    lv_obj_set_style_text_font(temp_title, &lv_font_montserrat_12, 0);
    lv_obj_align(temp_title, LV_ALIGN_TOP_LEFT, 0, 0);

    temp_value_label = lv_label_create(temp_box);
    lv_label_set_text(temp_value_label, "---");
    lv_obj_set_style_text_color(temp_value_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(temp_value_label, &lv_font_montserrat_24, 0);
    lv_obj_align(temp_value_label, LV_ALIGN_CENTER, 0, 8);

    // Fuel level - bottom right
    lv_obj_t *fuel_box = lv_obj_create(parent);
    lv_obj_set_size(fuel_box, 225, 100);
    lv_obj_set_pos(fuel_box, 250, 140);
    lv_obj_set_style_bg_color(fuel_box, lv_color_make(40, 25, 5), 0);
    lv_obj_set_style_border_color(fuel_box, lv_palette_main(LV_PALETTE_ORANGE), 0);
    lv_obj_set_style_border_width(fuel_box, 2, 0);
    lv_obj_set_style_radius(fuel_box, 8, 0);
    lv_obj_set_style_pad_all(fuel_box, 5, 0);
    lv_obj_clear_flag(fuel_box, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *fuel_title = lv_label_create(fuel_box);
    lv_label_set_text(fuel_title, "GEAR");
    lv_obj_set_style_text_color(fuel_title, lv_palette_lighten(LV_PALETTE_ORANGE, 2), 0);
    lv_obj_set_style_text_font(fuel_title, &lv_font_montserrat_12, 0);
    lv_obj_align(fuel_title, LV_ALIGN_TOP_LEFT, 0, 0);

    fuel_value_label = lv_label_create(fuel_box);
    lv_label_set_text(fuel_value_label, "---");
    lv_obj_set_style_text_color(fuel_value_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(fuel_value_label, &lv_font_montserrat_24, 0);
    lv_obj_align(fuel_value_label, LV_ALIGN_CENTER, 0, 8);

    // CAN status dot (bottom center)
    status_dot = lv_obj_create(parent);
    lv_obj_set_size(status_dot, 12, 12);
    lv_obj_set_pos(status_dot, 234, 258);
    lv_obj_set_style_radius(status_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(status_dot, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_border_width(status_dot, 0, 0);
    lv_obj_clear_flag(status_dot, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *status_lbl = lv_label_create(parent);
    lv_label_set_text(status_lbl, "CAN");
    lv_obj_set_style_text_color(status_lbl, lv_color_make(120, 120, 120), 0);
    lv_obj_set_style_text_font(status_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(status_lbl, 250, 255);
}

// ============================================================================
// Screen 2: All Sensors
// ============================================================================
static const char *sensor_names[] = {
    "RPM", "Speed", "Steer Angle", "Steer Rate",
    "Gas Pedal", "Brake", "Gear",
    "WS FL", "WS FR", "Doors"
};
static void build_sensors_screen(lv_obj_t *parent) {
    lv_obj_t *title = lv_label_create(parent);
    lv_label_set_text(title, "ALL SENSORS");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(title, 10, 5);

    // 10 sensors in 2 columns, 5 rows
    for (int i = 0; i < 10; i++) {
        int col = i / 5;
        int row = i % 5;
        int x = 10 + col * 240;
        int y = 28 + row * 48;

        // Name label
        lv_obj_t *name_lbl = lv_label_create(parent);
        lv_label_set_text(name_lbl, sensor_names[i]);
        lv_obj_set_style_text_color(name_lbl, lv_color_make(150, 150, 150), 0);
        lv_obj_set_style_text_font(name_lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_pos(name_lbl, x, y);

        // Value label
        sensor_labels[i] = lv_label_create(parent);
        lv_label_set_text(sensor_labels[i], "---");
        lv_obj_set_style_text_color(sensor_labels[i], lv_color_white(), 0);
        lv_obj_set_style_text_font(sensor_labels[i], &lv_font_montserrat_20, 0);
        lv_obj_set_pos(sensor_labels[i], x, y + 14);
    }
}

// ============================================================================
// Screen 3: CAN Diagnostics
// ============================================================================
static void build_diag_screen(lv_obj_t *parent) {
    lv_obj_t *title = lv_label_create(parent);
    lv_label_set_text(title, "CAN DIAGNOSTICS");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(title, 10, 5);

    // Status box
    lv_obj_t *status_box = lv_obj_create(parent);
    lv_obj_set_size(status_box, 460, 80);
    lv_obj_set_pos(status_box, 10, 30);
    lv_obj_set_style_bg_color(status_box, lv_color_make(20, 20, 30), 0);
    lv_obj_set_style_border_color(status_box, lv_color_make(60, 60, 80), 0);
    lv_obj_set_style_border_width(status_box, 1, 0);
    lv_obj_set_style_radius(status_box, 6, 0);
    lv_obj_set_style_pad_all(status_box, 8, 0);
    lv_obj_clear_flag(status_box, LV_OBJ_FLAG_SCROLLABLE);

    diag_status_label = lv_label_create(status_box);
    lv_label_set_text(diag_status_label, "CAN: Initializing...");
    lv_obj_set_style_text_color(diag_status_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(diag_status_label, &lv_font_montserrat_16, 0);
    lv_obj_set_pos(diag_status_label, 0, 0);

    // Stats box
    lv_obj_t *stats_box = lv_obj_create(parent);
    lv_obj_set_size(stats_box, 460, 100);
    lv_obj_set_pos(stats_box, 10, 120);
    lv_obj_set_style_bg_color(stats_box, lv_color_make(20, 20, 30), 0);
    lv_obj_set_style_border_color(stats_box, lv_color_make(60, 60, 80), 0);
    lv_obj_set_style_border_width(stats_box, 1, 0);
    lv_obj_set_style_radius(stats_box, 6, 0);
    lv_obj_set_style_pad_all(stats_box, 8, 0);
    lv_obj_clear_flag(stats_box, LV_OBJ_FLAG_SCROLLABLE);

    diag_stats_label = lv_label_create(stats_box);
    lv_label_set_text(diag_stats_label, "Requests: 0\nResponses: 0\nErrors: 0\nSuccess: --");
    lv_obj_set_style_text_color(diag_stats_label, lv_color_make(180, 220, 180), 0);
    lv_obj_set_style_text_font(diag_stats_label, &lv_font_montserrat_16, 0);
    lv_obj_set_pos(diag_stats_label, 0, 0);

    // Detail box
    lv_obj_t *detail_box = lv_obj_create(parent);
    lv_obj_set_size(detail_box, 460, 55);
    lv_obj_set_pos(detail_box, 10, 230);
    lv_obj_set_style_bg_color(detail_box, lv_color_make(20, 20, 30), 0);
    lv_obj_set_style_border_color(detail_box, lv_color_make(60, 60, 80), 0);
    lv_obj_set_style_border_width(detail_box, 1, 0);
    lv_obj_set_style_radius(detail_box, 6, 0);
    lv_obj_set_style_pad_all(detail_box, 8, 0);
    lv_obj_clear_flag(detail_box, LV_OBJ_FLAG_SCROLLABLE);

    diag_detail_label = lv_label_create(detail_box);
    lv_label_set_text(diag_detail_label, "TX: GPIO 22  RX: GPIO 4  500kbps");
    lv_obj_set_style_text_color(diag_detail_label, lv_color_make(150, 150, 180), 0);
    lv_obj_set_style_text_font(diag_detail_label, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(diag_detail_label, 0, 0);
}

// ============================================================================
// Screen 4: CAN Debug (low-level TWAI controller info)
// ============================================================================
static void build_debug_screen(lv_obj_t *parent) {
    lv_obj_t *title = lv_label_create(parent);
    lv_label_set_text(title, "CAN BUS DEBUG");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(title, 10, 5);

    // Mode info (static)
    lv_obj_t *mode_lbl = lv_label_create(parent);
    lv_label_set_text(mode_lbl, "Mode: NO_ACK + Decode");
    lv_obj_set_style_text_color(mode_lbl, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(mode_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(mode_lbl, 10, 25);

    // Bus state box
    lv_obj_t *bus_box = lv_obj_create(parent);
    lv_obj_set_size(bus_box, 460, 55);
    lv_obj_set_pos(bus_box, 10, 48);
    lv_obj_set_style_bg_color(bus_box, lv_color_make(20, 20, 30), 0);
    lv_obj_set_style_border_color(bus_box, lv_color_make(60, 60, 80), 0);
    lv_obj_set_style_border_width(bus_box, 1, 0);
    lv_obj_set_style_radius(bus_box, 6, 0);
    lv_obj_set_style_pad_all(bus_box, 6, 0);
    lv_obj_clear_flag(bus_box, LV_OBJ_FLAG_SCROLLABLE);

    debug_bus_label = lv_label_create(bus_box);
    lv_label_set_text(debug_bus_label, "Bus: --\nTX err: --  RX err: --");
    lv_obj_set_style_text_color(debug_bus_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(debug_bus_label, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(debug_bus_label, 0, 0);

    // Counters box
    lv_obj_t *cnt_box = lv_obj_create(parent);
    lv_obj_set_size(cnt_box, 460, 75);
    lv_obj_set_pos(cnt_box, 10, 110);
    lv_obj_set_style_bg_color(cnt_box, lv_color_make(20, 20, 30), 0);
    lv_obj_set_style_border_color(cnt_box, lv_color_make(60, 60, 80), 0);
    lv_obj_set_style_border_width(cnt_box, 1, 0);
    lv_obj_set_style_radius(cnt_box, 6, 0);
    lv_obj_set_style_pad_all(cnt_box, 6, 0);
    lv_obj_clear_flag(cnt_box, LV_OBJ_FLAG_SCROLLABLE);

    debug_counters_label = lv_label_create(cnt_box);
    lv_label_set_text(debug_counters_label, "TX failed: --\nRX missed: --\nRX queued: --");
    lv_obj_set_style_text_color(debug_counters_label, lv_color_make(180, 220, 180), 0);
    lv_obj_set_style_text_font(debug_counters_label, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(debug_counters_label, 0, 0);

    // Error details box
    lv_obj_t *err_box = lv_obj_create(parent);
    lv_obj_set_size(err_box, 460, 65);
    lv_obj_set_pos(err_box, 10, 192);
    lv_obj_set_style_bg_color(err_box, lv_color_make(20, 20, 30), 0);
    lv_obj_set_style_border_color(err_box, lv_color_make(60, 60, 80), 0);
    lv_obj_set_style_border_width(err_box, 1, 0);
    lv_obj_set_style_radius(err_box, 6, 0);
    lv_obj_set_style_pad_all(err_box, 6, 0);
    lv_obj_clear_flag(err_box, LV_OBJ_FLAG_SCROLLABLE);

    debug_errors_label = lv_label_create(err_box);
    lv_label_set_text(debug_errors_label, "Bus errors: --\nArb lost: --\nGPIO TX:%d RX:%d");
    lv_obj_set_style_text_color(debug_errors_label, lv_color_make(220, 180, 180), 0);
    lv_obj_set_style_text_font(debug_errors_label, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(debug_errors_label, 0, 0);
}

// ============================================================================
// Screen 5: CAN Sniffer
// ============================================================================
static void sniffer_up_cb(lv_event_t *e) {
    if (sniffer_scroll_offset > 0) sniffer_scroll_offset--;
}
static void sniffer_down_cb(lv_event_t *e) {
    const sniffer_state_t *s = can_sniffer_get_state();
    if (sniffer_scroll_offset + SNIFFER_DISPLAY_ROWS < s->num_ids) sniffer_scroll_offset++;
}
static void sniffer_reset_cb(lv_event_t *e) {
    can_sniffer_reset();
    sniffer_scroll_offset = 0;
}

static void build_sniffer_screen(lv_obj_t *parent) {
    // Summary line
    sniffer_summary_label = lv_label_create(parent);
    lv_label_set_text(sniffer_summary_label, "IDs: 0  Msgs: 0");
    lv_obj_set_style_text_color(sniffer_summary_label, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_text_font(sniffer_summary_label, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(sniffer_summary_label, 10, 3);

    // Column headers
    lv_obj_t *hdr = lv_label_create(parent);
    lv_label_set_text(hdr, "ID       CNT   DATA");
    lv_obj_set_style_text_color(hdr, lv_color_make(150, 150, 150), 0);
    lv_obj_set_style_text_font(hdr, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(hdr, 10, 22);

    // Data rows
    for (int i = 0; i < SNIFFER_DISPLAY_ROWS; i++) {
        sniffer_row_labels[i] = lv_label_create(parent);
        lv_label_set_text(sniffer_row_labels[i], "");
        lv_obj_set_style_text_color(sniffer_row_labels[i], lv_color_white(), 0);
        lv_obj_set_style_text_font(sniffer_row_labels[i], &lv_font_montserrat_14, 0);
        lv_obj_set_pos(sniffer_row_labels[i], 10, 36 + i * 22);
    }

    // Scroll up button
    lv_obj_t *btn_up = lv_button_create(parent);
    lv_obj_set_size(btn_up, 50, 30);
    lv_obj_set_pos(btn_up, 370, 5);
    lv_obj_set_style_bg_color(btn_up, lv_color_make(60, 60, 80), 0);
    lv_obj_add_event_cb(btn_up, sniffer_up_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *up_lbl = lv_label_create(btn_up);
    lv_label_set_text(up_lbl, LV_SYMBOL_UP);
    lv_obj_set_style_text_color(up_lbl, lv_color_white(), 0);
    lv_obj_center(up_lbl);

    // Scroll down button
    lv_obj_t *btn_dn = lv_button_create(parent);
    lv_obj_set_size(btn_dn, 50, 30);
    lv_obj_set_pos(btn_dn, 425, 5);
    lv_obj_set_style_bg_color(btn_dn, lv_color_make(60, 60, 80), 0);
    lv_obj_add_event_cb(btn_dn, sniffer_down_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *dn_lbl = lv_label_create(btn_dn);
    lv_label_set_text(dn_lbl, LV_SYMBOL_DOWN);
    lv_obj_set_style_text_color(dn_lbl, lv_color_white(), 0);
    lv_obj_center(dn_lbl);

    // Reset button
    lv_obj_t *btn_rst = lv_button_create(parent);
    lv_obj_set_size(btn_rst, 60, 26);
    lv_obj_set_pos(btn_rst, 405, 248);
    lv_obj_set_style_bg_color(btn_rst, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_add_event_cb(btn_rst, sniffer_reset_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *rst_lbl = lv_label_create(btn_rst);
    lv_label_set_text(rst_lbl, "CLEAR");
    lv_obj_set_style_text_color(rst_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_font(rst_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(rst_lbl);
}

// ============================================================================
// Navigation bar (bottom of screen)
// ============================================================================
static void build_nav_bar(lv_obj_t *scr) {
    // Nav bar background
    lv_obj_t *nav_bar = lv_obj_create(scr);
    lv_obj_set_size(nav_bar, SCREEN_W, 40);
    lv_obj_set_pos(nav_bar, 0, SCREEN_H - 40);
    lv_obj_set_style_bg_color(nav_bar, lv_color_make(25, 25, 35), 0);
    lv_obj_set_style_border_color(nav_bar, lv_color_make(50, 50, 70), 0);
    lv_obj_set_style_border_width(nav_bar, 1, 0);
    lv_obj_set_style_border_side(nav_bar, LV_BORDER_SIDE_TOP, 0);
    lv_obj_set_style_radius(nav_bar, 0, 0);
    lv_obj_set_style_pad_all(nav_bar, 0, 0);
    lv_obj_clear_flag(nav_bar, LV_OBJ_FLAG_SCROLLABLE);

    // "<" button (left)
    btn_prev = lv_button_create(nav_bar);
    lv_obj_set_size(btn_prev, 70, 34);
    lv_obj_set_pos(btn_prev, 5, 3);
    lv_obj_set_style_bg_color(btn_prev, lv_color_make(60, 60, 80), 0);
    lv_obj_set_style_radius(btn_prev, 6, 0);
    lv_obj_add_event_cb(btn_prev, btn_prev_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *prev_lbl = lv_label_create(btn_prev);
    lv_label_set_text(prev_lbl, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(prev_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_font(prev_lbl, &lv_font_montserrat_16, 0);
    lv_obj_center(prev_lbl);

    // ">" button (right)
    btn_next = lv_button_create(nav_bar);
    lv_obj_set_size(btn_next, 70, 34);
    lv_obj_set_pos(btn_next, SCREEN_W - 75, 3);
    lv_obj_set_style_bg_color(btn_next, lv_color_make(60, 60, 80), 0);
    lv_obj_set_style_radius(btn_next, 6, 0);
    lv_obj_add_event_cb(btn_next, btn_next_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *next_lbl = lv_label_create(btn_next);
    lv_label_set_text(next_lbl, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(next_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_font(next_lbl, &lv_font_montserrat_16, 0);
    lv_obj_center(next_lbl);

    // Screen title (center)
    screen_title_label = lv_label_create(nav_bar);
    lv_label_set_text(screen_title_label, "GAUGES");
    lv_obj_set_style_text_color(screen_title_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(screen_title_label, &lv_font_montserrat_14, 0);
    lv_obj_align(screen_title_label, LV_ALIGN_CENTER, 0, 0);

    // Dots (below title)
    int dot_size = 8;
    int dot_gap = 16;
    int total_w = NUM_SCREENS * dot_gap - (dot_gap - dot_size);
    int start_x = (SCREEN_W - total_w) / 2;

    for (int i = 0; i < NUM_SCREENS; i++) {
        nav_dots[i] = lv_obj_create(nav_bar);
        lv_obj_set_size(nav_dots[i], dot_size, dot_size);
        lv_obj_set_pos(nav_dots[i], start_x + i * dot_gap, 30);
        lv_obj_set_style_radius(nav_dots[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(nav_dots[i], 0, 0);
        lv_obj_clear_flag(nav_dots[i], LV_OBJ_FLAG_SCROLLABLE);
    }
    update_nav_ui();
}

// ============================================================================
// Build all screens
// ============================================================================
static void build_dashboard(void) {
    if (lvgl_port_lock(100)) {
        lv_obj_t *scr = lv_screen_active();
        lv_obj_clean(scr);
        lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

        // Create screen containers (content area above nav bar)
        #define CONTENT_H (SCREEN_H - 40)

        screen_gauges = lv_obj_create(scr);
        lv_obj_set_size(screen_gauges, SCREEN_W, CONTENT_H);
        lv_obj_set_pos(screen_gauges, 0, 0);
        lv_obj_set_style_bg_opa(screen_gauges, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(screen_gauges, 0, 0);
        lv_obj_set_style_pad_all(screen_gauges, 0, 0);
        lv_obj_clear_flag(screen_gauges, LV_OBJ_FLAG_SCROLLABLE);

        screen_sensors = lv_obj_create(scr);
        lv_obj_set_size(screen_sensors, SCREEN_W, CONTENT_H);
        lv_obj_set_pos(screen_sensors, 0, 0);
        lv_obj_set_style_bg_opa(screen_sensors, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(screen_sensors, 0, 0);
        lv_obj_set_style_pad_all(screen_sensors, 0, 0);
        lv_obj_clear_flag(screen_sensors, LV_OBJ_FLAG_SCROLLABLE);

        screen_diag = lv_obj_create(scr);
        lv_obj_set_size(screen_diag, SCREEN_W, CONTENT_H);
        lv_obj_set_pos(screen_diag, 0, 0);
        lv_obj_set_style_bg_opa(screen_diag, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(screen_diag, 0, 0);
        lv_obj_set_style_pad_all(screen_diag, 0, 0);
        lv_obj_clear_flag(screen_diag, LV_OBJ_FLAG_SCROLLABLE);

        screen_debug = lv_obj_create(scr);
        lv_obj_set_size(screen_debug, SCREEN_W, CONTENT_H);
        lv_obj_set_pos(screen_debug, 0, 0);
        lv_obj_set_style_bg_opa(screen_debug, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(screen_debug, 0, 0);
        lv_obj_set_style_pad_all(screen_debug, 0, 0);
        lv_obj_clear_flag(screen_debug, LV_OBJ_FLAG_SCROLLABLE);

        screen_sniffer = lv_obj_create(scr);
        lv_obj_set_size(screen_sniffer, SCREEN_W, CONTENT_H);
        lv_obj_set_pos(screen_sniffer, 0, 0);
        lv_obj_set_style_bg_opa(screen_sniffer, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(screen_sniffer, 0, 0);
        lv_obj_set_style_pad_all(screen_sniffer, 0, 0);
        lv_obj_clear_flag(screen_sniffer, LV_OBJ_FLAG_SCROLLABLE);

        screens[0] = screen_gauges;
        screens[1] = screen_sensors;
        screens[2] = screen_diag;
        screens[3] = screen_debug;
        screens[4] = screen_sniffer;

        // Build each screen
        build_gauges_screen(screen_gauges);
        build_sensors_screen(screen_sensors);
        build_diag_screen(screen_diag);
        build_debug_screen(screen_debug);
        build_sniffer_screen(screen_sniffer);
        build_nav_bar(scr);

        // Show only first screen
        lv_obj_add_flag(screen_sensors, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(screen_diag, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(screen_debug, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(screen_sniffer, LV_OBJ_FLAG_HIDDEN);
        current_screen = 0;

        lvgl_port_unlock();
    }
}

// ============================================================================
// Dashboard update timer
// ============================================================================
static void dashboard_timer_cb(lv_timer_t *timer) {
    const mercedes_data_t *mb = mercedes_decode_get_data();

    // Screen 1: Gauges — from Mercedes broadcast
    if (rpm_value_label) {
        lv_label_set_text_fmt(rpm_value_label, "%u", mb->engine_rpm);
    }
    if (speed_value_label) {
        lv_label_set_text_fmt(speed_value_label, "%u", mb->vehicle_speed_kmh);
    }
    if (temp_value_label) {
        // Steering angle: raw is in 0.5 degree units, show as degrees
        int deg = mb->steering_angle / 2;
        int half = (mb->steering_angle & 1) ? 5 : 0;
        lv_label_set_text_fmt(temp_value_label, "%d.%d", deg, half);
    }
    if (fuel_value_label) {
        // Gear display
        const char *gear_names[] = {"P", "R", "N", "D", "4", "5", "6", "7", "8"};
        if (mb->gear < 9) {
            lv_label_set_text(fuel_value_label, gear_names[mb->gear]);
        } else {
            lv_label_set_text_fmt(fuel_value_label, "%u", mb->gear);
        }
    }
    // Status dot: green if decoding broadcast data, red if not
    if (status_dot) {
        bool has_data = mb->decode_count > 0;
        lv_obj_set_style_bg_color(status_dot,
            has_data ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_RED), 0);
    }

    // Screen 2: All Sensors — Mercedes broadcast data
    if (!lv_obj_has_flag(screen_sensors, LV_OBJ_FLAG_HIDDEN)) {
        if (sensor_labels[0]) lv_label_set_text_fmt(sensor_labels[0], "%u", mb->engine_rpm);
        if (sensor_labels[1]) lv_label_set_text_fmt(sensor_labels[1], "%u km/h", mb->vehicle_speed_kmh);
        if (sensor_labels[2]) {
            int deg = mb->steering_angle / 2;
            lv_label_set_text_fmt(sensor_labels[2], "%d deg", deg);
        }
        if (sensor_labels[3]) lv_label_set_text_fmt(sensor_labels[3], "%d", mb->steering_rate);
        if (sensor_labels[4]) lv_label_set_text_fmt(sensor_labels[4], "%u %%", mb->gas_pedal);
        if (sensor_labels[5]) lv_label_set_text_fmt(sensor_labels[5], "%s (%u)",
            mb->brake_pressed ? "ON" : "OFF", mb->brake_position);
        if (sensor_labels[6]) {
            const char *gear_names[] = {"P", "R", "N", "D", "4", "5", "6", "7", "8"};
            if (mb->gear < 9) {
                lv_label_set_text(sensor_labels[6], gear_names[mb->gear]);
            } else {
                lv_label_set_text_fmt(sensor_labels[6], "%u", mb->gear);
            }
        }
        if (sensor_labels[7]) {
            // Wheel speed FL in 0.0375 mph units -> km/h: raw * 603 / 10000
            uint32_t ws_fl = (uint32_t)mb->wheel_speed_fl * 603 / 10000;
            lv_label_set_text_fmt(sensor_labels[7], "%"PRIu32" km/h", ws_fl);
        }
        if (sensor_labels[8]) {
            uint32_t ws_fr = (uint32_t)mb->wheel_speed_fr * 603 / 10000;
            lv_label_set_text_fmt(sensor_labels[8], "%"PRIu32" km/h", ws_fr);
        }
        if (sensor_labels[9]) {
            lv_label_set_text_fmt(sensor_labels[9], "0x%02X", mb->doors_open);
        }
    }

    // Screen 3: Diagnostics — broadcast decode stats
    if (!lv_obj_has_flag(screen_diag, LV_OBJ_FLAG_HIDDEN)) {
        if (diag_status_label) {
            bool running = can_driver_is_running();
            if (running && mb->decode_count > 0) {
                lv_label_set_text(diag_status_label, "CAN: NO_ACK - Decoding OK");
                lv_obj_set_style_text_color(diag_status_label, lv_palette_main(LV_PALETTE_GREEN), 0);
            } else if (running) {
                lv_label_set_text(diag_status_label, "CAN: Listening - No data yet");
                lv_obj_set_style_text_color(diag_status_label, lv_palette_main(LV_PALETTE_YELLOW), 0);
            } else {
                lv_label_set_text(diag_status_label, "CAN: Not connected");
                lv_obj_set_style_text_color(diag_status_label, lv_palette_main(LV_PALETTE_RED), 0);
            }
        }
        if (diag_stats_label) {
            const sniffer_state_t *sniff = can_sniffer_get_state();
            lv_label_set_text_fmt(diag_stats_label,
                "Total msgs: %" PRIu32 "\n"
                "Unique IDs: %d\n"
                "Decoded:    %" PRIu32 "\n"
                "RPM: %u  Speed: %u",
                sniff->total_msgs, sniff->num_ids,
                mb->decode_count,
                mb->engine_rpm, mb->vehicle_speed_kmh);
        }
        if (diag_detail_label) {
            lv_label_set_text_fmt(diag_detail_label,
                "Mode: NO_ACK  500kbps  rx_q=32\nTX:%d  RX:%d",
                CAN_TX_GPIO, CAN_RX_GPIO);
        }
    }

    // Screen 4: CAN Debug (only update if visible)
    if (!lv_obj_has_flag(screen_debug, LV_OBJ_FLAG_HIDDEN)) {
        can_debug_info_t dbg;
        if (can_driver_get_debug_info(&dbg) == ESP_OK) {
            const char *state_names[] = {"STOPPED", "RUNNING", "BUS-OFF", "RECOVERING"};
            const char *state_str = dbg.state <= 3 ? state_names[dbg.state] : "UNKNOWN";

            if (debug_bus_label) {
                lv_obj_set_style_text_color(debug_bus_label,
                    dbg.state == 1 ? lv_palette_main(LV_PALETTE_GREEN) :
                    dbg.state == 2 ? lv_palette_main(LV_PALETTE_RED) :
                    lv_palette_main(LV_PALETTE_YELLOW), 0);
                lv_label_set_text_fmt(debug_bus_label,
                    "Bus: %s\nTX err cnt: %" PRIu32 "  RX err cnt: %" PRIu32,
                    state_str, dbg.tx_error_counter, dbg.rx_error_counter);
            }
            if (debug_counters_label) {
                lv_label_set_text_fmt(debug_counters_label,
                    "TX failed:  %" PRIu32 "\n"
                    "RX missed:  %" PRIu32 "\n"
                    "RX queued:  %" PRIu32,
                    dbg.tx_failed_count, dbg.rx_missed_count, dbg.msgs_to_rx);
            }
            if (debug_errors_label) {
                lv_label_set_text_fmt(debug_errors_label,
                    "Bus errors: %" PRIu32 "\n"
                    "Arb lost:   %" PRIu32 "\n"
                    "GPIO  TX:%d  RX:%d",
                    dbg.bus_error_count, dbg.arb_lost_count,
                    CAN_TX_GPIO, CAN_RX_GPIO);
            }
        }
    }

    // Screen 5: Sniffer (only update if visible)
    if (!lv_obj_has_flag(screen_sniffer, LV_OBJ_FLAG_HIDDEN)) {
        const sniffer_state_t *sniff = can_sniffer_get_state();

        if (sniffer_summary_label) {
            lv_label_set_text_fmt(sniffer_summary_label,
                "IDs: %d  Msgs: %" PRIu32 "  Showing: %d-%d",
                sniff->num_ids, sniff->total_msgs,
                sniffer_scroll_offset + 1,
                sniffer_scroll_offset + SNIFFER_DISPLAY_ROWS > sniff->num_ids
                    ? sniff->num_ids : sniffer_scroll_offset + SNIFFER_DISPLAY_ROWS);
        }

        for (int i = 0; i < SNIFFER_DISPLAY_ROWS; i++) {
            if (!sniffer_row_labels[i]) continue;
            int idx = sniffer_scroll_offset + i;
            if (idx < sniff->num_ids) {
                const sniffer_entry_t *e = &sniff->entries[idx];
                // Format: ID  count  hex data bytes
                char hex[25] = "";
                int pos = 0;
                for (int b = 0; b < e->last_dlc && b < 8; b++) {
                    pos += lv_snprintf(hex + pos, sizeof(hex) - pos, "%02X ", e->last_data[b]);
                }
                lv_label_set_text_fmt(sniffer_row_labels[i],
                    "0x%03" PRIX32 " %5" PRIu32 "  %s",
                    e->id, e->count, hex);
            } else {
                lv_label_set_text(sniffer_row_labels[i], "");
            }
        }
    }
}

// ============================================================================
// app_main
// ============================================================================
void app_main(void) {
    xTaskCreate(blink_task, "blink", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "Starting initialization...");

    // Backlight
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT
    };
    gpio_config(&bk_gpio_config);
    gpio_set_level(PIN_NUM_BK_LIGHT, 1);

    // SPI Bus
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK, .mosi_io_num = PIN_NUM_MOSI, .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 480 * 320 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // ST7796 Display
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

    // I2C for Touch
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
        .x_max = 320, .y_max = 480,
        .rst_gpio_num = 25, .int_gpio_num = -1,
        .levels = { .reset = 0, .interrupt = 0 },
        .flags = { .swap_xy = true, .mirror_x = false, .mirror_y = true },
    };

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_NUM_0, &tp_io_config, &tp_io_handle);
    esp_lcd_touch_new_i2c_gt911(tp_io_handle, &touch_config, &touch_handle);

    // LVGL Port
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = 480 * 20,
        .double_buffer = false,
        .hres = 480, .vres = 320,
        .monochrome = false,
        .rotation = { .swap_xy = true, .mirror_x = true, .mirror_y = true },
        .color_format = LV_COLOR_FORMAT_RGB565,
        .flags = { .buff_dma = true, .swap_bytes = true }
    };
    lv_display_t *disp_handle = lvgl_port_add_disp(&disp_cfg);

    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp_handle,
        .handle = touch_handle
    };
    lvgl_port_add_touch(&touch_cfg);

    // Always use CAN Manager (NO_ACK mode + OBD2 requests + sniffer + mercedes decode)
    ESP_LOGI(TAG, "Initializing CAN Manager...");
    if (can_manager_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CAN Manager");
    } else {
        if (can_manager_start() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start CAN Manager");
        } else {
            ESP_LOGI(TAG, "CAN Manager started (NO_ACK + decode)");
        }
    }

    // Build dashboard UI
    build_dashboard();

    // Update timer - 200ms (5 Hz is enough for display)
    lv_timer_create(dashboard_timer_cb, 200, NULL);
}
