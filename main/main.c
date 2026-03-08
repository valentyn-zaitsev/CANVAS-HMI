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
#define CONTENT_TOP 18
#define CONTENT_H (SCREEN_H - 40 - CONTENT_TOP)

// 5 screens: Params, Temps chart, Speeds chart, Dynamics chart, Suspension chart
#define NUM_SCREENS 5
static int current_screen = 0;
static lv_obj_t *screens[NUM_SCREENS];

// Pinned CAN status bar label
static lv_obj_t *status_bar_label = NULL;

// ============================================================================
// Screen 1: Parameters list
// ============================================================================
#define NUM_PARAMS 40

static lv_obj_t *param_value_labels[NUM_PARAMS] = {NULL};
static lv_obj_t *param_raw_labels[NUM_PARAMS] = {NULL};

static const char *param_names[NUM_PARAMS] = {
    "Engine RPM",       // 0
    "RPM (0x308)",      // 1
    "Speed",            // 2
    "Gas Pedal",        // 3
    "Steer Angle",      // 4
    "Brake",            // 5
    "Gear (FSC)",       // 6
    "Drive Prog",       // 7
    "Oil Temp",         // 8
    "Coolant Temp",     // 9
    "Trans Temp",       // 10
    "Ambient Temp",     // 11
    "Fuel L/h",         // 12
    "Tank Level",       // 13
    "Turbine RPM",      // 14
    "Lateral G",        // 15
    "Yaw Rate",         // 16
    "L Blinker",        // 17
    "R Blinker",        // 18
    "Highbeam",         // 19
    "Doors",            // 20
    "Seatbelt Drv",     // 21
    "Seatbelt Pass",    // 22
    "Handbrake",        // 23
    "ESP Lamp",         // 24
    "ABS Lamp",         // 25
    "MIL Lamp",         // 26
    "Oil Warning",      // 27
    "Oil Level",        // 28
    "WS FL",            // 29
    "WS FR",            // 30
    "WS RL",            // 31
    "WS RR",            // 32
    "AIRMATIC FL",      // 33
    "AIRMATIC FR",      // 34
    "AIRMATIC RL",      // 35
    "AIRMATIC RR",      // 36
    "Style Accel",      // 37
    "Style Brake",      // 38
    "Decoded",          // 39
};

// ============================================================================
// Chart common
// ============================================================================
#define CHART_POINTS 25

// Screen 2: Temperatures
static lv_obj_t *chart_temp = NULL;
static lv_chart_series_t *ser_oil = NULL;
static lv_chart_series_t *ser_coolant = NULL;
static lv_chart_series_t *ser_trans = NULL;
static lv_chart_series_t *ser_ambient = NULL;

static int32_t data_oil[CHART_POINTS] = {
    20, 28, 38, 50, 60, 68, 75, 80, 84, 87,
    90, 92, 93, 94, 95, 96, 95, 97, 96, 95,
    94, 96, 97, 95, 94
};
static int32_t data_coolant[CHART_POINTS] = {
    20, 32, 45, 58, 68, 75, 80, 83, 85, 86,
    87, 88, 87, 88, 89, 88, 87, 88, 89, 88,
    87, 88, 87, 88, 87
};
static int32_t data_trans[CHART_POINTS] = {
    18, 24, 32, 42, 52, 60, 66, 70, 74, 76,
    78, 80, 81, 82, 83, 82, 83, 84, 83, 82,
    83, 84, 83, 82, 81
};
static int32_t data_ambient[CHART_POINTS] = {
    15, 16, 17, 18, 19, 20, 21, 22, 23, 22,
    21, 20, 19, 18, 19, 20, 21, 22, 23, 24,
    23, 22, 21, 20, 19
};

// Screen 3: Speeds/RPM
static lv_obj_t *chart_speed = NULL;
static lv_chart_series_t *ser_rpm = NULL;
static lv_chart_series_t *ser_turbine = NULL;
static lv_chart_series_t *ser_veh_speed = NULL;

static int32_t data_rpm[CHART_POINTS] = {
    762, 780, 850, 1200, 2500, 3200, 2800, 1500, 900, 780,
    1100, 2200, 3500, 4000, 3800, 2000, 1200, 800, 762, 762,
    1000, 1800, 2500, 1500, 800
};
static int32_t data_turbine[CHART_POINTS] = {
    700, 720, 800, 1100, 2300, 3000, 2600, 1400, 850, 720,
    1000, 2000, 3200, 3700, 3500, 1800, 1100, 750, 700, 700,
    950, 1650, 2300, 1400, 750
};
static int32_t data_vspeed[CHART_POINTS] = {
    0, 0, 10, 30, 60, 90, 80, 50, 20, 0,
    15, 45, 80, 120, 110, 60, 30, 5, 0, 0,
    20, 50, 70, 40, 10
};

// Screen 4: Dynamics
static lv_obj_t *chart_dyn = NULL;
static lv_chart_series_t *ser_lat_g = NULL;
static lv_chart_series_t *ser_yaw = NULL;

// Lateral G ×100 (so 0.15g = 15), Yaw rate ×10 (so 5.0°/s = 50)
static int32_t data_lat_g[CHART_POINTS] = {
    0, 0, 2, 5, 12, 25, 18, -5, -15, -8,
    3, 10, 30, 45, 35, -10, -20, -5, 0, 0,
    8, 15, 22, 10, 0
};
static int32_t data_yaw[CHART_POINTS] = {
    0, 0, 5, 10, 20, 40, 30, -8, -25, -15,
    5, 15, 50, 70, 55, -15, -35, -10, 0, 0,
    12, 25, 35, 18, 0
};

// Screen 5: Suspension
static lv_obj_t *chart_susp = NULL;
static lv_chart_series_t *ser_lev_fl = NULL;
static lv_chart_series_t *ser_lev_fr = NULL;
static lv_chart_series_t *ser_lev_rl = NULL;
static lv_chart_series_t *ser_lev_rr = NULL;

static int32_t data_lev_fl[CHART_POINTS] = {
    128, 128, 127, 126, 125, 124, 123, 124, 125, 126,
    127, 128, 128, 127, 126, 125, 126, 127, 128, 128,
    127, 126, 127, 128, 128
};
static int32_t data_lev_fr[CHART_POINTS] = {
    128, 128, 127, 126, 124, 123, 122, 123, 124, 126,
    127, 128, 128, 127, 125, 124, 125, 127, 128, 128,
    127, 126, 127, 128, 128
};
static int32_t data_lev_rl[CHART_POINTS] = {
    130, 130, 129, 128, 127, 126, 125, 126, 127, 128,
    129, 130, 130, 129, 128, 127, 128, 129, 130, 130,
    129, 128, 129, 130, 130
};
static int32_t data_lev_rr[CHART_POINTS] = {
    130, 130, 129, 128, 126, 125, 124, 125, 126, 128,
    129, 130, 130, 129, 127, 126, 127, 129, 130, 130,
    129, 128, 129, 130, 130
};

// Navigation
static lv_obj_t *nav_dots[NUM_SCREENS] = {NULL};
static lv_obj_t *btn_prev = NULL;
static lv_obj_t *btn_next = NULL;
static lv_obj_t *screen_title_label = NULL;

// ============================================================================
// Blink task
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
static const char *screen_titles[] = {
    "PARAMETERS", "TEMPERATURES", "SPEED / RPM", "DYNAMICS", "SUSPENSION"
};

static void update_nav_ui(void) {
    for (int i = 0; i < NUM_SCREENS; i++) {
        if (nav_dots[i]) {
            lv_obj_set_style_bg_color(nav_dots[i], lv_color_white(), 0);
            lv_obj_set_style_bg_opa(nav_dots[i],
                i == current_screen ? LV_OPA_COVER : LV_OPA_30, 0);
        }
    }
    if (screen_title_label)
        lv_label_set_text(screen_title_label, screen_titles[current_screen]);
}

static void cross_hide_now(void);  // forward decl

static void switch_screen(int new_screen) {
    if (new_screen < 0) new_screen = NUM_SCREENS - 1;
    if (new_screen >= NUM_SCREENS) new_screen = 0;
    if (new_screen == current_screen) return;
    cross_hide_now();
    lv_obj_add_flag(screens[current_screen], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(screens[new_screen], LV_OBJ_FLAG_HIDDEN);
    current_screen = new_screen;
    update_nav_ui();
}

static void btn_prev_cb(lv_event_t *e) { switch_screen(current_screen - 1); }
static void btn_next_cb(lv_event_t *e) { switch_screen(current_screen + 1); }

// ============================================================================
// Screen 1: Parameters (scrollable, pinned status bar)
// ============================================================================
static void build_params_screen(lv_obj_t *parent) {
    lv_obj_set_style_pad_all(parent, 0, 0);
    lv_obj_add_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(parent, LV_DIR_VER);

    // Header labels directly on parent (no container)
    lv_obj_t *h1 = lv_label_create(parent);
    lv_label_set_text(h1, "Parameter");
    lv_obj_set_style_text_color(h1, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_text_font(h1, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(h1, 7, 2);

    lv_obj_t *h2 = lv_label_create(parent);
    lv_label_set_text(h2, "Value");
    lv_obj_set_style_text_color(h2, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_text_font(h2, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(h2, 150, 2);

    lv_obj_t *h3 = lv_label_create(parent);
    lv_label_set_text(h3, "Raw");
    lv_obj_set_style_text_color(h3, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_text_font(h3, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(h3, 365, 2);

    for (int i = 0; i < NUM_PARAMS; i++) {
        int y = 18 + i * 16;

        // Labels placed directly on parent — no row container
        lv_obj_t *name_lbl = lv_label_create(parent);
        lv_label_set_text(name_lbl, param_names[i]);
        lv_obj_set_style_text_color(name_lbl, lv_color_make(180, 180, 200), 0);
        lv_obj_set_style_text_font(name_lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_pos(name_lbl, 7, y);

        param_value_labels[i] = lv_label_create(parent);
        lv_label_set_text(param_value_labels[i], "---");
        lv_obj_set_style_text_color(param_value_labels[i], lv_color_white(), 0);
        lv_obj_set_style_text_font(param_value_labels[i], &lv_font_montserrat_12, 0);
        lv_obj_set_pos(param_value_labels[i], 150, y);

        param_raw_labels[i] = lv_label_create(parent);
        lv_label_set_text(param_raw_labels[i], "");
        lv_obj_set_style_text_color(param_raw_labels[i], lv_color_make(120, 120, 140), 0);
        lv_obj_set_style_text_font(param_raw_labels[i], &lv_font_montserrat_12, 0);
        lv_obj_set_pos(param_raw_labels[i], 365, y);
    }
}

// ============================================================================
// Generic chart builder with inline labels (touch disabled for debug)
// ============================================================================
typedef struct {
    const char *name;
    lv_color_t color;
    int32_t *data;
    lv_chart_series_t **series_ptr;
} chart_series_cfg_t;

// Chart layout constants
#define CHART_PAD 5
#define CHART_Y 0
#define CHART_H 244

// Per-chart info for shared crosshair
typedef struct {
    int32_t *data[8];
    lv_color_t colors[8];
    int num_series;
    int y_min, y_max;
    const char **x_labels;
    int x_count;
    int chart_x, chart_w;
} chart_info_t;

static chart_info_t chart_infos[4];
static int chart_info_count = 0;

// Shared crosshair objects (created on scr, only 4 objects total)
static lv_obj_t *cross_h = NULL;
static lv_obj_t *cross_v = NULL;
static lv_obj_t *cross_xl = NULL;
static lv_obj_t *cross_yl = NULL;
static lv_timer_t *cross_timer = NULL;

static void cross_hide_cb(lv_timer_t *t) {
    if (cross_h) lv_obj_add_flag(cross_h, LV_OBJ_FLAG_HIDDEN);
    if (cross_v) lv_obj_add_flag(cross_v, LV_OBJ_FLAG_HIDDEN);
    if (cross_xl) lv_obj_add_flag(cross_xl, LV_OBJ_FLAG_HIDDEN);
    if (cross_yl) lv_obj_add_flag(cross_yl, LV_OBJ_FLAG_HIDDEN);
    cross_timer = NULL;
    lv_timer_del(t);
}

static void cross_hide_now(void) {
    if (cross_h) lv_obj_add_flag(cross_h, LV_OBJ_FLAG_HIDDEN);
    if (cross_v) lv_obj_add_flag(cross_v, LV_OBJ_FLAG_HIDDEN);
    if (cross_xl) lv_obj_add_flag(cross_xl, LV_OBJ_FLAG_HIDDEN);
    if (cross_yl) lv_obj_add_flag(cross_yl, LV_OBJ_FLAG_HIDDEN);
    if (cross_timer) { lv_timer_del(cross_timer); cross_timer = NULL; }
}

static void chart_screen_touch_cb(lv_event_t *e) {
    if (current_screen < 1 || current_screen > 4) return;
    chart_info_t *info = &chart_infos[current_screen - 1];
    if (info->num_series == 0) return;
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_PRESSING) {
        if (cross_timer) { lv_timer_del(cross_timer); cross_timer = NULL; }
        lv_indev_t *indev = lv_indev_active();
        if (!indev) return;
        lv_point_t tp;
        lv_indev_get_point(indev, &tp);

        int cx = info->chart_x + CHART_PAD;
        int cy = CONTENT_TOP + CHART_Y + CHART_PAD;
        int cw = info->chart_w - 2 * CHART_PAD;
        int ch = CHART_H - 2 * CHART_PAD;

        if (tp.x < cx || tp.x > cx + cw || tp.y < cy || tp.y > cy + ch) return;

        int rx = tp.x - cx;
        int pt = (rx * (CHART_POINTS - 1) + cw / 2) / cw;
        if (pt < 0) pt = 0;
        if (pt >= CHART_POINTS) pt = CHART_POINTS - 1;

        int ry = tp.y - cy;
        int best = 0, bdist = 999999;
        for (int s = 0; s < info->num_series; s++) {
            int vy = ch - ((info->data[s][pt] - info->y_min) * ch / (info->y_max - info->y_min));
            int d = abs(ry - vy);
            if (d < bdist) { bdist = d; best = s; }
        }

        int snap_x = cx + (pt * cw / (CHART_POINTS - 1));
        int32_t val = info->data[best][pt];
        int snap_y = cy + ch - ((val - info->y_min) * ch / (info->y_max - info->y_min));

        lv_color_t cc = info->colors[best];

        lv_obj_set_pos(cross_h, cx, snap_y);
        lv_obj_set_size(cross_h, cw, 1);
        lv_obj_set_style_bg_color(cross_h, cc, 0);
        lv_obj_clear_flag(cross_h, LV_OBJ_FLAG_HIDDEN);

        lv_obj_set_pos(cross_v, snap_x, cy);
        lv_obj_set_size(cross_v, 1, ch);
        lv_obj_set_style_bg_color(cross_v, cc, 0);
        lv_obj_clear_flag(cross_v, LV_OBJ_FLAG_HIDDEN);

        char vbuf[16];
        lv_snprintf(vbuf, sizeof(vbuf), "%d", (int)val);
        lv_label_set_text(cross_yl, vbuf);
        lv_obj_set_style_text_color(cross_yl, cc, 0);
        int ly = snap_y - 6;
        if (ly < CONTENT_TOP) ly = CONTENT_TOP;
        lv_obj_set_pos(cross_yl, 0, ly);
        lv_obj_clear_flag(cross_yl, LV_OBJ_FLAG_HIDDEN);

        // Interpolate time between x labels for exact point
        char xbuf[8];
        {
            int sh = 0, sm = 0, eh = 0, em = 0;
            sscanf(info->x_labels[0], "%d:%d", &sh, &sm);
            sscanf(info->x_labels[info->x_count - 1], "%d:%d", &eh, &em);
            int t0 = sh * 60 + sm, t1 = eh * 60 + em;
            int t = t0 + pt * (t1 - t0) / (CHART_POINTS - 1);
            lv_snprintf(xbuf, sizeof(xbuf), "%d:%02d", t / 60, t % 60);
        }
        lv_label_set_text(cross_xl, xbuf);
        lv_obj_set_style_text_color(cross_xl, cc, 0);
        int lx = snap_x - 15;
        if (lx < info->chart_x) lx = info->chart_x;
        if (lx > info->chart_x + info->chart_w - 40) lx = info->chart_x + info->chart_w - 40;
        lv_obj_set_pos(cross_xl, lx, CONTENT_TOP + CHART_H + 2);
        lv_obj_clear_flag(cross_xl, LV_OBJ_FLAG_HIDDEN);

    } else if (code == LV_EVENT_RELEASED) {
        if (cross_timer) lv_timer_del(cross_timer);
        cross_timer = lv_timer_create(cross_hide_cb, 2000, NULL);
        lv_timer_set_repeat_count(cross_timer, 1);
    }
}

static lv_obj_t *build_chart_generic(
    lv_obj_t *parent,
    const char *title_text,
    int y_min, int y_max,
    const char *y_unit,
    const char **x_labels, int x_count,
    chart_series_cfg_t *series_cfg, int num_series)
{
    (void)title_text;
    (void)y_unit;
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);

    // Dynamic Y-axis width based on label content
    int max_chars = 0;
    for (int i = 0; i < 5; i++) {
        int val = y_min + i * (y_max - y_min) / 4;
        int nc = (val < 0) ? 1 : 0;
        int av = abs(val);
        do { nc++; av /= 10; } while (av > 0);
        if (nc > max_chars) max_chars = nc;
    }
    int chart_x = max_chars * 8 + 6;
    if (chart_x < 22) chart_x = 22;
    int chart_w = SCREEN_W - chart_x - 2;

    lv_obj_t *chart = lv_chart_create(parent);
    lv_obj_set_size(chart, chart_w, CHART_H);
    lv_obj_set_pos(chart, chart_x, CHART_Y);
    lv_obj_set_style_bg_color(chart, lv_color_make(15, 15, 25), 0);
    lv_obj_set_style_border_color(chart, lv_color_make(60, 60, 80), 0);
    lv_obj_set_style_border_width(chart, 1, 0);
    lv_obj_set_style_radius(chart, 4, 0);
    lv_obj_set_style_line_color(chart, lv_color_make(40, 40, 55), 0);
    lv_obj_set_style_pad_all(chart, CHART_PAD, 0);
    lv_obj_clear_flag(chart, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart, CHART_POINTS);
    lv_chart_set_div_line_count(chart, 5, x_count);
    lv_chart_set_axis_range(chart, LV_CHART_AXIS_PRIMARY_Y, y_min, y_max);

    lv_obj_set_style_line_width(chart, 2, LV_PART_ITEMS);
    lv_obj_set_style_size(chart, 4, 4, LV_PART_INDICATOR);

    int content_h = CHART_H - 2 * CHART_PAD;

    // Compute label y-positions based on last data values, then de-overlap
    int label_y[8];
    for (int i = 0; i < num_series; i++) {
        int32_t last_val = series_cfg[i].data[CHART_POINTS - 1];
        label_y[i] = CHART_PAD + content_h - ((last_val - y_min) * content_h / (y_max - y_min)) - 14;
        if (label_y[i] < 0) label_y[i] = 0;
        if (label_y[i] > CHART_H - 14) label_y[i] = CHART_H - 14;
    }
    int order[8];
    for (int i = 0; i < num_series; i++) order[i] = i;
    for (int i = 0; i < num_series - 1; i++)
        for (int j = i + 1; j < num_series; j++)
            if (label_y[order[i]] > label_y[order[j]]) {
                int tmp = order[i]; order[i] = order[j]; order[j] = tmp;
            }
    for (int i = 1; i < num_series; i++) {
        if (label_y[order[i]] - label_y[order[i-1]] < 14)
            label_y[order[i]] = label_y[order[i-1]] + 14;
    }

    // Add series and inline labels on right side of chart
    for (int i = 0; i < num_series; i++) {
        *series_cfg[i].series_ptr = lv_chart_add_series(chart, series_cfg[i].color, LV_CHART_AXIS_PRIMARY_Y);
        lv_chart_set_series_ext_y_array(chart, *series_cfg[i].series_ptr, series_cfg[i].data);

        lv_obj_t *lbl = lv_label_create(parent);
        lv_label_set_text(lbl, series_cfg[i].name);
        lv_obj_set_style_text_color(lbl, series_cfg[i].color, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_bg_color(lbl, lv_color_make(15, 15, 25), 0);
        lv_obj_set_style_bg_opa(lbl, LV_OPA_70, 0);
        lv_obj_set_pos(lbl, chart_x + chart_w - CHART_PAD - 55, CHART_Y + label_y[i]);
    }

    // Y axis labels (5 evenly distributed, right-aligned)
    {
        char vbuf[16];
        for (int i = 0; i < 5; i++) {
            int val = y_min + i * (y_max - y_min) / 4;
            lv_snprintf(vbuf, sizeof(vbuf), "%d", val);
            lv_obj_t *yl = lv_label_create(parent);
            lv_label_set_text(yl, vbuf);
            lv_obj_set_style_text_color(yl, lv_color_make(100, 100, 120), 0);
            lv_obj_set_style_text_font(yl, &lv_font_montserrat_12, 0);
            lv_obj_set_width(yl, chart_x - 2);
            lv_obj_set_style_text_align(yl, LV_TEXT_ALIGN_RIGHT, 0);
            int y_pix = CHART_Y + CHART_PAD + content_h - (i * content_h / 4) - 6;
            if (y_pix < 0) y_pix = 0;
            lv_obj_set_pos(yl, 0, y_pix);
        }
    }

    // X axis labels
    {
        int content_w = chart_w - 2 * CHART_PAD;
        for (int i = 0; i < x_count; i++) {
            lv_obj_t *lbl = lv_label_create(parent);
            lv_label_set_text(lbl, x_labels[i]);
            lv_obj_set_style_text_color(lbl, lv_color_make(100, 100, 120), 0);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
            int xp = chart_x + CHART_PAD + (i * content_w / (x_count - 1)) - 15;
            if (xp < 0) xp = 0;
            if (xp > SCREEN_W - 35) xp = SCREEN_W - 35;
            lv_obj_set_pos(lbl, xp, CHART_Y + CHART_H + 2);
        }
    }

    lv_chart_refresh(chart);

    // Store chart info for crosshair
    if (chart_info_count < 4) {
        chart_info_t *ci = &chart_infos[chart_info_count++];
        ci->y_min = y_min; ci->y_max = y_max;
        ci->x_labels = x_labels; ci->x_count = x_count;
        ci->num_series = num_series;
        ci->chart_x = chart_x; ci->chart_w = chart_w;
        for (int i = 0; i < num_series && i < 8; i++) {
            ci->data[i] = series_cfg[i].data;
            ci->colors[i] = series_cfg[i].color;
        }
    }

    return chart;
}

// ============================================================================
// Navigation bar
// ============================================================================
static void build_nav_bar(lv_obj_t *scr) {
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

    btn_prev = lv_button_create(nav_bar);
    lv_obj_set_size(btn_prev, 60, 34);
    lv_obj_set_pos(btn_prev, 5, 3);
    lv_obj_set_style_bg_color(btn_prev, lv_color_make(60, 60, 80), 0);
    lv_obj_set_style_radius(btn_prev, 6, 0);
    lv_obj_add_event_cb(btn_prev, btn_prev_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *pl = lv_label_create(btn_prev);
    lv_label_set_text(pl, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(pl, lv_color_white(), 0);
    lv_obj_center(pl);

    btn_next = lv_button_create(nav_bar);
    lv_obj_set_size(btn_next, 60, 34);
    lv_obj_set_pos(btn_next, SCREEN_W - 65, 3);
    lv_obj_set_style_bg_color(btn_next, lv_color_make(60, 60, 80), 0);
    lv_obj_set_style_radius(btn_next, 6, 0);
    lv_obj_add_event_cb(btn_next, btn_next_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *nl = lv_label_create(btn_next);
    lv_label_set_text(nl, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(nl, lv_color_white(), 0);
    lv_obj_center(nl);

    screen_title_label = lv_label_create(nav_bar);
    lv_label_set_text(screen_title_label, screen_titles[0]);
    lv_obj_set_style_text_color(screen_title_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(screen_title_label, &lv_font_montserrat_14, 0);
    lv_obj_align(screen_title_label, LV_ALIGN_CENTER, 0, -5);

    int dot_gap = 14;
    int total_w = NUM_SCREENS * dot_gap - (dot_gap - 6);
    int start_x = (SCREEN_W - total_w) / 2;
    for (int i = 0; i < NUM_SCREENS; i++) {
        nav_dots[i] = lv_obj_create(nav_bar);
        lv_obj_set_size(nav_dots[i], 6, 6);
        lv_obj_set_pos(nav_dots[i], start_x + i * dot_gap, 32);
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
    static const char *x_times[] = {"12:00", "12:30", "13:00", "13:30", "14:00", "14:30"};

    // Phase 1: Create base layout + status bar + containers
    if (lvgl_port_lock(100)) {
        lv_obj_t *scr = lv_screen_active();
        lv_obj_clean(scr);
        lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

        lv_obj_t *sbar = lv_obj_create(scr);
        lv_obj_set_size(sbar, SCREEN_W, 18);
        lv_obj_set_pos(sbar, 0, 0);
        lv_obj_set_style_bg_color(sbar, lv_color_make(15, 30, 15), 0);
        lv_obj_set_style_border_width(sbar, 0, 0);
        lv_obj_set_style_radius(sbar, 0, 0);
        lv_obj_set_style_pad_all(sbar, 1, 0);
        lv_obj_clear_flag(sbar, LV_OBJ_FLAG_SCROLLABLE);

        status_bar_label = lv_label_create(sbar);
        lv_label_set_text(status_bar_label, "CAN: Initializing...");
        lv_obj_set_style_text_color(status_bar_label, lv_palette_main(LV_PALETTE_YELLOW), 0);
        lv_obj_set_style_text_font(status_bar_label, &lv_font_montserrat_12, 0);
        lv_obj_set_pos(status_bar_label, 5, 0);

        for (int i = 0; i < NUM_SCREENS; i++) {
            screens[i] = lv_obj_create(scr);
            lv_obj_set_size(screens[i], SCREEN_W, CONTENT_H);
            lv_obj_set_pos(screens[i], 0, CONTENT_TOP);
            lv_obj_set_style_bg_opa(screens[i], LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(screens[i], 0, 0);
            lv_obj_set_style_pad_all(screens[i], 0, 0);
            if (i > 0) lv_obj_add_flag(screens[i], LV_OBJ_FLAG_HIDDEN);
        }

        build_nav_bar(scr);
        current_screen = 0;
        lvgl_port_unlock();
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Phase 2: Parameters screen (many objects)
    if (lvgl_port_lock(100)) {
        build_params_screen(screens[0]);
        lvgl_port_unlock();
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Phase 3: Temperatures chart
    if (lvgl_port_lock(100)) {
        chart_series_cfg_t temp_series[] = {
            {"Oil",     lv_palette_main(LV_PALETTE_RED),    data_oil,     &ser_oil},
            {"Coolant", lv_palette_main(LV_PALETTE_GREEN),  data_coolant, &ser_coolant},
            {"Trans",   lv_palette_main(LV_PALETTE_ORANGE), data_trans,   &ser_trans},
            {"Ambient", lv_palette_main(LV_PALETTE_BLUE),   data_ambient, &ser_ambient},
        };
        chart_temp = build_chart_generic(screens[1],
            "Temperatures (emulated)", -10, 120, "C",
            x_times, 6, temp_series, 4);
        lvgl_port_unlock();
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Phase 4: Speed/RPM chart
    if (lvgl_port_lock(100)) {
        chart_series_cfg_t speed_series[] = {
            {"RPM",     lv_palette_main(LV_PALETTE_RED),   data_rpm,     &ser_rpm},
            {"Turbine", lv_palette_main(LV_PALETTE_AMBER), data_turbine, &ser_turbine},
            {"km/h",    lv_palette_main(LV_PALETTE_GREEN), data_vspeed,  &ser_veh_speed},
        };
        chart_speed = build_chart_generic(screens[2],
            "Speed / RPM (emulated)", 0, 4500, "RPM",
            x_times, 6, speed_series, 3);
        lv_chart_set_axis_range(chart_speed, LV_CHART_AXIS_SECONDARY_Y, 0, 200);
        lvgl_port_unlock();
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Phase 5: Dynamics chart
    if (lvgl_port_lock(100)) {
        chart_series_cfg_t dyn_series[] = {
            {"Lat G",  lv_palette_main(LV_PALETTE_RED),   data_lat_g, &ser_lat_g},
            {"Yaw",    lv_palette_main(LV_PALETTE_CYAN),  data_yaw,   &ser_yaw},
        };
        chart_dyn = build_chart_generic(screens[3],
            "Dynamics (emulated)", -80, 80, "",
            x_times, 6, dyn_series, 2);
        lvgl_port_unlock();
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Phase 6: Suspension chart
    if (lvgl_port_lock(100)) {
        chart_series_cfg_t susp_series[] = {
            {"FL", lv_palette_main(LV_PALETTE_RED),    data_lev_fl, &ser_lev_fl},
            {"FR", lv_palette_main(LV_PALETTE_GREEN),  data_lev_fr, &ser_lev_fr},
            {"RL", lv_palette_main(LV_PALETTE_BLUE),   data_lev_rl, &ser_lev_rl},
            {"RR", lv_palette_main(LV_PALETTE_ORANGE), data_lev_rr, &ser_lev_rr},
        };
        chart_susp = build_chart_generic(screens[4],
            "AIRMATIC Levels (emulated)", 110, 145, "",
            x_times, 6, susp_series, 4);
        lvgl_port_unlock();
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Phase 7: Crosshair objects on scr + touch handlers on chart screens
    if (lvgl_port_lock(100)) {
        lv_obj_t *scr = lv_screen_active();

        cross_h = lv_obj_create(scr);
        lv_obj_set_style_bg_color(cross_h, lv_color_make(100, 180, 255), 0);
        lv_obj_set_style_bg_opa(cross_h, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(cross_h, 0, 0);
        lv_obj_set_style_radius(cross_h, 0, 0);
        lv_obj_set_style_pad_all(cross_h, 0, 0);
        lv_obj_clear_flag(cross_h, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(cross_h, LV_OBJ_FLAG_HIDDEN);

        cross_v = lv_obj_create(scr);
        lv_obj_set_style_bg_color(cross_v, lv_color_make(100, 180, 255), 0);
        lv_obj_set_style_bg_opa(cross_v, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(cross_v, 0, 0);
        lv_obj_set_style_radius(cross_v, 0, 0);
        lv_obj_set_style_pad_all(cross_v, 0, 0);
        lv_obj_clear_flag(cross_v, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(cross_v, LV_OBJ_FLAG_HIDDEN);

        cross_xl = lv_label_create(scr);
        lv_obj_set_style_text_color(cross_xl, lv_color_make(100, 180, 255), 0);
        lv_obj_set_style_text_font(cross_xl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_bg_color(cross_xl, lv_color_make(20, 20, 40), 0);
        lv_obj_set_style_bg_opa(cross_xl, LV_OPA_80, 0);
        lv_obj_add_flag(cross_xl, LV_OBJ_FLAG_HIDDEN);

        cross_yl = lv_label_create(scr);
        lv_obj_set_style_text_color(cross_yl, lv_color_make(100, 180, 255), 0);
        lv_obj_set_style_text_font(cross_yl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_bg_color(cross_yl, lv_color_make(20, 20, 40), 0);
        lv_obj_set_style_bg_opa(cross_yl, LV_OPA_80, 0);
        lv_obj_add_flag(cross_yl, LV_OBJ_FLAG_HIDDEN);

        // Touch handlers on chart screens (screens already clickable by default)
        for (int i = 1; i < NUM_SCREENS; i++) {
            lv_obj_add_event_cb(screens[i], chart_screen_touch_cb, LV_EVENT_PRESSING, NULL);
            lv_obj_add_event_cb(screens[i], chart_screen_touch_cb, LV_EVENT_RELEASED, NULL);
        }

        lvgl_port_unlock();
    }
}

// ============================================================================
// Dashboard update timer
// ============================================================================
static void dashboard_timer_cb(lv_timer_t *timer) {
    const mercedes_data_t *mb = mercedes_decode_get_data();
    char buf[48];

    // Always update status bar
    if (status_bar_label) {
        bool running = can_driver_is_running();
        const sniffer_state_t *sniff = can_sniffer_get_state();
        if (running && mb->decode_count > 0) {
            lv_snprintf(buf, sizeof(buf), "CAN OK | IDs:%d | Dec:%"PRIu32,
                sniff->num_ids, mb->decode_count);
            lv_label_set_text(status_bar_label, buf);
            lv_obj_set_style_text_color(status_bar_label, lv_palette_main(LV_PALETTE_GREEN), 0);
            lv_obj_set_style_bg_color(lv_obj_get_parent(status_bar_label), lv_color_make(10, 30, 10), 0);
        } else if (running) {
            lv_label_set_text(status_bar_label, "CAN: Waiting for data...");
            lv_obj_set_style_text_color(status_bar_label, lv_palette_main(LV_PALETTE_YELLOW), 0);
            lv_obj_set_style_bg_color(lv_obj_get_parent(status_bar_label), lv_color_make(30, 30, 10), 0);
        } else {
            lv_label_set_text(status_bar_label, "CAN: Not running");
            lv_obj_set_style_text_color(status_bar_label, lv_palette_main(LV_PALETTE_RED), 0);
            lv_obj_set_style_bg_color(lv_obj_get_parent(status_bar_label), lv_color_make(30, 10, 10), 0);
        }
    }

    // Only update params when visible
    if (!lv_obj_has_flag(screens[0], LV_OBJ_FLAG_HIDDEN)) {
        #define SET_VAL(idx, fmt, ...) if(param_value_labels[idx]) lv_label_set_text_fmt(param_value_labels[idx], fmt, ##__VA_ARGS__)
        #define SET_RAW(idx, fmt, ...) if(param_raw_labels[idx]) lv_label_set_text_fmt(param_raw_labels[idx], fmt, ##__VA_ARGS__)
        #define SET_VAL_S(idx, s) if(param_value_labels[idx]) lv_label_set_text(param_value_labels[idx], s)

        // 0: Engine RPM (0x105)
        SET_VAL(0, "%u", mb->engine_rpm);
        SET_RAW(0, "%u", mb->engine_rpm);

        // 1: RPM from 0x308 (×0.25)
        {
            uint32_t rpm308 = (uint32_t)mb->nmot_rpm_raw / 4;
            SET_VAL(1, "%"PRIu32, rpm308);
            SET_RAW(1, "%u", mb->nmot_rpm_raw);
        }

        // 2: Speed (from RDU wheel speeds, 0.01 km/h)
        {
            uint32_t avg = ((uint32_t)mb->ws_fl_rdu + mb->ws_fr_rdu + mb->ws_rl_rdu + mb->ws_rr_rdu) / 4;
            uint32_t kmh = avg / 100;
            uint32_t frac = (avg % 100) / 10;
            lv_snprintf(buf, sizeof(buf), "%"PRIu32".%"PRIu32" km/h", kmh, frac);
            SET_VAL_S(2, buf);
            SET_RAW(2, "%u", mb->vehicle_speed_kmh);
        }

        // 3: Gas Pedal
        SET_VAL(3, "%u%%", mb->gas_pedal);
        SET_RAW(3, "%u", mb->gas_pedal);

        // 4: Steer Angle
        {
            int raw = mb->steering_angle;
            int deg = raw / 2;
            int frac = abs(raw % 2) * 5;
            if (raw < 0 && deg == 0)
                lv_snprintf(buf, sizeof(buf), "-%d.%d", 0, frac);
            else
                lv_snprintf(buf, sizeof(buf), "%d.%d", deg, frac);
            SET_VAL_S(4, buf);
            SET_RAW(4, "%d", mb->steering_angle);
        }

        // 5: Brake
        SET_VAL_S(5, mb->brake_pressed ? "ON" : "OFF");
        if (mb->brake_pressed)
            lv_obj_set_style_text_color(param_value_labels[5], lv_palette_main(LV_PALETTE_RED), 0);
        else
            lv_obj_set_style_text_color(param_value_labels[5], lv_color_white(), 0);
        SET_RAW(5, "%u/%u", mb->brake_pressed, mb->brake_position);

        // 6: Gear (FSC from 0x418)
        {
            const char *g = "?";
            switch (mb->gear_fsc) {
                case 0: g = "P"; break;
                case 1: g = "R"; break;
                case 2: g = "N"; break;
                case 3: g = "D"; break;
                case 4: g = "4"; break;
                case 5: g = "3"; break;
                case 6: g = "2"; break;
                case 7: g = "1"; break;
            }
            SET_VAL_S(6, g);
            SET_RAW(6, "%u/%u", mb->gear_fsc, mb->gear);
        }

        // 7: Drive Program
        SET_VAL(7, "%u", mb->drive_program);
        SET_RAW(7, "%u", mb->drive_program);

        // 8: Oil Temp
        SET_VAL(8, "%d C", mb->oil_temp_c);
        SET_RAW(8, "%d", mb->oil_temp_c + 40);

        // 9: Coolant Temp
        SET_VAL(9, "%d C", mb->coolant_temp_c);
        SET_RAW(9, "%d", mb->coolant_temp_c + 40);

        // 10: Trans Temp
        SET_VAL(10, "%d C", mb->trans_oil_temp_c);
        SET_RAW(10, "%d", mb->trans_oil_temp_c + 40);

        // 11: Ambient Temp (raw × 0.5 - 40)
        {
            int temp_x2 = mb->ambient_temp_raw - 80;  // (raw*0.5 - 40)*2
            int deg = temp_x2 / 2;
            int frac = abs(temp_x2 % 2) * 5;
            if (temp_x2 < 0 && deg == 0)
                lv_snprintf(buf, sizeof(buf), "-%d.%d C", 0, frac);
            else
                lv_snprintf(buf, sizeof(buf), "%d.%d C", deg, frac);
            SET_VAL_S(11, buf);
            SET_RAW(11, "%u", mb->ambient_temp_raw);
        }

        // 12: Fuel L/h (raw µl/250ms → L/h: raw * 0.0144 ≈ raw * 144 / 10000)
        {
            uint32_t lph_x100 = (uint32_t)mb->fuel_consumption * 144 / 100;
            lv_snprintf(buf, sizeof(buf), "%"PRIu32".%02"PRIu32, lph_x100 / 100, lph_x100 % 100);
            SET_VAL_S(12, buf);
            SET_RAW(12, "%u", mb->fuel_consumption);
        }

        // 13: Tank Level
        SET_VAL(13, "%u L", mb->tank_level);
        SET_RAW(13, "%u", mb->tank_level);

        // 14: Turbine RPM (raw × 0.25)
        {
            uint32_t trpm = (uint32_t)mb->turbine_speed_raw / 4;
            SET_VAL(14, "%"PRIu32, trpm);
            SET_RAW(14, "%u", mb->turbine_speed_raw);
        }

        // 15: Lateral G (raw × 0.01)
        {
            int g_val = mb->lateral_g_raw;
            int whole = g_val / 100;
            int frac = abs(g_val) % 100;
            lv_snprintf(buf, sizeof(buf), "%d.%02d g", whole, frac);
            SET_VAL_S(15, buf);
            SET_RAW(15, "%d", mb->lateral_g_raw);
        }

        // 16: Yaw Rate (raw × 0.005)
        {
            int yr = mb->yaw_rate_raw;
            int whole = (yr * 5) / 1000;
            int frac = abs((yr * 5) % 1000) / 10;
            lv_snprintf(buf, sizeof(buf), "%d.%02d d/s", whole, frac);
            SET_VAL_S(16, buf);
            SET_RAW(16, "%d", mb->yaw_rate_raw);
        }

        // 17-18: Blinkers
        SET_VAL_S(17, mb->left_blinker ? "ON" : "OFF");
        SET_RAW(17, "%u", mb->left_blinker);
        SET_VAL_S(18, mb->right_blinker ? "ON" : "OFF");
        SET_RAW(18, "%u", mb->right_blinker);

        // 19: Highbeam
        SET_VAL_S(19, (mb->highbeam_toggle || mb->highbeam_momentary) ? "ON" : "OFF");
        lv_snprintf(buf, sizeof(buf), "T%u M%u", mb->highbeam_toggle, mb->highbeam_momentary);
        if (param_raw_labels[19]) lv_label_set_text(param_raw_labels[19], buf);

        // 20: Doors
        if (mb->door_open_fl || mb->door_open_fr || mb->door_open_rl || mb->door_open_rr) {
            lv_snprintf(buf, sizeof(buf), "%s%s%s%s",
                mb->door_open_fl ? "FL " : "", mb->door_open_fr ? "FR " : "",
                mb->door_open_rl ? "RL " : "", mb->door_open_rr ? "RR " : "");
            SET_VAL_S(20, buf);
        } else {
            SET_VAL_S(20, "Closed");
        }
        SET_RAW(20, "0x%02X", mb->doors_open);

        // 21-22: Seatbelts
        SET_VAL_S(21, mb->seatbelt_driver ? "Yes" : "No");
        SET_RAW(21, "%u", mb->seatbelt_driver);
        SET_VAL_S(22, mb->seatbelt_passenger ? "Yes" : "No");
        SET_RAW(22, "%u", mb->seatbelt_passenger);

        // 23: Handbrake
        SET_VAL_S(23, mb->handbrake ? "ON" : "OFF");
        SET_RAW(23, "%u", mb->handbrake);

        // 24-26: Warning lamps
        SET_VAL_S(24, mb->esp_lamp ? "ON" : "OFF");
        SET_RAW(24, "%u", mb->esp_lamp);
        SET_VAL_S(25, mb->abs_lamp ? "ON" : "OFF");
        SET_RAW(25, "%u", mb->abs_lamp);
        SET_VAL_S(26, mb->mil_lamp ? "ON" : "OFF");
        SET_RAW(26, "%u", mb->mil_lamp);

        // 27: Oil warning
        SET_VAL_S(27, mb->oil_warning ? "WARN" : "OK");
        SET_RAW(27, "%u", mb->oil_warning);

        // 28: Oil Level
        SET_VAL(28, "%u", mb->oil_level);
        SET_RAW(28, "%u", mb->oil_level);

        // 29-32: Wheel speeds (RDU, ×0.01 km/h)
        for (int w = 0; w < 4; w++) {
            uint16_t raw_ws;
            switch (w) {
                case 0: raw_ws = mb->ws_fl_rdu; break;
                case 1: raw_ws = mb->ws_fr_rdu; break;
                case 2: raw_ws = mb->ws_rl_rdu; break;
                default: raw_ws = mb->ws_rr_rdu; break;
            }
            uint32_t kmh = raw_ws / 100;
            uint32_t frac = (raw_ws % 100) / 10;
            lv_snprintf(buf, sizeof(buf), "%"PRIu32".%"PRIu32" km/h", kmh, frac);
            if (param_value_labels[29 + w]) lv_label_set_text(param_value_labels[29 + w], buf);
            SET_RAW(29 + w, "%u", raw_ws);
        }

        // 33-36: AIRMATIC levels
        SET_VAL(33, "%u", mb->level_fl);
        SET_RAW(33, "%u", mb->level_fl);
        SET_VAL(34, "%u", mb->level_fr);
        SET_RAW(34, "%u", mb->level_fr);
        SET_VAL(35, "%u", mb->level_rl);
        SET_RAW(35, "%u", mb->level_rl);
        SET_VAL(36, "%u", mb->level_rr);
        SET_RAW(36, "%u", mb->level_rr);

        // 37-38: Driving style
        SET_VAL(37, "%u", mb->style_accel);
        SET_RAW(37, "%u", mb->style_accel);
        SET_VAL(38, "%u", mb->style_braking);
        SET_RAW(38, "%u", mb->style_braking);

        // 39: Decoded count
        SET_VAL(39, "%"PRIu32, mb->decode_count);
        if (param_raw_labels[39]) lv_label_set_text(param_raw_labels[39], "");

        #undef SET_VAL
        #undef SET_RAW
        #undef SET_VAL_S
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

    // CAN Manager (NO_ACK mode — DO NOT CHANGE)
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

    build_dashboard();
    if (lvgl_port_lock(1000)) {
        lv_timer_create(dashboard_timer_cb, 200, NULL);
        lvgl_port_unlock();
    }
}
