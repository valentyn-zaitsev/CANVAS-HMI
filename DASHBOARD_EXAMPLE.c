/**
 * Example: Integrating CAN Data into LVGL Dashboard
 * 
 * This file shows how to update your LVGL dashboard with real vehicle data
 * from the CAN bus. You can use this as a reference for your own implementation.
 */

#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "vehicle_data.h"
#include "esp_log.h"

static const char *TAG = "DASHBOARD_CAN";

// Dashboard UI elements
static lv_obj_t *rpm_arc = NULL;
static lv_obj_t *rpm_label = NULL;
static lv_obj_t *speed_label = NULL;
static lv_obj_t *temp_label = NULL;
static lv_obj_t *fuel_bar = NULL;
static lv_obj_t *load_label = NULL;
static lv_obj_t *throttle_label = NULL;

/**
 * Create dashboard with CAN data display
 */
void create_can_dashboard(void) {
    if (lvgl_port_lock(100)) {
        lv_obj_t *scr = lv_screen_active();
        lv_obj_clean(scr);
        lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

        // ===== TOP SECTION: RPM GAUGE =====
        
        // RPM Arc Gauge
        rpm_arc = lv_arc_create(scr);
        lv_obj_set_size(rpm_arc, 200, 200);
        lv_obj_align(rpm_arc, LV_ALIGN_TOP_MID, 0, 10);
        lv_arc_set_rotation(rpm_arc, 135);
        lv_arc_set_bg_angles(rpm_arc, 0, 270);
        lv_arc_set_range(rpm_arc, 0, 8000);  // 0-8000 RPM
        lv_arc_set_value(rpm_arc, 0);
        
        lv_obj_set_style_arc_color(rpm_arc, lv_color_make(30, 30, 30), LV_PART_MAIN);
        lv_obj_set_style_arc_width(rpm_arc, 20, LV_PART_MAIN);
        lv_obj_set_style_arc_color(rpm_arc, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR);
        lv_obj_set_style_arc_width(rpm_arc, 20, LV_PART_INDICATOR);
        lv_obj_set_style_arc_rounded(rpm_arc, true, LV_PART_INDICATOR);
        
        // RPM Label
        rpm_label = lv_label_create(scr);
        lv_obj_set_style_text_font(rpm_label, &lv_font_montserrat_32, 0);
        lv_obj_set_style_text_color(rpm_label, lv_color_white(), 0);
        lv_label_set_text(rpm_label, "0 RPM");
        lv_obj_align(rpm_label, LV_ALIGN_TOP_MID, 0, 90);

        // ===== MIDDLE SECTION: KEY PARAMETERS =====
        
        // Speed
        speed_label = lv_label_create(scr);
        lv_obj_set_style_text_font(speed_label, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_color(speed_label, lv_color_white(), 0);
        lv_label_set_text(speed_label, "Speed: 0 km/h");
        lv_obj_align(speed_label, LV_ALIGN_LEFT_MID, 10, -40);

        // Temperature
        temp_label = lv_label_create(scr);
        lv_obj_set_style_text_font(temp_label, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_color(temp_label, lv_color_white(), 0);
        lv_label_set_text(temp_label, "Temp: 0°C");
        lv_obj_align(temp_label, LV_ALIGN_LEFT_MID, 10, 0);

        // Engine Load
        load_label = lv_label_create(scr);
        lv_obj_set_style_text_font(load_label, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_color(load_label, lv_color_white(), 0);
        lv_label_set_text(load_label, "Load: 0%");
        lv_obj_align(load_label, LV_ALIGN_LEFT_MID, 10, 40);

        // ===== BOTTOM SECTION: FUEL & THROTTLE =====
        
        // Fuel Level Bar
        fuel_bar = lv_bar_create(scr);
        lv_obj_set_size(fuel_bar, 300, 30);
        lv_obj_align(fuel_bar, LV_ALIGN_BOTTOM_MID, 0, -50);
        lv_bar_set_range(fuel_bar, 0, 100);
        lv_bar_set_value(fuel_bar, 50, LV_ANIM_ON);
        lv_obj_set_style_bg_color(fuel_bar, lv_color_make(30, 30, 30), LV_PART_MAIN);
        lv_obj_set_style_bg_color(fuel_bar, lv_palette_main(LV_PALETTE_GREEN), LV_PART_INDICATOR);
        
        // Fuel Label
        lv_obj_t *fuel_label = lv_label_create(scr);
        lv_obj_set_style_text_font(fuel_label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(fuel_label, lv_color_white(), 0);
        lv_label_set_text(fuel_label, "Fuel Level");
        lv_obj_align(fuel_label, LV_ALIGN_BOTTOM_MID, 0, -85);

        // Throttle Position
        throttle_label = lv_label_create(scr);
        lv_obj_set_style_text_font(throttle_label, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(throttle_label, lv_color_white(), 0);
        lv_label_set_text(throttle_label, "Throttle: 0%");
        lv_obj_align(throttle_label, LV_ALIGN_BOTTOM_MID, 0, -10);

        lvgl_port_unlock();
    }
}

/**
 * Update dashboard with current vehicle data
 * Call this periodically (e.g., every 100ms) to refresh the display
 */
void update_can_dashboard(void) {
    if (lvgl_port_lock(100)) {
        vehicle_data_t *data = vehicle_data_get();
        
        // Update RPM gauge and label
        if (rpm_arc != NULL) {
            uint16_t rpm_clamped = (data->rpm > 8000) ? 8000 : data->rpm;
            lv_arc_set_value(rpm_arc, rpm_clamped);
        }
        
        if (rpm_label != NULL) {
            lv_label_set_text_fmt(rpm_label, "%u RPM", data->rpm);
        }
        
        // Update speed
        if (speed_label != NULL) {
            lv_label_set_text_fmt(speed_label, "Speed: %u km/h", data->vehicle_speed);
        }
        
        // Update temperature with color coding
        if (temp_label != NULL) {
            lv_label_set_text_fmt(temp_label, "Temp: %d°C", data->coolant_temp);
            
            // Color code: green (normal), yellow (warning), red (critical)
            lv_color_t color;
            if (data->coolant_temp < 80) {
                color = lv_color_make(100, 200, 255);  // Light blue - cold
            } else if (data->coolant_temp < 100) {
                color = lv_palette_main(LV_PALETTE_GREEN);  // Green - normal
            } else if (data->coolant_temp < 110) {
                color = lv_palette_main(LV_PALETTE_YELLOW);  // Yellow - warning
            } else {
                color = lv_palette_main(LV_PALETTE_RED);  // Red - critical
            }
            lv_obj_set_style_text_color(temp_label, color, 0);
        }
        
        // Update engine load
        if (load_label != NULL) {
            lv_label_set_text_fmt(load_label, "Load: %u%%", data->engine_load);
        }
        
        // Update fuel level bar
        if (fuel_bar != NULL) {
            lv_bar_set_value(fuel_bar, data->fuel_level, LV_ANIM_ON);
            
            // Color code fuel level
            lv_color_t fuel_color;
            if (data->fuel_level > 50) {
                fuel_color = lv_palette_main(LV_PALETTE_GREEN);
            } else if (data->fuel_level > 25) {
                fuel_color = lv_palette_main(LV_PALETTE_YELLOW);
            } else {
                fuel_color = lv_palette_main(LV_PALETTE_RED);
            }
            lv_obj_set_style_bg_color(fuel_bar, fuel_color, LV_PART_INDICATOR);
        }
        
        // Update throttle position
        if (throttle_label != NULL) {
            lv_label_set_text_fmt(throttle_label, "Throttle: %u%%", data->throttle_position);
        }
        
        lvgl_port_unlock();
    }
}

/**
 * Dashboard update task
 * Run this as a FreeRTOS task to continuously update the display
 */
void dashboard_update_task(void *arg) {
    ESP_LOGI(TAG, "Dashboard update task started");
    
    // Create dashboard
    create_can_dashboard();
    
    // Update loop
    while (1) {
        update_can_dashboard();
        vTaskDelay(pdMS_TO_TICKS(100));  // Update every 100ms
    }
}

/**
 * Example: How to use in app_main()
 * 
 * void app_main(void) {
 *     // ... initialize LCD, touch, LVGL, CAN manager ...
 *     
 *     // Create dashboard update task
 *     xTaskCreate(dashboard_update_task, "dashboard", 4096, NULL, 5, NULL);
 *     
 *     // ... rest of your code ...
 * }
 */

/**
 * Alternative: Simple update in main loop
 * 
 * void app_main(void) {
 *     // ... initialize LCD, touch, LVGL, CAN manager ...
 *     
 *     // Create dashboard once
 *     create_can_dashboard();
 *     
 *     // Main loop
 *     while (1) {
 *         update_can_dashboard();
 *         vTaskDelay(pdMS_TO_TICKS(100));
 *     }
 * }
 */
