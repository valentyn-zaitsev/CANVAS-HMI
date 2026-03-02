#include "can_manager.h"
#include "can_driver.h"
#include "obd2_pids.h"
#include "vehicle_data.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <string.h>

static const char *TAG = "CAN_MANAGER";

static const uint8_t priority_pids[] = {
    PID_ENGINE_RPM,
    PID_VEHICLE_SPEED,
    PID_ENGINE_COOLANT_TEMP,
    PID_ENGINE_LOAD,
    PID_THROTTLE_POSITION,
    PID_FUEL_LEVEL,
    PID_INTAKE_AIR_TEMP,
    PID_FUEL_PRESSURE,
    PID_CONTROL_MODULE_VOLTAGE,
    PID_O2_SENSOR_1_B1,
};

#define NUM_PRIORITY_PIDS (sizeof(priority_pids) / sizeof(priority_pids[0]))

static TaskHandle_t can_manager_task_handle = NULL;
static bool can_manager_running = false;
static can_manager_stats_t stats = {0};

static void can_manager_task(void *arg) {
    can_message_t rx_msg;
    uint8_t current_pid_index = 0;

    ESP_LOGI(TAG, "CAN Manager task started");
    vehicle_data_init();

    while (can_manager_running) {
        uint8_t pid = priority_pids[current_pid_index];

        if (obd2_request_pid(pid) == ESP_OK) {
            stats.request_count++;

            if (can_receive_message(&rx_msg, 250) == ESP_OK) {
                if ((rx_msg.identifier >= OBD2_RESPONSE_CAN_ID_BASE) &&
                    (rx_msg.identifier <= OBD2_RESPONSE_CAN_ID_BASE + 7)) {

                    uint8_t response_pid;
                    float value;

                    if (obd2_parse_response(rx_msg.data, &response_pid, &value) == ESP_OK) {
                        vehicle_data_update(vehicle_data_get(), response_pid, value);
                        stats.response_count++;
                        stats.last_response_pid = response_pid;
                        stats.last_response_value = value;
                    }
                }
            } else {
                stats.error_count++;
            }
        }

        current_pid_index = (current_pid_index + 1) % NUM_PRIORITY_PIDS;

        if (stats.request_count > 0 && stats.request_count % 100 == 0) {
            ESP_LOGI(TAG, "Req: %" PRIu32 " Resp: %" PRIu32 " Err: %" PRIu32 " (%.0f%%)",
                stats.request_count, stats.response_count, stats.error_count,
                (stats.response_count * 100.0f) / stats.request_count);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    vTaskDelete(NULL);
}

esp_err_t can_manager_init(void) {
    ESP_LOGI(TAG, "Initializing CAN Manager");

    if (can_driver_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CAN driver");
        return ESP_FAIL;
    }

    if (obd2_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize OBD-II library");
        can_driver_deinit();
        return ESP_FAIL;
    }

    memset(&stats, 0, sizeof(stats));
    ESP_LOGI(TAG, "CAN Manager initialized");
    return ESP_OK;
}

esp_err_t can_manager_deinit(void) {
    can_manager_stop();
    can_driver_deinit();
    return ESP_OK;
}

esp_err_t can_manager_start(void) {
    if (can_manager_running) return ESP_OK;

    can_manager_running = true;

    if (xTaskCreate(can_manager_task, "can_manager", 4096, NULL, 8, &can_manager_task_handle) != pdPASS) {
        can_manager_running = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "CAN Manager started");
    return ESP_OK;
}

esp_err_t can_manager_stop(void) {
    if (!can_manager_running) return ESP_OK;
    can_manager_running = false;
    if (can_manager_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(100));
        can_manager_task_handle = NULL;
    }
    return ESP_OK;
}

bool can_manager_is_running(void) {
    return can_manager_running;
}

const can_manager_stats_t* can_manager_get_stats(void) {
    return &stats;
}
