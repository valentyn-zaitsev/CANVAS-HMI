#include "can_manager.h"
#include "can_driver.h"
#include "obd2_pids.h"
#include "vehicle_data.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>

static const char *TAG = "CAN_MANAGER";

// Priority PIDs to request (in order)
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

/**
 * CAN Manager Task
 * Periodically requests PIDs and processes responses
 */
static void can_manager_task(void *arg) {
    can_message_t rx_msg;
    uint8_t current_pid_index = 0;
    uint32_t request_count = 0;
    uint32_t response_count = 0;
    uint32_t error_count = 0;
    
    ESP_LOGI(TAG, "CAN Manager task started");
    
    // Initialize vehicle data
    vehicle_data_init();
    
    while (can_manager_running) {
        // Request next PID
        uint8_t pid = priority_pids[current_pid_index];
        
        if (obd2_request_pid(pid) == ESP_OK) {
            request_count++;
            
            // Wait for response (500ms timeout)
            if (can_receive_message(&rx_msg, 500) == ESP_OK) {
                // Check if this is an OBD-II response
                if ((rx_msg.identifier >= OBD2_RESPONSE_CAN_ID_BASE) && 
                    (rx_msg.identifier <= OBD2_RESPONSE_CAN_ID_BASE + 7)) {
                    
                    uint8_t response_pid;
                    float value;
                    
                    if (obd2_parse_response(rx_msg.data, &response_pid, &value) == ESP_OK) {
                        // Update vehicle data
                        vehicle_data_update(vehicle_data_get(), response_pid, value);
                        response_count++;
                        
                        ESP_LOGD(TAG, "PID 0x%02X: %s = %.2f %s",
                            response_pid,
                            obd2_get_pid_name(response_pid),
                            value,
                            obd2_get_pid_unit(response_pid)
                        );
                    }
                }
            } else {
                error_count++;
                ESP_LOGW(TAG, "No response for PID 0x%02X", pid);
            }
        }
        
        // Move to next PID
        current_pid_index = (current_pid_index + 1) % NUM_PRIORITY_PIDS;
        
        // Log statistics every 100 requests
        if (request_count % 100 == 0) {
            ESP_LOGI(TAG, "Requests: %" PRIu32 ", Responses: %" PRIu32 ", Errors: %" PRIu32 ", Success rate: %.1f%%",
                request_count,
                response_count,
                error_count,
                (response_count * 100.0f) / request_count
            );
        }
        
        // Small delay between requests
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    ESP_LOGI(TAG, "CAN Manager task stopped");
    vTaskDelete(NULL);
}

esp_err_t can_manager_init(void) {
    ESP_LOGI(TAG, "Initializing CAN Manager");
    
    // Initialize CAN driver
    if (can_driver_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CAN driver");
        return ESP_FAIL;
    }
    
    // Initialize OBD-II library
    if (obd2_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize OBD-II library");
        can_driver_deinit();
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "CAN Manager initialized successfully");
    return ESP_OK;
}

esp_err_t can_manager_deinit(void) {
    can_manager_stop();
    can_driver_deinit();
    ESP_LOGI(TAG, "CAN Manager deinitialized");
    return ESP_OK;
}

esp_err_t can_manager_start(void) {
    if (can_manager_running) {
        ESP_LOGW(TAG, "CAN Manager already running");
        return ESP_OK;
    }
    
    can_manager_running = true;
    
    if (xTaskCreate(can_manager_task, "can_manager", 4096, NULL, 8, &can_manager_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CAN Manager task");
        can_manager_running = false;
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "CAN Manager started");
    return ESP_OK;
}

esp_err_t can_manager_stop(void) {
    if (!can_manager_running) {
        return ESP_OK;
    }
    
    can_manager_running = false;
    
    // Wait for task to finish
    if (can_manager_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(100));
        can_manager_task_handle = NULL;
    }
    
    ESP_LOGI(TAG, "CAN Manager stopped");
    return ESP_OK;
}

bool can_manager_is_running(void) {
    return can_manager_running;
}
