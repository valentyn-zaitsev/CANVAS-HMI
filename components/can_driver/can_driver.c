#include "can_driver.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <inttypes.h>

static const char *TAG = "CAN_DRIVER";

// Message queue
static QueueHandle_t can_rx_queue = NULL;
static bool can_initialized = false;

// CAN RX task handle
static TaskHandle_t can_rx_task_handle = NULL;

/**
 * CAN RX task - receives messages from CAN bus
 */
static void can_rx_task(void *arg) {
    twai_message_t message;
    can_message_t can_msg;
    
    ESP_LOGI(TAG, "CAN RX task started");
    
    while (can_initialized) {
        // Wait for message with 100ms timeout
        if (twai_receive(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
            // Convert TWAI message to our format
            can_msg.identifier = message.identifier;
            can_msg.data_length_code = message.data_length_code;
            can_msg.timestamp = xTaskGetTickCount();
            
            for (int i = 0; i < message.data_length_code; i++) {
                can_msg.data[i] = message.data[i];
            }
            
            // Send to queue
            if (xQueueSend(can_rx_queue, &can_msg, 0) != pdTRUE) {
                ESP_LOGW(TAG, "CAN RX queue full, dropping message");
            }
        }
    }
    
    vTaskDelete(NULL);
}

esp_err_t can_driver_init(void) {
    if (can_initialized) {
        ESP_LOGW(TAG, "CAN driver already initialized");
        return ESP_OK;
    }
    
    // Create message queue
    can_rx_queue = xQueueCreate(32, sizeof(can_message_t));
    if (can_rx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create CAN RX queue");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize TWAI configuration
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver");
        vQueueDelete(can_rx_queue);
        return ESP_FAIL;
    }
    
    // Start TWAI driver
    if (twai_start() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver");
        twai_driver_uninstall();
        vQueueDelete(can_rx_queue);
        return ESP_FAIL;
    }
    
    can_initialized = true;
    
    // Create CAN RX task
    if (xTaskCreate(can_rx_task, "can_rx", 4096, NULL, 10, &can_rx_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CAN RX task");
        twai_stop();
        twai_driver_uninstall();
        vQueueDelete(can_rx_queue);
        can_initialized = false;
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "CAN driver initialized successfully");
    ESP_LOGI(TAG, "CAN TX GPIO: %d, CAN RX GPIO: %d, Baudrate: %d", CAN_TX_GPIO, CAN_RX_GPIO, CAN_BAUDRATE);
    
    return ESP_OK;
}

esp_err_t can_driver_deinit(void) {
    if (!can_initialized) {
        return ESP_OK;
    }
    
    can_initialized = false;
    
    // Wait for RX task to finish
    if (can_rx_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Stop and uninstall TWAI driver
    twai_stop();
    twai_driver_uninstall();
    
    // Delete queue
    if (can_rx_queue != NULL) {
        vQueueDelete(can_rx_queue);
        can_rx_queue = NULL;
    }
    
    ESP_LOGI(TAG, "CAN driver deinitialized");
    return ESP_OK;
}

esp_err_t can_send_message(const can_message_t *msg) {
    if (!can_initialized || msg == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    twai_message_t twai_msg = {
        .identifier = msg->identifier,
        .data_length_code = msg->data_length_code,
        .extd = 0,  // Standard CAN ID (11-bit)
        .rtr = 0,   // Not a remote frame
        .ss = 0,
        .self = 0,
        .dlc_non_comp = 0,
    };
    
    for (int i = 0; i < msg->data_length_code; i++) {
        twai_msg.data[i] = msg->data[i];
    }
    
    esp_err_t ret = twai_transmit(&twai_msg, pdMS_TO_TICKS(100));

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send CAN message (0x%03" PRIX32 ")", msg->identifier);
    }
    
    return ret;
}

esp_err_t can_receive_message(can_message_t *msg, uint32_t timeout_ms) {
    if (!can_initialized || msg == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xQueueReceive(can_rx_queue, msg, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

bool can_message_available(void) {
    if (!can_initialized || can_rx_queue == NULL) {
        return false;
    }
    
    return uxQueueMessagesWaiting(can_rx_queue) > 0;
}

bool can_driver_is_running(void) {
    return can_initialized;
}
