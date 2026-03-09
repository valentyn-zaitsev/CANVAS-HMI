#include "can_driver.h"
#include "can_sniffer.h"
#include "mercedes_decode.h"
#include "sd_logger.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <string.h>
#include "esp_rom_sys.h"

static bool logging_session_active = false;

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

    // Enable bus-off recovery alerts
    twai_reconfigure_alerts(TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_ERR_PASS, NULL);

    while (can_initialized) {
        // Check for alerts (bus-off recovery)
        uint32_t alerts;
        if (twai_read_alerts(&alerts, 0) == ESP_OK) {
            if (alerts & TWAI_ALERT_BUS_OFF) {
                ESP_LOGW(TAG, "Bus-off detected, initiating recovery");
                twai_initiate_recovery();
            }
            if (alerts & TWAI_ALERT_BUS_RECOVERED) {
                ESP_LOGI(TAG, "Bus recovered, restarting");
                twai_start();
            }
            if (alerts & TWAI_ALERT_ERR_PASS) {
                ESP_LOGW(TAG, "Error passive state");
            }
        }

        // Wait for message with 100ms timeout
        if (twai_receive(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
            // Record in sniffer (sees ALL CAN traffic)
            can_sniffer_record(message.identifier, message.data, message.data_length_code);

            // Decode known Mercedes broadcast messages
            mercedes_decode_message(message.identifier, message.data, message.data_length_code);

            // Log to SD card
            if (sd_logger_is_mounted()) {
                if (!logging_session_active) {
                    sd_logger_start_session();
                    logging_session_active = true;
                }
                sd_logger_write(message.identifier, message.data, message.data_length_code);
            }

            // Convert TWAI message to our format and queue it
            can_msg.identifier = message.identifier;
            can_msg.data_length_code = message.data_length_code;
            can_msg.timestamp = xTaskGetTickCount();

            for (int i = 0; i < message.data_length_code; i++) {
                can_msg.data[i] = message.data[i];
            }

            // Send to queue (silently drop if full - no log spam)
            xQueueSend(can_rx_queue, &can_msg, 0);
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

    // NO_ACK mode: proven to work on real Mercedes CAN bus
    // Does not require ACK from other nodes, works both standalone and on live bus
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NO_ACK);
    g_config.rx_queue_len = 32;  // Default 5 is too small for busy CAN bus
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
    can_sniffer_init();
    mercedes_decode_init();

    // Create CAN RX task
    if (xTaskCreate(can_rx_task, "can_rx", 4096, NULL, 10, &can_rx_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CAN RX task");
        twai_stop();
        twai_driver_uninstall();
        vQueueDelete(can_rx_queue);
        can_initialized = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "CAN driver initialized (NO_ACK mode, 500kbps, rx_q=32)");
    ESP_LOGI(TAG, "TX GPIO: %d, RX GPIO: %d", CAN_TX_GPIO, CAN_RX_GPIO);

    return ESP_OK;
}

esp_err_t can_driver_deinit(void) {
    if (!can_initialized) {
        return ESP_OK;
    }

    can_initialized = false;

    // End SD logging session (deletes file if < 5 min)
    if (logging_session_active) {
        sd_logger_end_session();
        logging_session_active = false;
    }

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
        .extd = 0,
        .rtr = 0,
        .ss = 0,
        .self = 0,
        .dlc_non_comp = 0,
    };

    for (int i = 0; i < msg->data_length_code; i++) {
        twai_msg.data[i] = msg->data[i];
    }

    esp_err_t ret = twai_transmit(&twai_msg, pdMS_TO_TICKS(100));

    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "CAN TX failed (0x%03" PRIX32 ") err=%d", msg->identifier, ret);
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

esp_err_t can_driver_get_debug_info(can_debug_info_t *info) {
    if (info == NULL) return ESP_ERR_INVALID_ARG;
    if (!can_initialized) {
        memset(info, 0, sizeof(can_debug_info_t));
        return ESP_OK;
    }
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        info->state = (uint8_t)status.state;
        info->tx_error_counter = status.tx_error_counter;
        info->rx_error_counter = status.rx_error_counter;
        info->tx_failed_count = status.tx_failed_count;
        info->rx_missed_count = status.rx_missed_count;
        info->bus_error_count = status.bus_error_count;
        info->arb_lost_count = status.arb_lost_count;
        info->msgs_to_rx = status.msgs_to_rx;
        return ESP_OK;
    }
    return ESP_FAIL;
}
