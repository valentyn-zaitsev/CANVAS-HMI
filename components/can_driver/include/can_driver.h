#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * CAN Driver Configuration
 * SN65HVD230 is a CAN transceiver (not a controller)
 * ESP32 has built-in TWAI (Two-Wire Automotive Interface) controller
 * 
 * Wiring:
 * SN65HVD230 D (CAN_TX) -> ESP32 GPIO 5 (TWAI_TX)
 * SN65HVD230 R (CAN_RX) -> ESP32 GPIO 4 (TWAI_RX)
 * SN65HVD230 GND -> ESP32 GND
 * SN65HVD230 VCC -> 3.3V
 * SN65HVD230 CANH -> Vehicle CAN_H
 * SN65HVD230 CANL -> Vehicle CAN_L
 */

#define CAN_TX_GPIO 5
#define CAN_RX_GPIO 4
#define CAN_BAUDRATE 500000  // OBD-II standard: 500 kbps

// CAN Message structure
typedef struct {
    uint32_t identifier;
    uint8_t data_length_code;
    uint8_t data[8];
    uint32_t timestamp;
} can_message_t;

/**
 * Initialize CAN driver
 * @return ESP_OK on success
 */
esp_err_t can_driver_init(void);

/**
 * Deinitialize CAN driver
 * @return ESP_OK on success
 */
esp_err_t can_driver_deinit(void);

/**
 * Send CAN message
 * @param msg Pointer to CAN message
 * @return ESP_OK on success
 */
esp_err_t can_send_message(const can_message_t *msg);

/**
 * Receive CAN message (blocking with timeout)
 * @param msg Pointer to receive buffer
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if no message received
 */
esp_err_t can_receive_message(can_message_t *msg, uint32_t timeout_ms);

/**
 * Check if CAN message is available
 * @return true if message available, false otherwise
 */
bool can_message_available(void);

/**
 * Get CAN driver status
 * @return true if driver is initialized and running
 */
bool can_driver_is_running(void);

#ifdef __cplusplus
}
#endif

#endif // CAN_DRIVER_H
