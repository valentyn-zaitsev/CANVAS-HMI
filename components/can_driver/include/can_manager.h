#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * CAN Manager - Handles periodic OBD-II PID requests and responses
 */

/**
 * Initialize CAN manager
 * @return ESP_OK on success
 */
esp_err_t can_manager_init(void);

/**
 * Deinitialize CAN manager
 * @return ESP_OK on success
 */
esp_err_t can_manager_deinit(void);

/**
 * Start CAN manager task
 * @return ESP_OK on success
 */
esp_err_t can_manager_start(void);

/**
 * Stop CAN manager task
 * @return ESP_OK on success
 */
esp_err_t can_manager_stop(void);

/**
 * Check if CAN manager is running
 * @return true if running
 */
bool can_manager_is_running(void);

#ifdef __cplusplus
}
#endif

#endif // CAN_MANAGER_H
