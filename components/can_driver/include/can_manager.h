#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t request_count;
    uint32_t response_count;
    uint32_t error_count;
    uint8_t last_response_pid;
    float last_response_value;
} can_manager_stats_t;

esp_err_t can_manager_init(void);
esp_err_t can_manager_deinit(void);
esp_err_t can_manager_start(void);
esp_err_t can_manager_stop(void);
bool can_manager_is_running(void);
const can_manager_stats_t* can_manager_get_stats(void);

#ifdef __cplusplus
}
#endif

#endif // CAN_MANAGER_H
