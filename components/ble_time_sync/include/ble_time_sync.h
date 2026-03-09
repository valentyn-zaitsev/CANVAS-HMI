#pragma once

#include <stdbool.h>
#include <stdint.h>

// Initialize and start BLE time sync service
// ESP32 advertises as "MercedesCAN" with a writable time characteristic
void ble_time_sync_init(void);

// Check if time has been synced via BLE
bool ble_time_sync_is_synced(void);

// Get seconds since last sync (0 if never synced)
uint32_t ble_time_sync_age(void);
