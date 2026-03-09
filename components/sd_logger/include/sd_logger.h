#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// SD card SPI pins (SPI3/VSPI)
#define SD_PIN_MOSI  23
#define SD_PIN_MISO  19
#define SD_PIN_SCLK  18
#define SD_PIN_CS     5

// Mount point
#define SD_MOUNT_POINT "/sdcard"

// Minimum session duration (seconds) to keep log file
#define MIN_SESSION_SECONDS 300  // 5 minutes

// Initialize SD card (mount FATFS via SPI)
// Returns ESP_OK if card mounted, ESP_FAIL if no card
esp_err_t sd_logger_init(void);

// Check if SD card is mounted
bool sd_logger_is_mounted(void);

// Start a new logging session (creates new CSV file)
// Call when CAN data starts flowing
void sd_logger_start_session(void);

// Write a CAN message to the current log file
// Safe to call even if no session is active (will be ignored)
void sd_logger_write(uint32_t can_id, const uint8_t *data, uint8_t dlc);

// End the current session
// If session was shorter than 5 minutes, deletes the file
void sd_logger_end_session(void);

// List log files on SD card
// Returns number of files found, fills names array (caller provides buffer)
// Files are sorted newest first
typedef struct {
    char name[64];
    uint32_t size_bytes;
} sd_file_info_t;

int sd_logger_list_files(sd_file_info_t *files, int max_files);
