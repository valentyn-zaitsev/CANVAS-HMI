#ifndef CAN_CONFIG_H
#define CAN_CONFIG_H

/**
 * CAN Driver Configuration
 * Customize these settings for your specific hardware and requirements
 */

// ============================================================================
// GPIO Configuration
// ============================================================================

// ESP32 GPIO pins for CAN communication
#define CAN_TX_GPIO 5                   // TWAI_TX pin
#define CAN_RX_GPIO 4                   // TWAI_RX pin

// ============================================================================
// CAN Bus Configuration
// ============================================================================

// CAN Baudrate (OBD-II standard is 500 kbps)
#define CAN_BAUDRATE 500000             // 500 kbps

// CAN message queue depth
#define CAN_RX_QUEUE_SIZE 32            // Number of messages to buffer

// CAN RX task configuration
#define CAN_RX_TASK_STACK_SIZE 4096     // Stack size in bytes
#define CAN_RX_TASK_PRIORITY 10         // FreeRTOS task priority

// ============================================================================
// OBD-II Configuration
// ============================================================================

// OBD-II request timeout (milliseconds)
#define OBD2_REQUEST_TIMEOUT_MS 500     // Wait up to 500ms for response

// Number of priority PIDs to cycle through
#define OBD2_NUM_PRIORITY_PIDS 10       // Request these PIDs in order

// Delay between PID requests (milliseconds)
#define OBD2_REQUEST_DELAY_MS 50        // 50ms between requests = ~20 Hz

// ============================================================================
// CAN Manager Configuration
// ============================================================================

// CAN Manager task configuration
#define CAN_MANAGER_TASK_STACK_SIZE 4096    // Stack size in bytes
#define CAN_MANAGER_TASK_PRIORITY 8         // FreeRTOS task priority

// Statistics logging interval (number of requests)
#define CAN_MANAGER_STATS_INTERVAL 100      // Log stats every 100 requests

// ============================================================================
// Vehicle Data Configuration
// ============================================================================

// Maximum number of vehicle data samples to keep in history
#define VEHICLE_DATA_HISTORY_SIZE 100

// ============================================================================
// Debug Configuration
// ============================================================================

// Enable debug logging
#define CAN_DEBUG_LOGGING 1             // 1 = enabled, 0 = disabled

// Log level for CAN driver
#define CAN_LOG_LEVEL ESP_LOG_INFO      // ESP_LOG_DEBUG, ESP_LOG_INFO, ESP_LOG_WARN, ESP_LOG_ERROR

// ============================================================================
// Feature Configuration
// ============================================================================

// Enable DTC (Diagnostic Trouble Code) reading
#define ENABLE_DTC_READING 1            // 1 = enabled, 0 = disabled

// Enable data logging to SD card
#define ENABLE_DATA_LOGGING 0           // 1 = enabled, 0 = disabled

// Enable ML inference
#define ENABLE_ML_INFERENCE 0           // 1 = enabled, 0 = disabled

// ============================================================================
// Mercedes-Specific Configuration
// ============================================================================

// Mercedes CLS400 specific settings
#define MERCEDES_CLS400_2015 1          // Enable Mercedes-specific PIDs

// Support manufacturer-specific PIDs
#define ENABLE_MANUFACTURER_PIDS 1      // 1 = enabled, 0 = disabled

// ============================================================================
// Performance Configuration
// ============================================================================

// Optimize for low power consumption
#define LOW_POWER_MODE 0                // 1 = enabled, 0 = disabled

// Optimize for high performance
#define HIGH_PERFORMANCE_MODE 0         // 1 = enabled, 0 = disabled

// ============================================================================
// Validation
// ============================================================================

// Validate configuration at compile time
#if CAN_TX_GPIO == CAN_RX_GPIO
#error "CAN_TX_GPIO and CAN_RX_GPIO must be different"
#endif

#if CAN_BAUDRATE < 100000 || CAN_BAUDRATE > 1000000
#error "CAN_BAUDRATE must be between 100 kbps and 1000 kbps"
#endif

#if OBD2_REQUEST_TIMEOUT_MS < 100 || OBD2_REQUEST_TIMEOUT_MS > 5000
#error "OBD2_REQUEST_TIMEOUT_MS must be between 100ms and 5000ms"
#endif

#endif // CAN_CONFIG_H
