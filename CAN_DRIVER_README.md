# CAN Driver Integration for ESP32 LVGL Project

## Overview

This document describes the CAN bus driver integration for reading OBD-II data from your Mercedes CLS400 2015 via the SN65HVD230 CAN transceiver.

## Hardware Setup

### Wiring Diagram

```
ESP32                    SN65HVD230              Vehicle CAN Bus
─────────────────────────────────────────────────────────────
GPIO 5 (TWAI_TX)  ──────→ D (CAN_TX)
GPIO 4 (TWAI_RX)  ←────── R (CAN_RX)
GND               ──────→ GND
3.3V              ──────→ VCC
                         CANH ──────→ Vehicle CAN_H
                         CANL ──────→ Vehicle CAN_L
```

### Pin Configuration

| ESP32 Pin | Function | SN65HVD230 Pin |
|-----------|----------|----------------|
| GPIO 5    | TWAI_TX  | D (CAN_TX)     |
| GPIO 4    | TWAI_RX  | R (CAN_RX)     |
| GND       | Ground   | GND            |
| 3.3V      | Power    | VCC            |

### CAN Bus Parameters

- **Baudrate:** 500 kbps (OBD-II standard)
- **Protocol:** OBD-II (ISO 15031-5)
- **Message Format:** Standard CAN (11-bit identifiers)

## Software Architecture

### Component Structure

```
components/can_driver/
├── CMakeLists.txt
├── include/
│   ├── can_driver.h       # Low-level CAN driver
│   ├── obd2_pids.h        # OBD-II PID definitions
│   ├── vehicle_data.h     # Vehicle data structure
│   └── can_manager.h      # High-level CAN manager
├── can_driver.c           # CAN driver implementation
├── obd2_pids.c            # OBD-II PID library
├── vehicle_data.c         # Vehicle data management
└── can_manager.c          # CAN manager task
```

### Module Descriptions

#### 1. **can_driver.c / can_driver.h**
Low-level CAN communication using ESP32's TWAI (Two-Wire Automotive Interface) controller.

**Key Functions:**
- `can_driver_init()` - Initialize CAN driver
- `can_send_message()` - Send CAN message
- `can_receive_message()` - Receive CAN message (blocking)
- `can_message_available()` - Check if message is available

**Features:**
- Uses FreeRTOS queue for message buffering
- Background RX task for non-blocking reception
- 32-message queue depth

#### 2. **obd2_pids.c / obd2_pids.h**
OBD-II protocol implementation with PID database.

**Key Functions:**
- `obd2_request_pid()` - Request a specific PID
- `obd2_parse_response()` - Parse OBD-II response
- `obd2_get_pid_info()` - Get PID metadata
- `obd2_decode_pid()` - Decode raw PID data

**Supported PIDs:**
- Engine: RPM, Load, Coolant Temp, Intake Air Temp, Throttle Position
- Fuel: Pressure, Level, Trim values
- Emissions: O2 sensors, Lambda
- Electrical: Battery voltage
- And 30+ more standard PIDs

**Decode Functions:**
Each PID has a custom decode function that converts raw CAN data to human-readable values:
- RPM: `(A*256 + B) / 4`
- Speed: `A` (km/h)
- Temperature: `A - 40` (°C)
- Throttle: `(A * 100) / 255` (%)

#### 3. **vehicle_data.c / vehicle_data.h**
Stores current vehicle parameters and provides data access.

**Data Structure:**
```c
typedef struct {
    uint16_t rpm;
    uint8_t engine_load;
    int8_t coolant_temp;
    int8_t intake_air_temp;
    uint8_t throttle_position;
    uint8_t fuel_level;
    uint16_t fuel_pressure;
    uint8_t vehicle_speed;
    uint16_t battery_voltage;
    // ... more fields
} vehicle_data_t;
```

**Key Functions:**
- `vehicle_data_init()` - Initialize data structure
- `vehicle_data_update()` - Update with new PID value
- `vehicle_data_get()` - Get current data
- `vehicle_data_to_string()` - Format data as string

#### 4. **can_manager.c / can_manager.h**
High-level manager that orchestrates CAN communication.

**Key Functions:**
- `can_manager_init()` - Initialize all components
- `can_manager_start()` - Start background task
- `can_manager_stop()` - Stop background task

**Features:**
- Periodic PID requests (10 priority PIDs)
- Automatic response parsing
- Vehicle data updates
- Statistics logging (success rate, error count)

**Priority PIDs (requested in order):**
1. Engine RPM
2. Vehicle Speed
3. Coolant Temperature
4. Engine Load
5. Throttle Position
6. Fuel Level
7. Intake Air Temperature
8. Fuel Pressure
9. Battery Voltage
10. O2 Sensor 1 B1

## Usage

### Basic Integration

```c
#include "can_manager.h"
#include "vehicle_data.h"

// In app_main():
// Initialize and start CAN manager
if (can_manager_init() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize CAN Manager");
} else {
    if (can_manager_start() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start CAN Manager");
    }
}

// In main loop or task:
vehicle_data_t *data = vehicle_data_get();
printf("RPM: %u\n", data->rpm);
printf("Speed: %u km/h\n", data->vehicle_speed);
printf("Coolant: %d°C\n", data->coolant_temp);
```

### Accessing Vehicle Data

```c
vehicle_data_t *data = vehicle_data_get();

// Engine parameters
uint16_t rpm = data->rpm;
uint8_t load = data->engine_load;
int8_t temp = data->coolant_temp;

// Fuel system
uint8_t fuel_level = data->fuel_level;
uint16_t fuel_pressure = data->fuel_pressure;

// Vehicle speed
uint8_t speed = data->vehicle_speed;

// Electrical
uint16_t battery_mv = data->battery_voltage;  // in millivolts
```

### Requesting Custom PIDs

```c
#include "can_driver.h"
#include "obd2_pids.h"

// Request a specific PID
obd2_request_pid(PID_ENGINE_RPM);

// Receive response
can_message_t msg;
if (can_receive_message(&msg, 500) == ESP_OK) {
    uint8_t pid;
    float value;
    if (obd2_parse_response(msg.data, &pid, &value) == ESP_OK) {
        printf("PID 0x%02X = %.2f\n", pid, value);
    }
}
```

## OBD-II Communication Protocol

### Request Format

```
CAN ID: 0x7DF (Broadcast)
Data:   [Length] [Service] [PID] [0x00] [0x00] [0x00] [0x00] [0x00]
        [0x02]   [0x01]    [PID] [0x00] [0x00] [0x00] [0x00] [0x00]
```

### Response Format

```
CAN ID: 0x7E8 - 0x7EF (ECU Response)
Data:   [Length] [Service+0x40] [PID] [Data_A] [Data_B] [0x00] [0x00] [0x00]
        [0x03]   [0x41]         [PID] [Data_A] [Data_B] [0x00] [0x00] [0x00]
```

### Example: Reading RPM

**Request:**
```
CAN ID: 0x7DF
Data:   02 01 0C 00 00 00 00 00
        └─ Length
           └─ Service (Current Data)
              └─ PID (Engine RPM)
```

**Response:**
```
CAN ID: 0x7E8
Data:   03 41 0C 1A F4 00 00 00
        └─ Length
           └─ Service Response (0x41 = 0x01 + 0x40)
              └─ PID (Engine RPM)
                 └─ Data A (26)
                    └─ Data B (244)
                       
RPM = ((26 * 256) + 244) / 4 = 1700 RPM
```

## Troubleshooting

### No CAN Messages Received

1. **Check wiring:**
   - Verify GPIO 4 and 5 connections
   - Ensure GND is connected
   - Check CAN_H and CAN_L connections to vehicle

2. **Check CAN transceiver:**
   - Verify SN65HVD230 power (3.3V)
   - Check for shorts on CAN bus
   - Use oscilloscope to verify CAN signals

3. **Check logs:**
   ```
   I (xxx) CAN_DRIVER: CAN driver initialized successfully
   I (xxx) CAN_MANAGER: CAN Manager started
   ```

### Low Success Rate

- Increase timeout in `can_receive_message()` (currently 500ms)
- Check vehicle CAN bus load
- Verify OBD-II adapter is properly connected
- Try different PIDs (some may not be supported by your vehicle)

### Incorrect Values

- Verify PID decode functions match your vehicle's format
- Check if Mercedes uses manufacturer-specific PIDs
- Compare with OBD-II scanner app

## Performance Metrics

- **CAN Baudrate:** 500 kbps
- **Request Rate:** ~20 Hz (50ms between requests)
- **Response Timeout:** 500ms
- **Expected Success Rate:** 80-95% (depends on vehicle)
- **Memory Usage:** ~2 KB (vehicle data) + 4 KB (CAN RX task)

## Future Enhancements

1. **Extended PIDs:** Add Mercedes-specific manufacturer PIDs
2. **DTC Reading:** Read and clear Diagnostic Trouble Codes
3. **Data Logging:** Log to SD card for analysis
4. **ML Integration:** Anomaly detection and predictive maintenance
5. **Multi-frame Support:** Handle responses longer than 8 bytes
6. **ISO-TP Protocol:** For advanced diagnostics

## References

- [OBD-II Standard (ISO 15031-5)](https://en.wikipedia.org/wiki/OBD-II_PIDs)
- [ESP32 TWAI Driver Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html)
- [SN65HVD230 Datasheet](https://www.ti.com/product/SN65HVD230)
- [Mercedes CLS400 OBD-II PIDs](https://www.obd-codes.com/)

## License

This component is part of the ESP32 LVGL project.
