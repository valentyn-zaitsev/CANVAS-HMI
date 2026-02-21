# CAN Driver Quick Start Guide

## Step 1: Hardware Wiring

Connect your SN65HVD230 CAN transceiver to the ESP32:

```
ESP32 Pin 5  (GPIO 5)  → SN65HVD230 Pin D (CAN_TX)
ESP32 Pin 4  (GPIO 4)  → SN65HVD230 Pin R (CAN_RX)
ESP32 GND             → SN65HVD230 GND
ESP32 3.3V            → SN65HVD230 VCC

SN65HVD230 CANH       → Vehicle OBD-II CAN_H (Pin 6)
SN65HVD230 CANL       → Vehicle OBD-II CAN_L (Pin 14)
```

## Step 2: Build and Flash

```bash
cd /home/tantrum/Documents/esp/esp32-lvgl
idf.py build
idf.py flash
idf.py monitor
```

## Step 3: Monitor Output

You should see logs like:

```
I (xxx) CAN_DRIVER: CAN driver initialized successfully
I (xxx) CAN_DRIVER: CAN TX GPIO: 5, CAN RX GPIO: 4, Baudrate: 500000
I (xxx) OBD2_PIDS: OBD-II PID library initialized
I (xxx) CAN_MANAGER: Initializing CAN Manager
I (xxx) CAN_MANAGER: CAN Manager initialized successfully
I (xxx) CAN_MANAGER: CAN Manager started
I (xxx) CAN_MANAGER: CAN Manager task started
D (xxx) CAN_MANAGER: PID 0x0C: Engine RPM = 1234.50 rpm
D (xxx) CAN_MANAGER: PID 0x0D: Vehicle Speed = 65 km/h
D (xxx) CAN_MANAGER: PID 0x05: Coolant Temperature = 92 °C
```

## Step 4: Access Vehicle Data

In your code, you can now access real-time vehicle data:

```c
#include "vehicle_data.h"

// Get current vehicle data
vehicle_data_t *data = vehicle_data_get();

// Use the data
printf("RPM: %u\n", data->rpm);
printf("Speed: %u km/h\n", data->vehicle_speed);
printf("Coolant: %d°C\n", data->coolant_temp);
printf("Fuel: %u%%\n", data->fuel_level);
printf("Battery: %u.%uV\n", 
    data->battery_voltage / 1000,
    (data->battery_voltage % 1000) / 100
);
```

## Step 5: Display on LVGL

Update your dashboard to show real vehicle data:

```c
void update_dashboard_with_can_data(void) {
    if (lvgl_port_lock(100)) {
        vehicle_data_t *data = vehicle_data_get();
        
        // Update RPM gauge (0-8000 range)
        lv_arc_set_value(rpm_arc, (data->rpm * 100) / 8000);
        
        // Update speed label
        lv_label_set_text_fmt(speed_label, "%u km/h", data->vehicle_speed);
        
        // Update temperature label
        lv_label_set_text_fmt(temp_label, "%d°C", data->coolant_temp);
        
        // Update fuel level
        lv_bar_set_value(fuel_bar, data->fuel_level, LV_ANIM_ON);
        
        lvgl_port_unlock();
    }
}
```

## Supported PIDs

The driver supports these PIDs by default:

| PID  | Name                      | Unit  |
|------|---------------------------|-------|
| 0x0C | Engine RPM                | rpm   |
| 0x0D | Vehicle Speed             | km/h  |
| 0x05 | Coolant Temperature       | °C    |
| 0x04 | Engine Load               | %     |
| 0x11 | Throttle Position         | %     |
| 0x2F | Fuel Level                | %     |
| 0x0F | Intake Air Temperature    | °C    |
| 0x0A | Fuel Pressure             | kPa   |
| 0x42 | Control Module Voltage    | V     |
| 0x14 | O2 Sensor 1 B1            | V     |

## Troubleshooting

### Issue: "Failed to initialize CAN Manager"

**Solution:**
- Check GPIO 4 and 5 are not used by other peripherals
- Verify SN65HVD230 power supply (3.3V)
- Check for loose connections

### Issue: No PID responses received

**Solution:**
- Verify vehicle is running (OBD-II port is powered)
- Check CAN bus connections (CANH and CANL)
- Use a multimeter to verify voltage on CAN lines
- Try with an OBD-II scanner app to verify vehicle is responding

### Issue: Incorrect values

**Solution:**
- Some Mercedes models use manufacturer-specific PIDs
- Check if your vehicle supports standard OBD-II
- Compare values with OBD-II scanner app
- Verify decode functions match your vehicle's format

## Next Steps

1. **Add more PIDs:** Edit `priority_pids[]` in `can_manager.c`
2. **Add data logging:** Save vehicle data to SD card
3. **Add ML models:** Implement anomaly detection
4. **Improve UI:** Create professional dashboard screens
5. **Add alerts:** Implement warning system for critical values

## Files Modified

- `main/main.c` - Added CAN manager initialization
- `components/can_driver/` - New CAN driver component

## Component Files

- `can_driver.c` - CAN hardware driver
- `obd2_pids.c` - OBD-II protocol and PID database
- `vehicle_data.c` - Vehicle data management
- `can_manager.c` - High-level CAN manager

## API Reference

### can_manager.h
```c
esp_err_t can_manager_init(void);
esp_err_t can_manager_start(void);
esp_err_t can_manager_stop(void);
bool can_manager_is_running(void);
```

### vehicle_data.h
```c
vehicle_data_t* vehicle_data_get(void);
void vehicle_data_update(vehicle_data_t *data, uint8_t pid, float value);
int vehicle_data_to_string(char *buffer, size_t size);
```

### obd2_pids.h
```c
esp_err_t obd2_request_pid(uint8_t pid);
esp_err_t obd2_parse_response(const uint8_t *data, uint8_t *pid, float *value);
const char* obd2_get_pid_name(uint8_t pid);
float obd2_decode_pid(uint8_t pid, const uint8_t *data);
```

## Support

For issues or questions, check:
1. CAN_DRIVER_README.md - Detailed documentation
2. Component header files - API documentation
3. ESP-IDF TWAI documentation
4. OBD-II standard documentation
