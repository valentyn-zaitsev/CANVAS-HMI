# Mercedes CAN Dashboard

ESP32-based CAN bus dashboard for Mercedes-Benz vehicles (tested on CLS400 W218). Displays real-time vehicle data on a 3.5" touchscreen with charts and data logging.

## Hardware

- **MCU**: ESP32 (CYD 3.5" board)
- **Display**: ST7796 480x320 TFT (SPI2)
- **Touch**: GT911 capacitive (I2C)
- **CAN**: SN65HVD230 transceiver, TX=GPIO22, RX=GPIO4, 500kbps
- **SD Card** (optional): SPI3, CS=GPIO5

### Pin Map

| Function | GPIO |
|----------|------|
| LCD SCLK | 14 |
| LCD MOSI | 13 |
| LCD MISO | 12 |
| LCD CS | 15 |
| LCD DC | 2 |
| Backlight | 27 |
| Touch SCL | 32 |
| Touch SDA | 33 |
| Touch RST | 25 |
| Touch INT | 21 |
| CAN TX | 22 |
| CAN RX | 4 |
| SD MOSI | 23 |
| SD MISO | 19 |
| SD SCLK | 18 |
| SD CS | 5 |

## Screens

1. **PARAMETERS** - Scrollable list of 40 decoded CAN parameters with live values and raw hex
2. **LOG FILES** - File selector for recorded driving sessions
3. **TEMPERATURES** - Oil, coolant, transmission, ambient temps chart with range slider
4. **SPEED / RPM** - Engine RPM, turbine RPM, vehicle speed (dual Y-axis)
5. **DYNAMICS** - Lateral G-force and yaw rate
6. **SUSPENSION** - AIRMATIC air suspension levels (FL/FR/RL/RR)

Screens are built lazily on first navigation to save LVGL memory.

## Features

- Real-time CAN decoding using Mercedes DBC definitions (opendbc)
- Touch crosshair on charts with value readout
- Range slider on temperature chart for zooming timeline
- Manual date/time setting (tap clock in status bar)
- SD card CAN data logging with 5-minute minimum session filter
- Status bar showing CAN state, message count, and clock

## Build

Requires ESP-IDF v5.2.

```bash
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

## CAN Configuration

- Mode: `TWAI_MODE_NO_ACK` (passive listener, no ACK on bus)
- Baud: 500 kbps
- RX queue: 32 frames
- No acceptance filter (receives all IDs)

## Project Structure

```
main/main.c                          - UI, dashboard, app logic
components/can_driver/               - CAN bus driver, sniffer, Mercedes decoder
components/sd_logger/                - SD card FATFS logging
components/ble_time_sync/            - BLE time sync (disabled, breaks touch I2C)
components/espressif__esp_lvgl_port/ - LVGL display/touch port
```
