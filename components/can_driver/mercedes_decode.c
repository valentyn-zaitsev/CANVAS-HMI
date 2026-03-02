#include "mercedes_decode.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static mercedes_data_t mb_data;

void mercedes_decode_init(void) {
    memset(&mb_data, 0, sizeof(mb_data));
}

void mercedes_decode_message(uint32_t id, const uint8_t *data, uint8_t dlc) {
    switch (id) {
        case MB_ID_GAS_PEDAL:  // 0x105
            // ENGINE_RPM: bytes 0-1 big-endian, raw value (divide by 4 not needed per DBC)
            // From sniffer: 02 C7 = 711 decimal, car showed ~720 RPM
            // The DBC says start_bit=4, length=12, factor=1 — but empirical data
            // suggests bytes 0-1 as 16-bit big-endian gives close RPM
            if (dlc >= 5) {
                mb_data.engine_rpm = ((uint16_t)data[0] << 8) | data[1];
                mb_data.gas_pedal = data[4];
                mb_data.combined_gas = data[3];
            }
            break;

        case MB_ID_WHEEL_SPEEDS:  // 0x203
            // 4 wheel speeds, each 16-bit big-endian, scale 0.0375 mph
            if (dlc >= 8) {
                mb_data.wheel_speed_fl = ((uint16_t)data[0] << 8) | data[1];
                mb_data.wheel_speed_fr = ((uint16_t)data[2] << 8) | data[3];
                mb_data.wheel_speed_rl = ((uint16_t)data[4] << 8) | data[5];
                mb_data.wheel_speed_rr = ((uint16_t)data[6] << 8) | data[7];
                // Average wheel speed, convert to km/h
                // raw * 0.0375 mph * 1.60934 = raw * 0.0603 km/h
                // Use integer math: raw * 603 / 10000
                uint32_t avg_raw = ((uint32_t)mb_data.wheel_speed_fl +
                                    mb_data.wheel_speed_fr +
                                    mb_data.wheel_speed_rl +
                                    mb_data.wheel_speed_rr) / 4;
                mb_data.vehicle_speed_kmh = (uint8_t)(avg_raw * 603 / 10000);
            }
            break;

        case MB_ID_STEER_SENSOR:  // 0x003
            // STEER_ANGLE: bytes 0-1 big-endian, signed, scale 0.5 degrees
            // STEER_RATE: bytes 2-3 big-endian, signed
            if (dlc >= 4) {
                mb_data.steering_angle = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
                mb_data.steering_rate = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
            }
            break;

        case MB_ID_BRAKE_MODULE:  // 0x005
            // BRAKE_PRESSED: bit 0 of byte 0
            // BRAKE_POSITION: bytes 2-3 big-endian (10-bit)
            if (dlc >= 4) {
                mb_data.brake_pressed = data[0] & 0x01;
                mb_data.brake_position = ((uint16_t)(data[2] & 0x03) << 8) | data[3];
            }
            break;

        case MB_ID_GEAR_PACKET:  // 0x073
            // GEAR: lower nibble of byte 0
            if (dlc >= 1) {
                mb_data.gear = data[0] & 0x0F;
            }
            break;

        case MB_ID_DOOR_SENSORS:  // 0x283
            // Door states as bitmask in byte 0
            if (dlc >= 1) {
                mb_data.doors_open = data[0] & 0x0F;
            }
            break;

        default:
            return;  // Unknown ID, don't update stats
    }

    mb_data.decode_count++;
    mb_data.last_decode_tick = xTaskGetTickCount();
}

const mercedes_data_t *mercedes_decode_get_data(void) {
    return &mb_data;
}
