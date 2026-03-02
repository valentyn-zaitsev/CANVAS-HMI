#ifndef MERCEDES_DECODE_H
#define MERCEDES_DECODE_H

#include <stdint.h>
#include "vehicle_data.h"

#ifdef __cplusplus
extern "C" {
#endif

// Mercedes W212/W218 broadcast CAN IDs (from opendbc mercedes_benz_e350_2010.dbc)
#define MB_ID_STEER_SENSOR   0x003  // Steering angle, rate
#define MB_ID_BRAKE_MODULE   0x005  // Brake pressed, position
#define MB_ID_STEER_TORQUE   0x00E  // Steering torque
#define MB_ID_DRIVER_CTRL    0x045  // Blinkers, highbeam, cruise buttons
#define MB_ID_GEAR_LEVER     0x06D  // P/R/N/D
#define MB_ID_GEAR_PACKET    0x073  // Current gear
#define MB_ID_GAS_PEDAL      0x105  // Engine RPM, gas pedal, combined gas
#define MB_ID_WHEEL_ENC      0x201  // Wheel encoder counters
#define MB_ID_WHEEL_SPEEDS   0x203  // 4 wheel speeds
#define MB_ID_IGNITION       0x245  // Ignition state
#define MB_ID_CRUISE_CTRL    0x101  // Cruise control
#define MB_ID_DOOR_SENSORS   0x283  // Door open/closed
#define MB_ID_CRUISE_CTRL3   0x378  // Cruise set speed

// Decoded broadcast data
typedef struct {
    // From GAS_PEDAL (0x105)
    uint16_t engine_rpm;
    uint8_t gas_pedal;
    uint8_t combined_gas;

    // From WHEEL_SPEEDS (0x203)
    uint16_t wheel_speed_fl;  // in 0.0375 mph units
    uint16_t wheel_speed_fr;
    uint16_t wheel_speed_rl;
    uint16_t wheel_speed_rr;
    uint8_t vehicle_speed_kmh;

    // From STEER_SENSOR (0x003)
    int16_t steering_angle;   // in 0.5 degree units
    int16_t steering_rate;

    // From BRAKE_MODULE (0x005)
    uint8_t brake_pressed;
    uint16_t brake_position;

    // From GEAR_PACKET (0x073)
    uint8_t gear;

    // From DOOR_SENSORS (0x283)
    uint8_t doors_open;       // bitmask: bit0=FL, bit1=FR, bit2=RL, bit3=RR

    // Stats
    uint32_t decode_count;
    uint32_t last_decode_tick;
} mercedes_data_t;

void mercedes_decode_init(void);
void mercedes_decode_message(uint32_t id, const uint8_t *data, uint8_t dlc);
const mercedes_data_t *mercedes_decode_get_data(void);

#ifdef __cplusplus
}
#endif

#endif // MERCEDES_DECODE_H
