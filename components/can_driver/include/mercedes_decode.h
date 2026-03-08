#ifndef MERCEDES_DECODE_H
#define MERCEDES_DECODE_H

#include <stdint.h>
#include "vehicle_data.h"

#ifdef __cplusplus
extern "C" {
#endif

// === OpenDBC CAN IDs (from mercedes_benz_e350_2010.dbc) ===
#define MB_ID_STEER_SENSOR   0x003  // Steering angle, rate, direction
#define MB_ID_BRAKE_MODULE   0x005  // Brake pressed, position
#define MB_ID_STEER_TORQUE   0x00E  // Steering torque
#define MB_ID_DRIVER_CTRL    0x045  // Blinkers, highbeam, cruise buttons
#define MB_ID_GEAR_LEVER     0x06D  // P/R/N/D lever
#define MB_ID_GEAR_PACKET    0x073  // Current gear (opendbc)
#define MB_ID_CRUISE_CTRL    0x101  // Cruise control
#define MB_ID_GAS_PEDAL      0x105  // Engine RPM, gas pedal, combined gas
#define MB_ID_WHEEL_ENC      0x201  // Wheel encoder counters
#define MB_ID_WHEEL_SPEEDS   0x203  // 4 wheel speeds + moving flags
#define MB_ID_IGNITION       0x245  // Ignition state
#define MB_ID_DOOR_SENSORS   0x283  // Door open/closed
#define MB_ID_SEATBELT       0x375  // Seatbelt latched
#define MB_ID_CRUISE_CTRL3   0x378  // Cruise set speed, enabled/disabled

// === CAN C Bus IDs (from Xentry DAT / rnd-ash / MHH AUTO — verified) ===
#define MB_ID_ESP_STATUS     0x200  // RDU_A1: ESP status, brake light, handbrake
#define MB_ID_ENGINE_TRANS    0x208  // MS_208h: Engine-to-transmission
#define MB_ID_TRANS_MAIN     0x218  // GS_218h: Transmission main
#define MB_ID_DYNAMICS       0x224  // RDU_A2: Lateral G, yaw rate, steering
#define MB_ID_WHEEL_SPD_RDU  0x228  // RDU_A3: Wheel speeds (0.01 km/h)
#define MB_ID_STEER_ANGLE    0x236  // RDU_A4: Steering angle dedicated
#define MB_ID_ENGINE_MAIN    0x308  // MS_308h: RPM, oil temp, oil level
#define MB_ID_ENGINE_TORQUE  0x312  // MS_312h: Engine torque
#define MB_ID_FUEL_DATA      0x320  // GW_C_B9: Fuel consumption, tank level
#define MB_ID_TRANS_SPEEDS   0x338  // GS_338h: Trans output + turbine speed
#define MB_ID_AIRMATIC       0x340  // FS_340h: Suspension levels
#define MB_ID_INST_CLUSTER   0x3F0  // KOMBI_3F0h: Ambient temp
#define MB_ID_TRANS_STATUS   0x418  // GS_418h: Gear, program, trans temp
#define MB_ID_DRIVING_STYLE  0x580  // AAD_580h: Driving style
#define MB_ID_COOLANT_TEMP   0x608  // MS_608h: Coolant temperature

// Decoded broadcast data
typedef struct {
    // === From OpenDBC signals ===
    // From GAS_PEDAL (0x105)
    uint16_t engine_rpm;        // empirical 16-bit BE from 0x105
    uint8_t gas_pedal;
    uint8_t combined_gas;

    // From WHEEL_SPEEDS (0x203) — opendbc: 0.0375 mph units
    uint16_t wheel_speed_fl;
    uint16_t wheel_speed_fr;
    uint16_t wheel_speed_rl;
    uint16_t wheel_speed_rr;
    uint8_t vehicle_speed_kmh;
    uint8_t wheel_moving_fl;
    uint8_t wheel_moving_fr;
    uint8_t wheel_moving_rl;
    uint8_t wheel_moving_rr;

    // From STEER_SENSOR (0x003)
    int16_t steering_angle;     // in 0.5 degree units
    int16_t steering_rate;
    uint8_t steer_direction;

    // From STEER_TORQUE (0x00E)
    uint8_t steering_torque;

    // From BRAKE_MODULE (0x005)
    uint8_t brake_pressed;
    uint16_t brake_position;

    // From GEAR_PACKET (0x073)
    uint8_t gear;

    // From GEAR_LEVER (0x06D)
    uint8_t gear_lever_park;
    uint8_t gear_lever_reverse;
    uint8_t gear_lever_neutral_up;
    uint8_t gear_lever_neutral_down;
    uint8_t gear_lever_drive;

    // From DRIVER_CONTROLS (0x045)
    uint8_t left_blinker;
    uint8_t right_blinker;
    uint8_t highbeam_toggle;
    uint8_t highbeam_momentary;
    uint8_t cruise_cancel;
    uint8_t cruise_resume;
    uint8_t cruise_accel_high;
    uint8_t cruise_decel_high;
    uint8_t cruise_accel_low;
    uint8_t cruise_decel_low;

    // From CRUISE_CTRL3 (0x378)
    uint8_t cruise_set_speed;
    uint8_t cruise_enabled;
    uint8_t cruise_disabled;

    // From DOOR_SENSORS (0x283)
    uint8_t doors_open;
    uint8_t door_open_fl;
    uint8_t door_open_fr;
    uint8_t door_open_rl;
    uint8_t door_open_rr;
    uint8_t brake_pressed_2;

    // From SEATBELT (0x375)
    uint8_t seatbelt_driver;
    uint8_t seatbelt_passenger;

    // From IGNITION (0x245)
    uint8_t ignition_raw;

    // From WHEEL_ENC (0x201)
    uint8_t wheel_enc_1;
    uint8_t wheel_enc_2;
    uint8_t wheel_enc_3;
    uint8_t wheel_enc_4;

    // === From CAN C Bus (Xentry DAT verified) ===
    // From ENGINE_MAIN (0x0308)
    uint16_t nmot_rpm_raw;      // raw 16-bit, multiply by 0.25 for RPM
    int8_t oil_temp_c;          // byte5 - 40
    uint8_t oil_level;          // byte6
    uint8_t oil_quality;        // byte7
    uint8_t oil_overheat;       // UEHITZ
    uint8_t coolant_overheat;   // TEMP_KL
    uint8_t oil_warning;        // OEL_KL
    uint8_t mil_lamp;           // DIAG_KL (check engine)

    // From COOLANT_TEMP (0x0608)
    int8_t coolant_temp_c;      // byte0 - 40

    // From TRANS_STATUS (0x0418)
    uint8_t gear_fsc;           // FSC: 0=P,1=R,2=N,3=D,4-7=manual
    uint8_t drive_program;      // FPC: driving program C/S/M
    int8_t trans_oil_temp_c;    // byte2 - 40

    // From TRANS_SPEEDS (0x0338)
    uint16_t trans_output_raw;  // NAB: ×0.25 RPM
    uint16_t turbine_speed_raw; // NTURBINE: ×0.25 RPM

    // From FUEL_DATA (0x0320)
    uint16_t fuel_consumption;  // µl/250ms
    uint8_t tank_level;         // liters

    // From INST_CLUSTER (0x03F0)
    uint8_t ambient_temp_raw;   // ×0.5 - 40 °C

    // From DYNAMICS (0x0224)
    int8_t lateral_g_raw;       // ×0.01 g
    int16_t yaw_rate_raw;       // ×0.005 °/s
    uint16_t steer_angle_rdu;   // LRW: ×0.1 °

    // From WHEEL_SPD_RDU (0x0228)
    uint16_t ws_fl_rdu;         // ×0.01 km/h
    uint16_t ws_fr_rdu;
    uint16_t ws_rl_rdu;
    uint16_t ws_rr_rdu;
    uint8_t ws_dir_fl;          // 0=stop,1=fwd,2=back
    uint8_t ws_dir_fr;
    uint8_t ws_dir_rl;
    uint8_t ws_dir_rr;

    // From ESP_STATUS (0x0200)
    uint8_t brake_light;        // BLS
    uint8_t esp_lamp;
    uint8_t abs_lamp;
    uint8_t handbrake;          // HAS_KL

    // From AIRMATIC (0x0340)
    uint8_t level_fl;
    uint8_t level_fr;
    uint8_t level_rl;
    uint8_t level_rr;

    // From DRIVING_STYLE (0x0580)
    uint8_t style_accel;        // FTK_BMI >100 = dynamic
    uint8_t style_lateral;      // FTK_LMI
    uint8_t style_braking;      // FTK_VMI

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
