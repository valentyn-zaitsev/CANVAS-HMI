#include "mercedes_decode.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static mercedes_data_t mb_data;

void mercedes_decode_init(void) {
    memset(&mb_data, 0, sizeof(mb_data));
}

void mercedes_decode_message(uint32_t id, const uint8_t *data, uint8_t dlc) {
    switch (id) {

        // ================================================================
        // OpenDBC signals (tested on real car via OBD port)
        // ================================================================

        case MB_ID_GAS_PEDAL:  // 0x105
            if (dlc >= 5) {
                mb_data.engine_rpm = ((uint16_t)data[0] << 8) | data[1];
                mb_data.gas_pedal = data[4];
                mb_data.combined_gas = data[3];
            }
            break;

        case MB_ID_WHEEL_SPEEDS:  // 0x203
            if (dlc >= 8) {
                mb_data.wheel_speed_fl = ((uint16_t)(data[0] & 0x07) << 8) | data[1];
                mb_data.wheel_speed_fr = ((uint16_t)(data[2] & 0x07) << 8) | data[3];
                mb_data.wheel_speed_rl = ((uint16_t)(data[4] & 0x07) << 8) | data[5];
                mb_data.wheel_speed_rr = ((uint16_t)(data[6] & 0x07) << 8) | data[7];
                uint32_t avg_raw = ((uint32_t)mb_data.wheel_speed_fl +
                                    mb_data.wheel_speed_fr +
                                    mb_data.wheel_speed_rl +
                                    mb_data.wheel_speed_rr) / 4;
                mb_data.vehicle_speed_kmh = (uint8_t)(avg_raw * 603 / 10000);
                mb_data.wheel_moving_fl = (data[0] >> 6) & 0x01;
                mb_data.wheel_moving_fr = (data[2] >> 6) & 0x01;
                mb_data.wheel_moving_rl = (data[4] >> 6) & 0x01;
                mb_data.wheel_moving_rr = (data[6] >> 6) & 0x01;
            }
            break;

        case MB_ID_STEER_SENSOR:  // 0x003
            if (dlc >= 5) {
                int16_t raw_angle = (int16_t)(((data[0] & 0x0F) << 8) | data[1]);
                if (raw_angle & 0x800) raw_angle |= (int16_t)0xF000;
                mb_data.steering_angle = -raw_angle;
                int16_t raw_rate = (int16_t)(((data[2] & 0x0F) << 8) | data[3]);
                if (raw_rate & 0x800) raw_rate |= (int16_t)0xF000;
                mb_data.steering_rate = raw_rate;
                mb_data.steer_direction = (data[0] >> 4) & 0x01;
            }
            break;

        case MB_ID_BRAKE_MODULE:  // 0x005
            if (dlc >= 4) {
                mb_data.brake_pressed = data[0] & 0x01;
                mb_data.brake_position = ((uint16_t)(data[2] & 0x03) << 8) | data[3];
            }
            break;

        case MB_ID_STEER_TORQUE:  // 0x00E
            if (dlc >= 2) {
                mb_data.steering_torque = data[1];
            }
            break;

        case MB_ID_DRIVER_CTRL:  // 0x045
            if (dlc >= 3) {
                mb_data.left_blinker = (data[2] >> 0) & 0x01;
                mb_data.right_blinker = (data[2] >> 1) & 0x01;
                mb_data.highbeam_toggle = (data[2] >> 2) & 0x01;
                mb_data.highbeam_momentary = (data[2] >> 3) & 0x01;
                mb_data.cruise_cancel = (data[0] >> 0) & 0x01;
                mb_data.cruise_resume = (data[0] >> 1) & 0x01;
                mb_data.cruise_accel_high = (data[0] >> 2) & 0x01;
                mb_data.cruise_decel_high = (data[0] >> 3) & 0x01;
                mb_data.cruise_accel_low = (data[0] >> 4) & 0x01;
                mb_data.cruise_decel_low = (data[0] >> 5) & 0x01;
            }
            break;

        case MB_ID_GEAR_LEVER:  // 0x06D
            if (dlc >= 2) {
                mb_data.gear_lever_reverse = (data[1] >> 0) & 0x01;
                mb_data.gear_lever_neutral_up = (data[1] >> 1) & 0x01;
                mb_data.gear_lever_neutral_down = (data[1] >> 2) & 0x01;
                mb_data.gear_lever_drive = (data[1] >> 3) & 0x01;
                mb_data.gear_lever_park = (data[1] >> 4) & 0x01;
            }
            break;

        case MB_ID_GEAR_PACKET:  // 0x073
            if (dlc >= 1) {
                mb_data.gear = data[0] & 0x0F;
            }
            break;

        case MB_ID_DOOR_SENSORS:  // 0x283
            if (dlc >= 4) {
                mb_data.doors_open = data[0];
                mb_data.door_open_fl = (data[0] >> 1) & 0x01;
                mb_data.door_open_fr = (data[0] >> 3) & 0x01;
                mb_data.door_open_rl = (data[0] >> 5) & 0x01;
                mb_data.door_open_rr = (data[0] >> 7) & 0x01;
                mb_data.brake_pressed_2 = (data[3] >> 3) & 0x01;
            }
            break;

        case MB_ID_SEATBELT:  // 0x375
            if (dlc >= 3) {
                mb_data.seatbelt_driver = (data[2] >> 0) & 0x01;
                mb_data.seatbelt_passenger = (data[2] >> 2) & 0x01;
            }
            break;

        case MB_ID_IGNITION:  // 0x245
            if (dlc >= 1) {
                mb_data.ignition_raw = data[0];
            }
            break;

        case MB_ID_WHEEL_ENC:  // 0x201
            if (dlc >= 4) {
                mb_data.wheel_enc_1 = data[0];
                mb_data.wheel_enc_2 = data[1];
                mb_data.wheel_enc_3 = data[2];
                mb_data.wheel_enc_4 = data[3];
            }
            break;

        case MB_ID_CRUISE_CTRL3:  // 0x378
            if (dlc >= 5) {
                mb_data.cruise_set_speed = data[1];
                mb_data.cruise_enabled = (data[4] >> 2) & 0x01;
                mb_data.cruise_disabled = (data[4] >> 4) & 0x01;
            }
            break;

        // ================================================================
        // CAN C Bus signals (Xentry DAT verified — direct bus tap)
        // ================================================================

        case MB_ID_ENGINE_MAIN:  // 0x0308 — MS_308h
            // NMOT: bits 8-23, factor 0.25
            // T_OEL: byte 5, offset -40
            // OEL_FS: byte 6, OEL_QUAL: byte 7
            // UEHITZ: bit 32, TEMP_KL: bit 39, OEL_KL: bit 29, DIAG_KL: bit 30
            if (dlc >= 8) {
                mb_data.nmot_rpm_raw = ((uint16_t)data[1] << 8) | data[2];
                mb_data.oil_temp_c = (int8_t)(data[5] - 40);
                mb_data.oil_level = data[6];
                mb_data.oil_quality = data[7];
                mb_data.oil_overheat = (data[4] >> 0) & 0x01;   // bit 32
                mb_data.coolant_overheat = (data[4] >> 7) & 0x01; // bit 39
                mb_data.oil_warning = (data[3] >> 5) & 0x01;    // bit 29
                mb_data.mil_lamp = (data[3] >> 6) & 0x01;       // bit 30
            }
            break;

        case MB_ID_COOLANT_TEMP:  // 0x0608 — MS_608h
            // T_MOT: byte 0, offset -40
            if (dlc >= 1) {
                mb_data.coolant_temp_c = (int8_t)(data[0] - 40);
            }
            break;

        case MB_ID_TRANS_STATUS:  // 0x0418 — GS_418h
            // FSC: byte 0 (gear: 0=P,1=R,2=N,3=D)
            // FPC: byte 1 (driving program)
            // T_GET: byte 2, offset -40
            if (dlc >= 3) {
                mb_data.gear_fsc = data[0];
                mb_data.drive_program = data[1];
                mb_data.trans_oil_temp_c = (int8_t)(data[2] - 40);
            }
            break;

        case MB_ID_TRANS_SPEEDS:  // 0x0338 — GS_338h
            // NAB: bytes 0-1, factor 0.25 RPM (0xFFFF = N/A)
            // NTURBINE: bytes 6-7, factor 0.25 RPM
            if (dlc >= 8) {
                mb_data.trans_output_raw = ((uint16_t)data[0] << 8) | data[1];
                mb_data.turbine_speed_raw = ((uint16_t)data[6] << 8) | data[7];
            }
            break;

        case MB_ID_FUEL_DATA:  // 0x0320 — GW_C_B9
            // VB: bytes 0-1 (µl/250ms)
            // TANK_FS: byte 2 (liters)
            if (dlc >= 3) {
                mb_data.fuel_consumption = ((uint16_t)data[0] << 8) | data[1];
                mb_data.tank_level = data[2];
            }
            break;

        case MB_ID_INST_CLUSTER:  // 0x03F0 — KOMBI_3F0h
            // T_AUSSEN: byte 2, factor 0.5, offset -40
            if (dlc >= 3) {
                mb_data.ambient_temp_raw = data[2];
            }
            break;

        case MB_ID_DYNAMICS:  // 0x0224 — RDU_A2
            // AY_S: byte 0, factor 0.01 g (signed)
            // GIER_ROH: bytes 1-2, factor 0.005 °/s (signed)
            // LRW: bits 26-39 (14 bits), factor 0.1 °
            if (dlc >= 5) {
                mb_data.lateral_g_raw = (int8_t)data[0];
                mb_data.yaw_rate_raw = (int16_t)(((uint16_t)data[1] << 8) | data[2]);
                // LRW: bits 26..39 = byte3[1:0] << 12 | byte4 << 4 | ... complex
                // Simpler: bytes 3-4, mask 14 bits
                mb_data.steer_angle_rdu = (((uint16_t)(data[3] & 0x3F) << 8) | data[4]);
            }
            break;

        case MB_ID_WHEEL_SPD_RDU:  // 0x0228 — RDU_A3
            // Each wheel: 2-bit direction + 14-bit speed (×0.01 km/h)
            if (dlc >= 8) {
                mb_data.ws_dir_fl = (data[0] >> 6) & 0x03;
                mb_data.ws_fl_rdu = ((uint16_t)(data[0] & 0x3F) << 8) | data[1];
                mb_data.ws_dir_fr = (data[2] >> 6) & 0x03;
                mb_data.ws_fr_rdu = ((uint16_t)(data[2] & 0x3F) << 8) | data[3];
                mb_data.ws_dir_rl = (data[4] >> 6) & 0x03;
                mb_data.ws_rl_rdu = ((uint16_t)(data[4] & 0x3F) << 8) | data[5];
                mb_data.ws_dir_rr = (data[6] >> 6) & 0x03;
                mb_data.ws_rr_rdu = ((uint16_t)(data[6] & 0x3F) << 8) | data[7];
            }
            break;

        case MB_ID_ESP_STATUS:  // 0x0200 — RDU_A1
            // BLS: bit 2, ESP_LAMP: bit 8, ABS_LAMP: bit 9, HAS_KL: bit 14
            if (dlc >= 2) {
                mb_data.brake_light = (data[0] >> 2) & 0x01;
                mb_data.esp_lamp = (data[1] >> 0) & 0x01;
                mb_data.abs_lamp = (data[1] >> 1) & 0x01;
                mb_data.handbrake = (data[1] >> 6) & 0x01;
            }
            break;

        case MB_ID_AIRMATIC:  // 0x0340 — FS_340h
            // FZGN_VL: byte 2, VR: byte 3, HL: byte 4, HR: byte 5
            if (dlc >= 6) {
                mb_data.level_fl = data[2];
                mb_data.level_fr = data[3];
                mb_data.level_rl = data[4];
                mb_data.level_rr = data[5];
            }
            break;

        case MB_ID_DRIVING_STYLE:  // 0x0580 — AAD_580h
            // FTK_BMI: byte 0, FTK_LMI: byte 1, FTK_VMI: byte 2
            if (dlc >= 3) {
                mb_data.style_accel = data[0];
                mb_data.style_lateral = data[1];
                mb_data.style_braking = data[2];
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
