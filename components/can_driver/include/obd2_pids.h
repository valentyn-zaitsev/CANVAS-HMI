#ifndef OBD2_PIDS_H
#define OBD2_PIDS_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * OBD-II PID Definitions
 * Standard PIDs (Mode 01) for vehicle diagnostics
 */

// OBD-II Service modes
#define OBD2_SERVICE_CURRENT_DATA       0x01
#define OBD2_SERVICE_FREEZE_FRAME       0x02
#define OBD2_SERVICE_DTC                0x03
#define OBD2_SERVICE_CLEAR_DTC          0x04
#define OBD2_SERVICE_O2_SENSOR_DATA     0x05
#define OBD2_SERVICE_SUPPORTED_PIDS     0x00

// Standard OBD-II PIDs (Mode 01)
#define PID_SUPPORTED_PIDS_01_20        0x00
#define PID_MONITOR_STATUS              0x01
#define PID_DTC_FREEZE_FRAME            0x02
#define PID_FUEL_SYSTEM_STATUS          0x03
#define PID_ENGINE_LOAD                 0x04
#define PID_ENGINE_COOLANT_TEMP         0x05
#define PID_SHORT_TERM_FUEL_TRIM_B1     0x06
#define PID_LONG_TERM_FUEL_TRIM_B1      0x07
#define PID_SHORT_TERM_FUEL_TRIM_B2     0x08
#define PID_LONG_TERM_FUEL_TRIM_B2      0x09
#define PID_FUEL_PRESSURE               0x0A
#define PID_INTAKE_MANIFOLD_PRESSURE    0x0B
#define PID_ENGINE_RPM                  0x0C
#define PID_VEHICLE_SPEED               0x0D
#define PID_TIMING_ADVANCE              0x0E
#define PID_INTAKE_AIR_TEMP             0x0F
#define PID_MAF_AIRFLOW                 0x10
#define PID_THROTTLE_POSITION           0x11
#define PID_SECONDARY_AIR_STATUS        0x12
#define PID_O2_SENSOR_PRESENT           0x13
#define PID_O2_SENSOR_1_B1              0x14
#define PID_O2_SENSOR_2_B1              0x15
#define PID_O2_SENSOR_1_B2              0x16
#define PID_O2_SENSOR_2_B2              0x17
#define PID_OBD_STANDARDS               0x1C
#define PID_RUNTIME_SINCE_START         0x1F
#define PID_DISTANCE_WITH_MIL_ON        0x21
#define PID_FUEL_RAIL_PRESSURE          0x22
#define PID_FUEL_RAIL_PRESSURE_DIESEL   0x23
#define PID_O2_SENSOR_1_B1_VOLTAGE      0x24
#define PID_O2_SENSOR_2_B1_VOLTAGE      0x25
#define PID_O2_SENSOR_1_B2_VOLTAGE      0x26
#define PID_O2_SENSOR_2_B2_VOLTAGE      0x27
#define PID_COMMANDED_EGR               0x2C
#define PID_EGR_ERROR                   0x2D
#define PID_FUEL_LEVEL                  0x2F
#define PID_EVAP_SYSTEM_VAPOR_PRESSURE  0x32
#define PID_ABSOLUTE_BAROMETRIC_PRESSURE 0x33
#define PID_O2_SENSOR_1_B1_CURRENT      0x34
#define PID_O2_SENSOR_2_B1_CURRENT      0x35
#define PID_O2_SENSOR_1_B2_CURRENT      0x36
#define PID_O2_SENSOR_2_B2_CURRENT      0x37
#define PID_CATALYST_TEMP_B1_S1         0x3C
#define PID_CATALYST_TEMP_B2_S1         0x3D
#define PID_CONTROL_MODULE_VOLTAGE      0x42
#define PID_ABSOLUTE_LOAD_VALUE         0x43
#define PID_FUEL_AIR_COMMANDED_EQUIV    0x44
#define PID_RELATIVE_THROTTLE_POS       0x45
#define PID_AMBIENT_AIR_TEMP            0x46
#define PID_ABSOLUTE_THROTTLE_POS_B     0x47
#define PID_ABSOLUTE_THROTTLE_POS_C     0x48
#define PID_ACCELERATOR_PEDAL_POS_D     0x49
#define PID_ACCELERATOR_PEDAL_POS_E     0x4A
#define PID_ACCELERATOR_PEDAL_POS_F     0x4B
#define PID_COMMANDED_THROTTLE_ACTUATOR 0x4C
#define PID_TIME_RUN_WITH_MIL_ON        0x4D
#define PID_TIME_SINCE_DTC_CLEARED      0x4E
#define PID_FUEL_TYPE                   0x51
#define PID_ETHANOL_FUEL_PERCENT        0x52
#define PID_ABSOLUTE_EVAP_SYSTEM_VAPOR_PRESSURE 0x53
#define PID_SHORT_TERM_SECONDARY_O2_TRIM_B1 0x55
#define PID_LONG_TERM_SECONDARY_O2_TRIM_B1  0x56
#define PID_SHORT_TERM_SECONDARY_O2_TRIM_B2 0x57
#define PID_LONG_TERM_SECONDARY_O2_TRIM_B2  0x58
#define PID_FUEL_RAIL_ABSOLUTE_PRESSURE 0x59
#define PID_RELATIVE_ACCELERATOR_PEDAL_POS 0x5A
#define PID_HYBRID_BATTERY_PACK_REMAINING_LIFE 0x5B
#define PID_ENGINE_OIL_TEMP             0x5C
#define PID_FUEL_INJECTION_TIMING       0x5D
#define PID_ENGINE_FUEL_RATE            0x5E
#define PID_EMISSION_STANDARDS_COMPLIANCE 0x65
#define PID_NOX_SENSOR_CORRECTED_DATA   0x72
#define PID_MANIFOLD_SURFACE_TEMP       0x73
#define PID_PAM_INTAKE_VALVE_LIFT       0x74
#define PID_TURBOCHARGER_COMPRESSOR_INLET_PRESSURE 0x75
#define PID_TURBOCHARGER_BOOST_CONTROL_PRESSURE 0x76

// CAN IDs for OBD-II communication
#define OBD2_REQUEST_CAN_ID             0x7DF  // Broadcast request
#define OBD2_RESPONSE_CAN_ID_BASE       0x7E8  // Response from ECU (0x7E8 - 0x7EF)

// PID Data structure
typedef struct {
    uint8_t pid;
    const char *name;
    const char *unit;
    uint8_t num_bytes;
    float (*decode_func)(const uint8_t *data);
} obd2_pid_t;

/**
 * Initialize OBD-II PID library
 * @return ESP_OK on success
 */
esp_err_t obd2_init(void);

/**
 * Request a PID from the vehicle
 * @param pid PID to request
 * @return ESP_OK on success
 */
esp_err_t obd2_request_pid(uint8_t pid);

/**
 * Parse OBD-II response
 * @param data CAN data (8 bytes)
 * @param pid Output PID that was responded
 * @param value Output decoded value
 * @return ESP_OK on success
 */
esp_err_t obd2_parse_response(const uint8_t *data, uint8_t *pid, float *value);

/**
 * Get PID information
 * @param pid PID to lookup
 * @return Pointer to PID info, or NULL if not found
 */
const obd2_pid_t* obd2_get_pid_info(uint8_t pid);

/**
 * Get PID name
 * @param pid PID to lookup
 * @return PID name string
 */
const char* obd2_get_pid_name(uint8_t pid);

/**
 * Get PID unit
 * @param pid PID to lookup
 * @return PID unit string
 */
const char* obd2_get_pid_unit(uint8_t pid);

/**
 * Decode PID value
 * @param pid PID to decode
 * @param data Raw CAN data (2 bytes for most PIDs)
 * @return Decoded value
 */
float obd2_decode_pid(uint8_t pid, const uint8_t *data);

/**
 * Request supported PIDs
 * @return ESP_OK on success
 */
esp_err_t obd2_request_supported_pids(void);

/**
 * Clear Diagnostic Trouble Codes
 * @return ESP_OK on success
 */
esp_err_t obd2_clear_dtc(void);

/**
 * Read Diagnostic Trouble Codes
 * @return ESP_OK on success
 */
esp_err_t obd2_read_dtc(void);

#ifdef __cplusplus
}
#endif

#endif // OBD2_PIDS_H
