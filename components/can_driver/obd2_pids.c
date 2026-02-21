#include "obd2_pids.h"
#include "can_driver.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "OBD2_PIDS";

// Forward declarations for decode functions
static float decode_rpm(const uint8_t *data);
static float decode_speed(const uint8_t *data);
static float decode_temp(const uint8_t *data);
static float decode_load(const uint8_t *data);
static float decode_fuel_pressure(const uint8_t *data);
static float decode_throttle(const uint8_t *data);
static float decode_o2_voltage(const uint8_t *data);
static float decode_fuel_trim(const uint8_t *data);
static float decode_timing_advance(const uint8_t *data);
static float decode_maf(const uint8_t *data);
static float decode_fuel_level(const uint8_t *data);
static float decode_voltage(const uint8_t *data);
static float decode_generic_byte(const uint8_t *data);
static float decode_generic_word(const uint8_t *data);

// OBD-II PID Database
static const obd2_pid_t obd2_pids[] = {
    // Engine parameters
    {PID_ENGINE_RPM, "Engine RPM", "rpm", 2, decode_rpm},
    {PID_ENGINE_LOAD, "Engine Load", "%", 1, decode_load},
    {PID_ENGINE_COOLANT_TEMP, "Coolant Temperature", "°C", 1, decode_temp},
    {PID_INTAKE_AIR_TEMP, "Intake Air Temperature", "°C", 1, decode_temp},
    {PID_THROTTLE_POSITION, "Throttle Position", "%", 1, decode_throttle},
    
    // Fuel system
    {PID_FUEL_PRESSURE, "Fuel Pressure", "kPa", 1, decode_fuel_pressure},
    {PID_SHORT_TERM_FUEL_TRIM_B1, "Short Term Fuel Trim B1", "%", 1, decode_fuel_trim},
    {PID_LONG_TERM_FUEL_TRIM_B1, "Long Term Fuel Trim B1", "%", 1, decode_fuel_trim},
    {PID_FUEL_LEVEL, "Fuel Level", "%", 1, decode_fuel_level},
    {PID_FUEL_INJECTION_TIMING, "Fuel Injection Timing", "°", 2, decode_timing_advance},
    {PID_ENGINE_FUEL_RATE, "Engine Fuel Rate", "L/h", 2, decode_generic_word},
    
    // Emissions
    {PID_O2_SENSOR_1_B1, "O2 Sensor 1 B1", "V", 2, decode_o2_voltage},
    {PID_O2_SENSOR_2_B1, "O2 Sensor 2 B1", "V", 2, decode_o2_voltage},
    {PID_O2_SENSOR_1_B2, "O2 Sensor 1 B2", "V", 2, decode_o2_voltage},
    {PID_O2_SENSOR_2_B2, "O2 Sensor 2 B2", "V", 2, decode_o2_voltage},
    
    // Vehicle speed and timing
    {PID_VEHICLE_SPEED, "Vehicle Speed", "km/h", 1, decode_speed},
    {PID_TIMING_ADVANCE, "Timing Advance", "°", 1, decode_timing_advance},
    
    // Electrical
    {PID_CONTROL_MODULE_VOLTAGE, "Control Module Voltage", "V", 2, decode_voltage},
    
    // Air flow
    {PID_MAF_AIRFLOW, "MAF Air Flow", "g/s", 2, decode_maf},
    {PID_INTAKE_MANIFOLD_PRESSURE, "Intake Manifold Pressure", "kPa", 1, decode_generic_byte},
    
    // Pressure
    {PID_FUEL_RAIL_PRESSURE, "Fuel Rail Pressure", "kPa", 2, decode_generic_word},
    {PID_FUEL_RAIL_PRESSURE_DIESEL, "Fuel Rail Pressure (Diesel)", "kPa", 2, decode_generic_word},
    
    // Temperature
    {PID_ENGINE_OIL_TEMP, "Engine Oil Temperature", "°C", 1, decode_temp},
    {PID_AMBIENT_AIR_TEMP, "Ambient Air Temperature", "°C", 1, decode_temp},
    
    // Terminator
    {0xFF, NULL, NULL, 0, NULL}
};

// Decode functions
static float decode_rpm(const uint8_t *data) {
    return ((data[0] * 256) + data[1]) / 4.0f;
}

static float decode_speed(const uint8_t *data) {
    return data[0];
}

static float decode_temp(const uint8_t *data) {
    return data[0] - 40;
}

static float decode_load(const uint8_t *data) {
    return (data[0] * 100) / 255.0f;
}

static float decode_fuel_pressure(const uint8_t *data) {
    return data[0] * 3;
}

static float decode_throttle(const uint8_t *data) {
    return (data[0] * 100) / 255.0f;
}

static float decode_o2_voltage(const uint8_t *data) {
    return (data[0] * 8) / 1000.0f;
}

static float decode_fuel_trim(const uint8_t *data) {
    return ((data[0] - 128) * 100) / 128.0f;
}

static float decode_timing_advance(const uint8_t *data) {
    return (data[0] / 2.0f) - 64;
}

static float decode_maf(const uint8_t *data) {
    return ((data[0] * 256) + data[1]) / 100.0f;
}

static float decode_fuel_level(const uint8_t *data) {
    return (data[0] * 100) / 255.0f;
}

static float decode_voltage(const uint8_t *data) {
    return ((data[0] * 256) + data[1]) / 1000.0f;
}

static float decode_generic_byte(const uint8_t *data) {
    return data[0];
}

static float decode_generic_word(const uint8_t *data) {
    return (data[0] * 256) + data[1];
}

esp_err_t obd2_init(void) {
    ESP_LOGI(TAG, "OBD-II PID library initialized");
    return ESP_OK;
}

esp_err_t obd2_request_pid(uint8_t pid) {
    can_message_t msg = {
        .identifier = OBD2_REQUEST_CAN_ID,
        .data_length_code = 8,
        .data = {0x02, OBD2_SERVICE_CURRENT_DATA, pid, 0x00, 0x00, 0x00, 0x00, 0x00}
    };
    
    return can_send_message(&msg);
}

esp_err_t obd2_parse_response(const uint8_t *data, uint8_t *pid, float *value) {
    if (data == NULL || pid == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check if this is a valid OBD-II response
    if (data[0] < 2 || data[1] != (OBD2_SERVICE_CURRENT_DATA + 0x40)) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    *pid = data[2];
    
    // Find PID in database and decode
    const obd2_pid_t *pid_info = obd2_get_pid_info(*pid);
    if (pid_info == NULL || pid_info->decode_func == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    
    *value = pid_info->decode_func(&data[3]);
    
    return ESP_OK;
}

const obd2_pid_t* obd2_get_pid_info(uint8_t pid) {
    for (int i = 0; obd2_pids[i].pid != 0xFF; i++) {
        if (obd2_pids[i].pid == pid) {
            return &obd2_pids[i];
        }
    }
    return NULL;
}

const char* obd2_get_pid_name(uint8_t pid) {
    const obd2_pid_t *info = obd2_get_pid_info(pid);
    return info ? info->name : "Unknown PID";
}

const char* obd2_get_pid_unit(uint8_t pid) {
    const obd2_pid_t *info = obd2_get_pid_info(pid);
    return info ? info->unit : "";
}

float obd2_decode_pid(uint8_t pid, const uint8_t *data) {
    const obd2_pid_t *info = obd2_get_pid_info(pid);
    if (info == NULL || info->decode_func == NULL) {
        return 0.0f;
    }
    return info->decode_func(data);
}

esp_err_t obd2_request_supported_pids(void) {
    return obd2_request_pid(PID_SUPPORTED_PIDS_01_20);
}

esp_err_t obd2_clear_dtc(void) {
    can_message_t msg = {
        .identifier = OBD2_REQUEST_CAN_ID,
        .data_length_code = 8,
        .data = {0x01, OBD2_SERVICE_CLEAR_DTC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    };
    
    return can_send_message(&msg);
}

esp_err_t obd2_read_dtc(void) {
    can_message_t msg = {
        .identifier = OBD2_REQUEST_CAN_ID,
        .data_length_code = 8,
        .data = {0x01, OBD2_SERVICE_DTC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    };
    
    return can_send_message(&msg);
}
