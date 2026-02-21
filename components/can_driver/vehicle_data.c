#include "vehicle_data.h"
#include "obd2_pids.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>

static const char *TAG = "VEHICLE_DATA";

static vehicle_data_t g_vehicle_data = {0};

vehicle_data_t* vehicle_data_init(void) {
    memset(&g_vehicle_data, 0, sizeof(vehicle_data_t));
    g_vehicle_data.last_update = time(NULL);
    ESP_LOGI(TAG, "Vehicle data initialized");
    return &g_vehicle_data;
}

void vehicle_data_update(vehicle_data_t *data, uint8_t pid, float value) {
    if (data == NULL) {
        return;
    }
    
    switch (pid) {
        case PID_ENGINE_RPM:
            data->rpm = (uint16_t)value;
            break;
        case PID_ENGINE_LOAD:
            data->engine_load = (uint8_t)value;
            break;
        case PID_ENGINE_COOLANT_TEMP:
            data->coolant_temp = (int8_t)value;
            break;
        case PID_INTAKE_AIR_TEMP:
            data->intake_air_temp = (int8_t)value;
            break;
        case PID_THROTTLE_POSITION:
            data->throttle_position = (uint8_t)value;
            break;
        case PID_FUEL_PRESSURE:
            data->fuel_pressure = (uint16_t)value;
            break;
        case PID_SHORT_TERM_FUEL_TRIM_B1:
            data->short_term_fuel_trim_b1 = (int8_t)value;
            break;
        case PID_LONG_TERM_FUEL_TRIM_B1:
            data->long_term_fuel_trim_b1 = (int8_t)value;
            break;
        case PID_O2_SENSOR_1_B1:
            data->o2_sensor_1_b1 = (uint8_t)value;
            break;
        case PID_O2_SENSOR_2_B1:
            data->o2_sensor_2_b1 = (uint8_t)value;
            break;
        case PID_VEHICLE_SPEED:
            data->vehicle_speed = (uint8_t)value;
            break;
        case PID_TIMING_ADVANCE:
            data->timing_advance = (int8_t)value;
            break;
        case PID_CONTROL_MODULE_VOLTAGE:
            data->battery_voltage = (uint16_t)value;
            break;
        case PID_FUEL_LEVEL:
            data->fuel_level = (uint8_t)value;
            break;
        default:
            break;
    }
    
    data->last_update = time(NULL);
    data->update_count++;
}

vehicle_data_t* vehicle_data_get(void) {
    return &g_vehicle_data;
}

void vehicle_data_reset(void) {
    memset(&g_vehicle_data, 0, sizeof(vehicle_data_t));
    g_vehicle_data.last_update = time(NULL);
}

int vehicle_data_to_string(char *buffer, size_t size) {
    if (buffer == NULL || size == 0) {
        return 0;
    }
    
    return snprintf(buffer, size,
        "RPM: %u\n"
        "Speed: %u km/h\n"
        "Coolant: %d°C\n"
        "Load: %u%%\n"
        "Throttle: %u%%\n"
        "Fuel: %u%%\n"
        "Battery: %u.%uV\n"
        "Updates: %" PRIu32 "\n",
        g_vehicle_data.rpm,
        g_vehicle_data.vehicle_speed,
        g_vehicle_data.coolant_temp,
        g_vehicle_data.engine_load,
        g_vehicle_data.throttle_position,
        g_vehicle_data.fuel_level,
        g_vehicle_data.battery_voltage / 1000,
        (g_vehicle_data.battery_voltage % 1000) / 100,
        g_vehicle_data.update_count
    );
}
