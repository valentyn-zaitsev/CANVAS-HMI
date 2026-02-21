#ifndef VEHICLE_DATA_H
#define VEHICLE_DATA_H

#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Vehicle Data Structure
 * Stores current vehicle parameters
 */

typedef struct {
    // Engine parameters
    uint16_t rpm;
    uint8_t engine_load;
    int8_t coolant_temp;
    int8_t intake_air_temp;
    uint8_t throttle_position;
    
    // Fuel system
    uint8_t fuel_level;
    uint16_t fuel_pressure;
    int8_t short_term_fuel_trim_b1;
    int8_t long_term_fuel_trim_b1;
    
    // Emissions
    uint8_t o2_sensor_1_b1;
    uint8_t o2_sensor_2_b1;
    
    // Vehicle speed and timing
    uint8_t vehicle_speed;
    int8_t timing_advance;
    
    // Electrical
    uint16_t battery_voltage;
    
    // Transmission (if available)
    uint8_t gear;
    uint8_t transmission_temp;
    
    // Timestamps
    time_t last_update;
    uint32_t update_count;
    
    // Status flags
    uint8_t mil_status;  // Malfunction Indicator Lamp
    uint8_t dtc_count;   // Number of Diagnostic Trouble Codes
    
} vehicle_data_t;

/**
 * Initialize vehicle data structure
 * @return Pointer to vehicle data
 */
vehicle_data_t* vehicle_data_init(void);

/**
 * Update vehicle data with new PID value
 * @param data Vehicle data structure
 * @param pid PID that was updated
 * @param value New value
 */
void vehicle_data_update(vehicle_data_t *data, uint8_t pid, float value);

/**
 * Get vehicle data
 * @return Pointer to current vehicle data
 */
vehicle_data_t* vehicle_data_get(void);

/**
 * Reset vehicle data
 */
void vehicle_data_reset(void);

/**
 * Get formatted string of vehicle data
 * @param buffer Output buffer
 * @param size Buffer size
 * @return Number of bytes written
 */
int vehicle_data_to_string(char *buffer, size_t size);

#ifdef __cplusplus
}
#endif

#endif // VEHICLE_DATA_H
