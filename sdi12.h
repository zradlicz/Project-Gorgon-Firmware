#ifndef SDI12_H
#define SDI12_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"

// SDI-12 Protocol Constants
#define SDI12_BAUD_RATE         1200
#define SDI12_BREAK_MS          12      // Minimum break duration in milliseconds
#define SDI12_MARKING_MS        8       // Marking period after break
#define SDI12_RESPONSE_TIME_MS  15      // Maximum response time

// SDI-12 Command Types
typedef enum {
    SDI12_CMD_ACKNOWLEDGE,              // a! - Acknowledge active
    SDI12_CMD_IDENTIFY,                 // aI! - Send identification
    SDI12_CMD_ADDRESS_QUERY,            // ?! - Address query
    SDI12_CMD_CHANGE_ADDRESS,           // aAb! - Change address from a to b
    SDI12_CMD_START_MEASUREMENT,        // aM! - Start measurement
    SDI12_CMD_START_MEASUREMENT_CRC,    // aMC! - Start measurement with CRC
    SDI12_CMD_SEND_DATA,                // aD0! - Send data
    SDI12_CMD_CONTINUOUS_MEASUREMENT,   // aC! - Start concurrent measurement
    SDI12_CMD_EXTENDED_COMMAND,         // aX... - Extended commands
    SDI12_CMD_UNKNOWN
} sdi12_command_type_t;

// SDI-12 Command Structure
typedef struct {
    char address;                       // Sensor address (0-9, a-z, A-Z)
    sdi12_command_type_t type;          // Command type
    char parameter[16];                 // Additional parameters
    uint8_t data_index;                 // For aD commands (D0, D1, etc.)
} sdi12_command_t;

// SDI-12 Sensor Configuration
typedef struct {
    char address;                       // Sensor address
    char sdi_version[3];                // SDI-12 version (e.g., "14")
    char vendor_id[9];                  // Vendor identification (8 chars max)
    char sensor_model[7];               // Sensor model (6 chars max)
    char sensor_version[4];             // Sensor version (3 chars max)
    char serial_number[14];             // Serial number (optional, 13 chars max)
} sdi12_sensor_info_t;

// SDI-12 Measurement Data
typedef struct {
    float values[9];                    // Up to 9 measurement values
    uint8_t num_values;                 // Number of values
    uint16_t time_seconds;              // Time until data ready (seconds)
} sdi12_measurement_t;

// Function declarations

/**
 * Initialize SDI-12 interface
 * @param data_pin GPIO pin for SDI-12 data line
 * @param sensor_info Pointer to sensor information structure
 * @return true if initialization successful
 */
bool sdi12_init(uint data_pin, const sdi12_sensor_info_t *sensor_info);

/**
 * Process incoming SDI-12 commands
 * Called periodically to check for and handle commands
 */
void sdi12_task(void);

/**
 * Parse received SDI-12 command
 * @param cmd_string Command string received
 * @param cmd Pointer to command structure to fill
 * @return true if command parsed successfully
 */
bool sdi12_parse_command(const char *cmd_string, sdi12_command_t *cmd);

/**
 * Send SDI-12 response
 * @param response Response string to send
 */
void sdi12_send_response(const char *response);

/**
 * Send break signal (wake up all sensors on bus)
 */
void sdi12_send_break(void);

/**
 * Set measurement callback function
 * @param callback Function to call when measurement is requested
 */
void sdi12_set_measurement_callback(bool (*callback)(sdi12_measurement_t *data));

/**
 * Format identification response
 * @param buffer Buffer to store response
 * @param sensor_info Sensor information
 */
void sdi12_format_identification(char *buffer, const sdi12_sensor_info_t *sensor_info);

/**
 * Format measurement response (aM! command)
 * @param buffer Buffer to store response
 * @param address Sensor address
 * @param time_sec Time until data ready
 * @param num_values Number of values to return
 */
void sdi12_format_measurement_response(char *buffer, char address, uint16_t time_sec, uint8_t num_values);

/**
 * Format data response (aD0! command)
 * @param buffer Buffer to store response
 * @param address Sensor address
 * @param data Measurement data
 */
void sdi12_format_data_response(char *buffer, char address, const sdi12_measurement_t *data);

#endif // SDI12_H
