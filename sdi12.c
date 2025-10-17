s
#include "sdi12.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "sdi12_uart.pio.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

// SDI-12 State
static struct {
    PIO pio;
    uint sm_tx;
    uint sm_rx;
    uint data_pin;
    sdi12_sensor_info_t sensor_info;
    bool (*measurement_callback)(sdi12_measurement_t *data);
    sdi12_measurement_t pending_measurement;
    bool measurement_ready;
    char rx_buffer[128];
    uint8_t rx_index;
    absolute_time_t last_char_time;
} sdi12_state;

// Calculate PIO clock divider for 1200 baud
// System clock is typically 125MHz
// For 1200 baud: clock_div = 125MHz / (1200 * 8) = ~13020.8
static float sdi12_get_clock_div(void) {
    uint32_t sys_clk = clock_get_hz(clk_sys);
    // 8x oversampling for better stability
    return (float)sys_clk / (SDI12_BAUD_RATE * 8);
}

bool sdi12_init(uint data_pin, const sdi12_sensor_info_t *sensor_info) {
    sdi12_state.data_pin = data_pin;
    sdi12_state.pio = pio0;
    sdi12_state.measurement_callback = NULL;
    sdi12_state.measurement_ready = false;
    sdi12_state.rx_index = 0;
    memcpy(&sdi12_state.sensor_info, sensor_info, sizeof(sdi12_sensor_info_t));

    // Configure GPIO as input initially (high impedance when idle)
    gpio_init(data_pin);
    gpio_set_dir(data_pin, GPIO_IN);
    gpio_pull_up(data_pin); // Pull-up to marking state

    // Load PIO programs
    uint offset_tx = pio_add_program(sdi12_state.pio, &sdi12_tx_program);
    uint offset_rx = pio_add_program(sdi12_state.pio, &sdi12_rx_program);

    // Get free state machines
    sdi12_state.sm_tx = pio_claim_unused_sm(sdi12_state.pio, true);
    sdi12_state.sm_rx = pio_claim_unused_sm(sdi12_state.pio, true);

    // Configure TX state machine
    pio_sm_config c_tx = sdi12_tx_program_get_default_config(offset_tx);
    sm_config_set_out_pins(&c_tx, data_pin, 1);
    sm_config_set_set_pins(&c_tx, data_pin, 1);
    sm_config_set_sideset_pins(&c_tx, data_pin);
    sm_config_set_clkdiv(&c_tx, sdi12_get_clock_div());
    sm_config_set_out_shift(&c_tx, true, false, 8); // Shift right, no autopull, 8 bits
    sm_config_set_fifo_join(&c_tx, PIO_FIFO_JOIN_TX); // Use all FIFO for TX

    pio_gpio_init(sdi12_state.pio, data_pin);
    pio_sm_set_consecutive_pindirs(sdi12_state.pio, sdi12_state.sm_tx, data_pin, 1, true);

    pio_sm_init(sdi12_state.pio, sdi12_state.sm_tx, offset_tx, &c_tx);

    // Configure RX state machine
    pio_sm_config c_rx = sdi12_rx_program_get_default_config(offset_rx);
    sm_config_set_in_pins(&c_rx, data_pin);
    sm_config_set_jmp_pin(&c_rx, data_pin);
    sm_config_set_clkdiv(&c_rx, sdi12_get_clock_div());
    sm_config_set_in_shift(&c_rx, true, false, 8); // Shift right, no autopush, 8 bits
    sm_config_set_fifo_join(&c_rx, PIO_FIFO_JOIN_RX); // Use all FIFO for RX

    pio_sm_init(sdi12_state.pio, sdi12_state.sm_rx, offset_rx, &c_rx);

    // Enable state machines
    pio_sm_set_enabled(sdi12_state.pio, sdi12_state.sm_rx, true);
    pio_sm_set_enabled(sdi12_state.pio, sdi12_state.sm_tx, true);

    printf("SDI-12 initialized on GPIO%d at %d baud\n", data_pin, SDI12_BAUD_RATE);
    printf("Sensor address: %c\n", sdi12_state.sensor_info.address);

    return true;
}

void sdi12_send_break(void) {
    // Send break: spacing (high/5V) for 12ms minimum
    gpio_set_dir(sdi12_state.data_pin, GPIO_OUT);
    gpio_put(sdi12_state.data_pin, 1); // Spacing state
    sleep_ms(SDI12_BREAK_MS);

    // Marking period: low for 8.33ms
    gpio_put(sdi12_state.data_pin, 0); // Marking state
    sleep_ms(SDI12_MARKING_MS);

    // Return to input mode
    gpio_set_dir(sdi12_state.data_pin, GPIO_IN);
}

void sdi12_send_response(const char *response) {
    // Take control of the data line
    gpio_set_dir(sdi12_state.data_pin, GPIO_OUT);

    // Small delay before responding (within 15ms response window)
    sleep_us(1000); // 1ms delay

    // Send each character
    for (int i = 0; response[i] != '\0'; i++) {
        // Wait for TX FIFO to have space
        while (pio_sm_is_tx_fifo_full(sdi12_state.pio, sdi12_state.sm_tx)) {
            tight_loop_contents();
        }

        // Send character
        pio_sm_put(sdi12_state.pio, sdi12_state.sm_tx, (uint32_t)response[i]);
    }

    // Wait for transmission to complete
    sleep_us(strlen(response) * 10000); // ~10ms per character at 1200 baud

    // Release the line (return to high impedance)
    gpio_set_dir(sdi12_state.data_pin, GPIO_IN);

    printf("SDI-12 TX: %s\n", response);
}

bool sdi12_parse_command(const char *cmd_string, sdi12_command_t *cmd) {
    if (!cmd_string || !cmd) return false;

    size_t len = strlen(cmd_string);
    if (len < 2) return false; // Minimum: "a!"

    // Extract address
    cmd->address = cmd_string[0];

    // Check if address is valid
    if (!isalnum(cmd->address) && cmd->address != '?') {
        return false;
    }

    // Check for terminator
    if (cmd_string[len - 1] != '!') {
        return false;
    }

    // Determine command type
    if (len == 2) {
        // Simple acknowledge: "a!"
        cmd->type = SDI12_CMD_ACKNOWLEDGE;
    } else if (cmd_string[1] == 'I' && len == 3) {
        // Identification: "aI!"
        cmd->type = SDI12_CMD_IDENTIFY;
    } else if (cmd_string[0] == '?' && len == 2) {
        // Address query: "?!"
        cmd->type = SDI12_CMD_ADDRESS_QUERY;
    } else if (cmd_string[1] == 'A' && len == 4) {
        // Change address: "aAb!"
        cmd->type = SDI12_CMD_CHANGE_ADDRESS;
        cmd->parameter[0] = cmd_string[2];
        cmd->parameter[1] = '\0';
    } else if (cmd_string[1] == 'M') {
        // Start measurement: "aM!" or "aMC!"
        if (len == 3) {
            cmd->type = SDI12_CMD_START_MEASUREMENT;
        } else if (len == 4 && cmd_string[2] == 'C') {
            cmd->type = SDI12_CMD_START_MEASUREMENT_CRC;
        } else {
            cmd->type = SDI12_CMD_UNKNOWN;
        }
    } else if (cmd_string[1] == 'D' && len >= 4) {
        // Send data: "aD0!" through "aD9!"
        cmd->type = SDI12_CMD_SEND_DATA;
        cmd->data_index = cmd_string[2] - '0';
    } else if (cmd_string[1] == 'C' && len == 3) {
        // Concurrent measurement: "aC!"
        cmd->type = SDI12_CMD_CONTINUOUS_MEASUREMENT;
    } else if (cmd_string[1] == 'X') {
        // Extended command: "aX..."
        cmd->type = SDI12_CMD_EXTENDED_COMMAND;
        strncpy(cmd->parameter, &cmd_string[2], sizeof(cmd->parameter) - 1);
    } else {
        cmd->type = SDI12_CMD_UNKNOWN;
    }

    return true;
}

void sdi12_format_identification(char *buffer, const sdi12_sensor_info_t *sensor_info) {
    // Format: allccccccccmmmmmmvvvxxxxxxxxxx<CR><LF>
    // a - address
    // ll - SDI-12 version (e.g., "14")
    // cccccccc - vendor (8 chars)
    // mmmmmm - model (6 chars)
    // vvv - version (3 chars)
    // xxxxxxxxxx - serial number (optional, up to 13 chars)

    snprintf(buffer, 64, "%c%s%-8s%-6s%-3s%s\r\n",
             sensor_info->address,
             sensor_info->sdi_version,
             sensor_info->vendor_id,
             sensor_info->sensor_model,
             sensor_info->sensor_version,
             sensor_info->serial_number);
}

void sdi12_format_measurement_response(char *buffer, char address, uint16_t time_sec, uint8_t num_values) {
    // Format: atttn<CR><LF>
    // a - address
    // ttt - time in seconds (000-999)
    // n - number of values (0-9)

    snprintf(buffer, 16, "%c%03d%d\r\n", address, time_sec, num_values);
}

void sdi12_format_data_response(char *buffer, char address, const sdi12_measurement_t *data) {
    // Format: a±x.xxx±y.yyy...<CR><LF>
    // Maximum 35 characters including address, CR, LF

    char *ptr = buffer;
    *ptr++ = address;

    for (int i = 0; i < data->num_values && i < 9; i++) {
        // Format each value with sign
        int written = snprintf(ptr, 10, "%+.2f", data->values[i]);
        ptr += written;

        // Check if we're approaching limit (leave room for CR LF)
        if (ptr - buffer > 33) break;
    }

    *ptr++ = '\r';
    *ptr++ = '\n';
    *ptr = '\0';
}

void sdi12_set_measurement_callback(bool (*callback)(sdi12_measurement_t *data)) {
    sdi12_state.measurement_callback = callback;
}

static void sdi12_handle_command(const sdi12_command_t *cmd) {
    char response[80];

    // Check if command is addressed to this sensor
    if (cmd->address != sdi12_state.sensor_info.address && cmd->address != '?') {
        return; // Not for us, ignore
    }

    switch (cmd->type) {
        case SDI12_CMD_ACKNOWLEDGE:
            // Simple acknowledge: respond with address
            snprintf(response, sizeof(response), "%c\r\n", sdi12_state.sensor_info.address);
            sdi12_send_response(response);
            break;

        case SDI12_CMD_IDENTIFY:
            // Send identification string
            sdi12_format_identification(response, &sdi12_state.sensor_info);
            sdi12_send_response(response);
            break;

        case SDI12_CMD_ADDRESS_QUERY:
            // Respond with our address
            snprintf(response, sizeof(response), "%c\r\n", sdi12_state.sensor_info.address);
            sdi12_send_response(response);
            break;

        case SDI12_CMD_START_MEASUREMENT:
        case SDI12_CMD_START_MEASUREMENT_CRC:
            // Trigger measurement
            if (sdi12_state.measurement_callback) {
                sdi12_state.measurement_ready = false;
                if (sdi12_state.measurement_callback(&sdi12_state.pending_measurement)) {
                    // Respond with time and number of values
                    sdi12_format_measurement_response(response,
                                                     sdi12_state.sensor_info.address,
                                                     sdi12_state.pending_measurement.time_seconds,
                                                     sdi12_state.pending_measurement.num_values);
                    sdi12_send_response(response);
                    sdi12_state.measurement_ready = true;
                }
            }
            break;

        case SDI12_CMD_SEND_DATA:
            // Send measurement data (only support D0 for now)
            if (cmd->data_index == 0 && sdi12_state.measurement_ready) {
                sdi12_format_data_response(response, sdi12_state.sensor_info.address,
                                          &sdi12_state.pending_measurement);
                sdi12_send_response(response);
            }
            break;

        case SDI12_CMD_CHANGE_ADDRESS:
            // Address change - not implemented for now
            break;

        default:
            // Unknown command - no response
            break;
    }
}

void sdi12_task(void) {
    // Check for received characters
    while (!pio_sm_is_rx_fifo_empty(sdi12_state.pio, sdi12_state.sm_rx)) {
        uint8_t c = (uint8_t)pio_sm_get(sdi12_state.pio, sdi12_state.sm_rx);

        // Store character in buffer
        if (sdi12_state.rx_index < sizeof(sdi12_state.rx_buffer) - 1) {
            sdi12_state.rx_buffer[sdi12_state.rx_index++] = c;
            sdi12_state.last_char_time = get_absolute_time();

            // Check for command terminator
            if (c == '!') {
                sdi12_state.rx_buffer[sdi12_state.rx_index] = '\0';

                printf("SDI-12 RX: %s\n", sdi12_state.rx_buffer);

                // Parse and handle command
                sdi12_command_t cmd;
                if (sdi12_parse_command(sdi12_state.rx_buffer, &cmd)) {
                    sdi12_handle_command(&cmd);
                }

                // Reset buffer
                sdi12_state.rx_index = 0;
            }
        } else {
            // Buffer overflow, reset
            sdi12_state.rx_index = 0;
        }
    }

    // Timeout detection (reset if no character for 100ms)
    if (sdi12_state.rx_index > 0) {
        if (absolute_time_diff_us(sdi12_state.last_char_time, get_absolute_time()) > 100000) {
            sdi12_state.rx_index = 0; // Reset on timeout
        }
    }
}
