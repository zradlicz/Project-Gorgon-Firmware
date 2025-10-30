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
    sdi12_measurement_callback_t measurement_callback;
    sdi12_measurement_t pending_measurement;
    bool measurement_ready;
    char rx_buffer[128];
    uint8_t rx_index;
    absolute_time_t last_char_time;
} sdi12_state;

// Calculate PIO clock divider for 1200 baud
// System clock is typically 125MHz
// For 1200 baud with 8 cycles per bit: clock_div = 125MHz / (1200 * 8)
static float sdi12_get_clock_div(void) {
    uint32_t sys_clk = clock_get_hz(clk_sys);
    // 8 cycles per bit for PIO timing
    return (float)sys_clk / (SDI12_BAUD_RATE * 8);
}

bool sdi12_init(uint data_pin, const sdi12_sensor_info_t *sensor_info) {
    sdi12_state.data_pin = data_pin;
    sdi12_state.pio = pio0;
    sdi12_state.rx_index = 0;
    sdi12_state.measurement_callback = NULL;
    sdi12_state.measurement_ready = false;
    memcpy(&sdi12_state.sensor_info, sensor_info, sizeof(sdi12_sensor_info_t));

    // Configure GPIO - initially as input without pull-up (level shifter handles idle state)
    gpio_init(data_pin);
    gpio_set_dir(data_pin, GPIO_IN);
    gpio_disable_pulls(data_pin); // No pull-up/down, level shifter handles the signal

    // SDI-12 uses inverted logic (marking=LOW, spacing=HIGH)
    // Invert at GPIO level so PIO sees standard UART logic
    gpio_set_inover(data_pin, GPIO_OVERRIDE_INVERT);

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
    sm_config_set_out_shift(&c_tx, true, false, 32); // Shift right, no autopull, 32-bit threshold (like UART example)
    sm_config_set_fifo_join(&c_tx, PIO_FIFO_JOIN_TX); // Use all FIFO for TX

    pio_sm_init(sdi12_state.pio, sdi12_state.sm_tx, offset_tx, &c_tx);

    // Configure RX state machine
    pio_sm_config c_rx = sdi12_rx_program_get_default_config(offset_rx);
    sm_config_set_in_pins(&c_rx, data_pin);
    sm_config_set_jmp_pin(&c_rx, data_pin);
    sm_config_set_clkdiv(&c_rx, sdi12_get_clock_div());
    sm_config_set_in_shift(&c_rx, true, false, 8); // Shift RIGHT for LSB-first, no autopush, 8 bits (7 data + 1 parity)
    sm_config_set_fifo_join(&c_rx, PIO_FIFO_JOIN_RX); // Use all FIFO for RX

    pio_sm_init(sdi12_state.pio, sdi12_state.sm_rx, offset_rx, &c_rx);

    // Enable RX state machine (TX will be enabled when needed)
    pio_sm_set_enabled(sdi12_state.pio, sdi12_state.sm_rx, true);

    uint32_t sys_clk = clock_get_hz(clk_sys);
    float clk_div = sdi12_get_clock_div();
    printf("SDI-12 initialized on GPIO%d at %d baud\n", data_pin, SDI12_BAUD_RATE);
    printf("System clock: %u Hz\n", sys_clk);
    printf("Clock divider: %.2f\n", clk_div);
    printf("Effective PIO clock: %.2f Hz\n", (float)sys_clk / clk_div);
    printf("Sensor address: %c\n", sdi12_state.sensor_info.address);

    return true;
}

void sdi12_send_response(const char *response) {
    printf("SDI-12 TX: %s", response);

    // Disable RX while transmitting
    pio_sm_set_enabled(sdi12_state.pio, sdi12_state.sm_rx, false);

    // Clear RX FIFO to avoid picking up our own transmission
    pio_sm_clear_fifos(sdi12_state.pio, sdi12_state.sm_rx);

    // Also clear TX FIFO before starting
    pio_sm_clear_fifos(sdi12_state.pio, sdi12_state.sm_tx);

    // Take control of the data line for transmission
    // Set pin to LOW (idle state for inverted SDI-12 logic) before switching to PIO control
    pio_sm_set_pins_with_mask(sdi12_state.pio, sdi12_state.sm_tx,
                               0u << sdi12_state.data_pin,
                               1u << sdi12_state.data_pin);
    pio_sm_set_pindirs_with_mask(sdi12_state.pio, sdi12_state.sm_tx,
                                  1u << sdi12_state.data_pin,
                                  1u << sdi12_state.data_pin);
    pio_gpio_init(sdi12_state.pio, sdi12_state.data_pin);

    // Enable TX state machine
    pio_sm_set_enabled(sdi12_state.pio, sdi12_state.sm_tx, true);

    // Small delay before responding (within 15ms response window)
    sleep_us(1000); // 1ms delay

    // Send each character
    for (int i = 0; response[i] != '\0'; i++) {
        // Wait for TX FIFO to have space
        while (pio_sm_is_tx_fifo_full(sdi12_state.pio, sdi12_state.sm_tx)) {
            tight_loop_contents();
        }

        // Get 7-bit character
        uint8_t data = (uint8_t)(response[i] & 0x7F);

        // Calculate even parity (count number of 1s in the 7 data bits)
        uint8_t parity = 0;
        uint8_t temp = data;
        for (int bit = 0; bit < 7; bit++) {
            parity ^= (temp & 1);
            temp >>= 1;
        }

        // Combine 7 data bits + 1 parity bit (parity in bit 7)
        uint8_t byte_with_parity = data | (parity << 7);

        // Invert all bits for SDI-12's inverted logic (marking=LOW, spacing=HIGH)
        uint8_t inverted_byte = ~byte_with_parity;

        printf("TX char '%c': data=0x%02X parity=%d byte=0x%02X inverted=0x%02X\n",
               response[i], data, parity, byte_with_parity, inverted_byte);

        pio_sm_put(sdi12_state.pio, sdi12_state.sm_tx, (uint32_t)inverted_byte);
    }

    // Wait for TX FIFO to drain
    while (!pio_sm_is_tx_fifo_empty(sdi12_state.pio, sdi12_state.sm_tx)) {
        tight_loop_contents();
    }

    // Wait for transmission to complete (state machine to finish)
    // Each character takes ~8.33ms at 1200 baud (1/1200 * 10 bits)
    sleep_us(strlen(response) * 10000);

    // Disable TX state machine
    pio_sm_set_enabled(sdi12_state.pio, sdi12_state.sm_tx, false);

    // Release the line (return to input for RX)
    pio_sm_set_consecutive_pindirs(sdi12_state.pio, sdi12_state.sm_tx, sdi12_state.data_pin, 1, false);
    gpio_set_inover(sdi12_state.data_pin, GPIO_OVERRIDE_INVERT);

    // Wait for line to settle before re-enabling RX
    sleep_us(10000); // 10ms settling time (increased from 5ms)

    // Clear RX FIFO again before re-enabling
    pio_sm_clear_fifos(sdi12_state.pio, sdi12_state.sm_rx);

    // Re-enable RX
    pio_sm_set_enabled(sdi12_state.pio, sdi12_state.sm_rx, true);

    // Additional delay after re-enabling to let RX stabilize
    sleep_us(2000); // 2ms
}

bool sdi12_parse_command(const char *cmd_string, sdi12_command_t *cmd) {
    if (!cmd_string || !cmd) return false;

    size_t len = strlen(cmd_string);
    if (len < 2) return false; // Minimum: "a!"

    // Clear the command structure
    memset(cmd, 0, sizeof(sdi12_command_t));

    // Extract address (first character)
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
    if (len == 2 && cmd->address != '?') {
        // Simple acknowledge: "a!"
        cmd->type = SDI12_CMD_ACKNOWLEDGE;
    } else if (len >= 3 && cmd_string[1] == 'I' && cmd_string[len-1] == '!') {
        // Identification: "aI!"
        cmd->type = SDI12_CMD_IDENTIFY;
    } else if (cmd->address == '?' && len == 2) {
        // Address query: "?!"
        cmd->type = SDI12_CMD_ADDRESS_QUERY;
    } else if (len == 4 && cmd_string[1] == 'A') {
        // Change address: "aAb!"
        cmd->type = SDI12_CMD_CHANGE_ADDRESS;
        cmd->parameter[0] = cmd_string[2];
        cmd->parameter[1] = '\0';
    } else if (cmd_string[1] == 'M') {
        // Start measurement: "aM!" or "aMx!" (x = 1-9)
        cmd->type = SDI12_CMD_START_MEASUREMENT;
        if (len == 4 && isdigit(cmd_string[2])) {
            // aMx! format (e.g., aM1!, aM2!, aM3!)
            cmd->measurement_index = cmd_string[2] - '0';
        } else if (len == 3) {
            // aM! format (default, index 0)
            cmd->measurement_index = 0;
        } else {
            cmd->type = SDI12_CMD_UNKNOWN;
        }
    } else if (cmd_string[1] == 'D' && len >= 4 && isdigit(cmd_string[2])) {
        // Send data: "aD0!" through "aD9!"
        cmd->type = SDI12_CMD_SEND_DATA;
        cmd->data_index = cmd_string[2] - '0';
    } else {
        cmd->type = SDI12_CMD_UNKNOWN;
    }

    return true;
}

void sdi12_format_identification(char *buffer, const sdi12_sensor_info_t *sensor_info) {
    // Format: allccccccccmmmmmmvvvxxxxxxxxxx<CR><LF>
    // a - address (1 char)
    // ll - SDI-12 version (2 chars, e.g., "14")
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
        int written = snprintf(ptr, 12, "%+.2f", data->values[i]);
        ptr += written;

        // Check if we're approaching limit (leave room for CR LF)
        if (ptr - buffer > 33) break;
    }

    *ptr++ = '\r';
    *ptr++ = '\n';
    *ptr = '\0';
}

void sdi12_set_measurement_callback(sdi12_measurement_callback_t callback) {
    sdi12_state.measurement_callback = callback;
}

static void sdi12_handle_command(const sdi12_command_t *cmd) {
    char response[80];

    // Check if command is addressed to this sensor
    if (cmd->address != sdi12_state.sensor_info.address && cmd->address != '?') {
        printf("Command not for this sensor (address: %c, ours: %c)\n",
               cmd->address, sdi12_state.sensor_info.address);
        return; // Not for us, ignore
    }

    switch (cmd->type) {
        case SDI12_CMD_IDENTIFY:
            // Send identification string
            printf("Sending identification response\n");
            sdi12_format_identification(response, &sdi12_state.sensor_info);
            sdi12_send_response(response);
            break;

        case SDI12_CMD_START_MEASUREMENT:
            // Start measurement command (aM!, aM1!, aM2!, aM3!)
            printf("Measurement command received (index: %d)\n", cmd->measurement_index);
            if (sdi12_state.measurement_callback) {
                sdi12_state.measurement_ready = false;

                // Call the callback to get measurement data
                if (sdi12_state.measurement_callback(cmd->measurement_index, &sdi12_state.pending_measurement)) {
                    // Format and send response: atttn<CR><LF>
                    sdi12_format_measurement_response(response,
                                                     sdi12_state.sensor_info.address,
                                                     sdi12_state.pending_measurement.time_seconds,
                                                     sdi12_state.pending_measurement.num_values);
                    sdi12_send_response(response);
                    sdi12_state.measurement_ready = true;
                    printf("Measurement ready: %d values, %d seconds\n",
                           sdi12_state.pending_measurement.num_values,
                           sdi12_state.pending_measurement.time_seconds);
                } else {
                    printf("Measurement callback failed\n");
                }
            } else {
                printf("No measurement callback registered\n");
            }
            break;

        case SDI12_CMD_SEND_DATA:
            // Send data command (aD0!, aD1!, etc.)
            printf("Send data command received (index: %d)\n", cmd->data_index);
            if (cmd->data_index == 0) {
                if (sdi12_state.measurement_ready) {
                    // Format and send data response: a±x.xxx±y.yyy<CR><LF>
                    sdi12_format_data_response(response, sdi12_state.sensor_info.address,
                                              &sdi12_state.pending_measurement);
                    sdi12_send_response(response);
                    printf("Data sent\n");
                } else {
                    printf("No measurement ready - send aM command first\n");
                }
            } else {
                printf("Invalid data index: %d (only D0 supported)\n", cmd->data_index);
            }
            break;

        case SDI12_CMD_ACKNOWLEDGE:
        case SDI12_CMD_ADDRESS_QUERY:
        case SDI12_CMD_CHANGE_ADDRESS:
        default:
            printf("Command not supported\n");
            break;
    }
}

void sdi12_task(void) {
    // Check for received characters from RX FIFO
    while (!pio_sm_is_rx_fifo_empty(sdi12_state.pio, sdi12_state.sm_rx)) {
        uint32_t raw_word = pio_sm_get(sdi12_state.pio, sdi12_state.sm_rx);
        // Data is right-shifted, so it's left-justified in the 32-bit word
        // Read from the uppermost byte (bits 31:24)
        uint8_t c = (uint8_t)(raw_word >> 24);

        // Debug: show the full 32-bit word and byte as received
        printf("RX raw word: 0x%08X, byte: 0x%02X (binary: ", raw_word, c);
        for (int i = 7; i >= 0; i--) {
            printf("%d", (c >> i) & 1);
        }
        printf(")\n");

        // GPIO inversion already handled the SDI-12 inverted logic
        // Now we have 8 bits: 7 data bits (LSB) + 1 parity bit (MSB)
        uint8_t parity_bit = (c >> 7) & 0x01;

        // Extract 7 data bits
        c &= 0x7F;

        // TODO: Validate even parity
        // For now, just use the data bits

        // Debug: print received byte
        printf("RX byte: 0x%02X (parity=%d) '%c'\n",
               c, parity_bit, (c >= 32 && c < 127) ? c : '.');

        // Ignore non-printable characters (likely break signal artifacts)
        // SDI-12 uses printable ASCII characters (space to ~)
        if (c < 0x20 || c == 0x7F) {
            printf("Ignoring invalid byte (break/noise artifact)\n");
            continue;
        }

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

                    // Drain any remaining characters from FIFO after command handling
                    // This prevents buffered characters from being processed as part of next command
                    while (!pio_sm_is_rx_fifo_empty(sdi12_state.pio, sdi12_state.sm_rx)) {
                        pio_sm_get(sdi12_state.pio, sdi12_state.sm_rx);
                        printf("Drained leftover character from RX FIFO\n");
                    }

                    // Small delay after handling command to ensure line is stable
                    sleep_us(5000); // 5ms
                } else {
                    printf("Failed to parse command\n");
                }

                // Reset buffer
                sdi12_state.rx_index = 0;
            }
        } else {
            // Buffer overflow, reset
            printf("RX buffer overflow, resetting\n");
            sdi12_state.rx_index = 0;
        }
    }

    // Timeout detection (reset if no character for 100ms)
    if (sdi12_state.rx_index > 0) {
        if (absolute_time_diff_us(sdi12_state.last_char_time, get_absolute_time()) > 100000) {
            printf("RX timeout, resetting buffer\n");
            sdi12_state.rx_index = 0; // Reset on timeout
        }
    }
}
