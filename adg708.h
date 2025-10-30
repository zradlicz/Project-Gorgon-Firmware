#ifndef ADG708_H
#define ADG708_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"

// ADG708 is an 8-channel analog multiplexer
// Channels: 1-8 (switch 1-8 in datasheet)
// Control: 3 address lines (A0, A1, A2) + EN (active HIGH)

typedef struct {
    uint pin_en;   // Enable pin (active HIGH)
    uint pin_a0;   // Address bit 0
    uint pin_a1;   // Address bit 1
    uint pin_a2;   // Address bit 2
} adg708_config_t;

/**
 * Initialize the ADG708 mux GPIO pins
 * @param config Pin configuration for the mux
 */
void adg708_init(const adg708_config_t *config);

/**
 * Select a specific channel on the mux (1-8)
 * @param channel Channel number to select (1-8, corresponds to switch number)
 * @return true if successful, false if invalid channel
 */
bool adg708_select_channel(uint8_t channel);

/**
 * Disable the mux (all channels disconnected)
 */
void adg708_disable(void);

/**
 * Enable the mux (selected channel connected)
 */
void adg708_enable(void);

#endif // ADG708_H
