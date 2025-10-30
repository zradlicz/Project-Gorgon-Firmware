#include "adg708.h"
#include "pico/stdlib.h"
#include <stdio.h>

// Store pin configuration
static adg708_config_t mux_config;
static bool initialized = false;

void adg708_init(const adg708_config_t *config) {
    // Save configuration
    mux_config = *config;

    // Initialize GPIO pins as outputs
    gpio_init(mux_config.pin_en);
    gpio_init(mux_config.pin_a0);
    gpio_init(mux_config.pin_a1);
    gpio_init(mux_config.pin_a2);

    gpio_set_dir(mux_config.pin_en, GPIO_OUT);
    gpio_set_dir(mux_config.pin_a0, GPIO_OUT);
    gpio_set_dir(mux_config.pin_a1, GPIO_OUT);
    gpio_set_dir(mux_config.pin_a2, GPIO_OUT);

    // Start with mux disabled (EN low = disabled)
    gpio_put(mux_config.pin_en, 0);

    // Set address to 0 (will select switch 1 when enabled)
    gpio_put(mux_config.pin_a0, 0);
    gpio_put(mux_config.pin_a1, 0);
    gpio_put(mux_config.pin_a2, 0);

    initialized = true;
    printf("ADG708 mux initialized (EN=%d, A0=%d, A1=%d, A2=%d)\n",
           mux_config.pin_en, mux_config.pin_a0,
           mux_config.pin_a1, mux_config.pin_a2);
}

bool adg708_select_channel(uint8_t channel) {
    if (!initialized) {
        printf("ERROR: ADG708 not initialized!\n");
        return false;
    }

    // ADG708 has 8 switches numbered 1-8
    if (channel < 1 || channel > 8) {
        printf("ERROR: Invalid channel %d (must be 1-8)\n", channel);
        return false;
    }

    // Convert channel (1-8) to address (0-7)
    uint8_t address = channel - 1;

    // Set address lines based on channel number
    gpio_put(mux_config.pin_a0, (address >> 0) & 0x01);
    gpio_put(mux_config.pin_a1, (address >> 1) & 0x01);
    gpio_put(mux_config.pin_a2, (address >> 2) & 0x01);

    // Enable the mux (active HIGH)
    gpio_put(mux_config.pin_en, 1);

    printf("ADG708: Switch %d selected (A2=%d, A1=%d, A0=%d, EN=1)\n",
           channel,
           (address >> 2) & 0x01,
           (address >> 1) & 0x01,
           (address >> 0) & 0x01);

    return true;
}

void adg708_disable(void) {
    if (!initialized) {
        printf("ERROR: ADG708 not initialized!\n");
        return;
    }

    // Disable mux (EN low = no switch connected)
    gpio_put(mux_config.pin_en, 0);
    printf("ADG708: Mux disabled (EN=0)\n");
}

void adg708_enable(void) {
    if (!initialized) {
        printf("ERROR: ADG708 not initialized!\n");
        return;
    }

    // Enable mux (EN high = selected switch connected)
    gpio_put(mux_config.pin_en, 1);
    printf("ADG708: Mux enabled (EN=1)\n");
}
