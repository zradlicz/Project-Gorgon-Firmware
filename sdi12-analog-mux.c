#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "ad1724.h"

// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
spi_inst_t *SPI_PORT = spi0;
const uint PIN_MISO = 16;
const uint PIN_CS   = 17;
const uint PIN_SCK  = 18;
const uint PIN_MOSI = 19;

// Perform LED initialisation
int pico_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // For Pico W devices we need to initialise the driver etc
    return cyw43_arch_init();
#endif
}

// Turn the led on or off
void pico_set_led(bool led_on) {
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}

int main()
{
    stdio_init_all();

    // Initialize LED
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    // SPI initialisation. ADCs often need slower speeds and specific modes
    spi_init(SPI_PORT, 1000*1000);

    // Set SPI mode 0 (CPOL=0, CPHA=0) which is common for ADCs
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    // Check ADC communication
    bool comms_ok = adc_verify_communication();

    while (true) {
        if (comms_ok) {
            // Fast blink for good communication
            pico_set_led(true);
            sleep_ms(100);
            pico_set_led(false);
            sleep_ms(100);
        } else {
            // Slow blink for bad communication
            pico_set_led(true);
            sleep_ms(1000);
            pico_set_led(false);
            sleep_ms(1000);
        }
    }
}