#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "ad1724.h"
#include "sdi12.h"

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

// SDI-12 Defines
const uint PIN_SDI12_DATA = 15; // GPIO15 for SDI-12 data line (needs level shifter to 5V)

// Global RTD config for SDI-12 callback
static rtd_config_t global_rtd_config;
static float last_temperature = 0.0f;

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

// SDI-12 measurement callback
// Called when the SDI-12 master requests a measurement
bool sdi12_measurement_callback(sdi12_measurement_t *data) {
    uint32_t rtd_data;
    uint8_t channel;

    printf("SDI-12 measurement requested\n");

    // Try to read current RTD data
    if (adc_read_rtd_data(&rtd_data, &channel)) {
        // Calculate temperature
        last_temperature = adc_calculate_temperature(rtd_data, &global_rtd_config);

        // Fill in measurement data
        data->values[0] = last_temperature;
        data->num_values = 1;
        data->time_seconds = 0; // Data ready immediately (ADC in continuous mode)

        printf("Temperature measurement: %.2f°C\n", last_temperature);
        return true;
    } else {
        // No data available yet, use last measurement
        data->values[0] = last_temperature;
        data->num_values = 1;
        data->time_seconds = 1; // Wait 1 second for next sample

        printf("Using last temperature: %.2f°C\n", last_temperature);
        return true;
    }
}

int main(){
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

    if (!comms_ok) {

        while (true) {
            // Slow blink for bad communication
            printf("AD7124 communication failed!\n");
            pico_set_led(true);
            sleep_ms(1000);
            pico_set_led(false);
            sleep_ms(1000);
        }
    }

    printf("AD7124 communication OK\n");

    // Configure RTD measurement (PT100, 4-wire ratiometric configuration as shown in the image)
    global_rtd_config.r_ref = 2960.0f;           // 4.99kΩ reference resistor (RREF1)
    global_rtd_config.r_rtd_0 = 100.0f;          // PT100 RTD (100Ω at 0°C) - using potentiometer for test
    global_rtd_config.alpha = 0.00385f;          // PT100 temperature coefficient
    global_rtd_config.excitation_current = AD7124_IOUT_250UA;  // 250µA excitation current (more stable)
    global_rtd_config.rtd_ainp = AD7124_AIN2;    // RTD positive connection (AIN2)
    global_rtd_config.rtd_ainm = AD7124_AIN3;    // RTD negative connection (AIN3)
    global_rtd_config.ref_ainp = AD7124_AIN0;    // Reference positive (across RREF1)
    global_rtd_config.ref_ainm = AD7124_AIN1;    // Reference negative (across RREF1)
    global_rtd_config.iout_pin = AD7124_AIN0;    // Excitation current output on AIN0

    printf("Configuring RTD measurement system...\n");
    bool rtd_config_ok = adc_configure_rtd(&global_rtd_config);

    if (!rtd_config_ok) {
        printf("RTD configuration failed!\n");
        while (true) {
            pico_set_led(true);
            sleep_ms(200);
            pico_set_led(false);
            sleep_ms(200);
        }
    }

    printf("RTD configuration complete.\n");

    // Initialize SDI-12 sensor interface
    printf("\n=== Initializing SDI-12 Sensor ===\n");
    sdi12_sensor_info_t sensor_info = {
        .address = '0',                     // Default address: 0
        .sdi_version = "14",                // SDI-12 version 1.4
        .vendor_id = "PICODIY ",            // Vendor ID (8 chars)
        .sensor_model = "PT100 ",           // Model (6 chars)
        .sensor_version = "1.0",            // Version (3 chars)
        .serial_number = "001"              // Serial number
    };

    if (!sdi12_init(PIN_SDI12_DATA, &sensor_info)) {
        printf("SDI-12 initialization failed!\n");
        while (true) {
            pico_set_led(true);
            sleep_ms(100);
            pico_set_led(false);
            sleep_ms(100);
        }
    }

    // Register measurement callback
    sdi12_set_measurement_callback(sdi12_measurement_callback);

    printf("SDI-12 sensor ready!\n");
    printf("Address: %c\n", sensor_info.address);
    printf("Send SDI-12 commands on GPIO%d (requires 3.3V <-> 5V level shifter)\n", PIN_SDI12_DATA);
    printf("\nExample commands:\n");
    printf("  0!     - Acknowledge (returns '0')\n");
    printf("  0I!    - Identify sensor\n");
    printf("  0M!    - Start measurement\n");
    printf("  0D0!   - Get data\n");
    printf("\n=== Main Loop: SDI-12 + RTD Monitoring ===\n");

    // Main loop - handle SDI-12 commands and update measurements
    uint32_t measurement_count = 0;
    absolute_time_t last_print_time = get_absolute_time();

    while (true) {
        // Process SDI-12 commands
        sdi12_task();

        // Periodically read and update temperature (every 2 seconds for monitoring)
        if (absolute_time_diff_us(last_print_time, get_absolute_time()) > 2000000) {
            measurement_count++;

            uint32_t rtd_data;
            uint8_t channel;

            // Try to read RTD data
            if (adc_read_rtd_data(&rtd_data, &channel)) {
                // Calculate temperature
                last_temperature = adc_calculate_temperature(rtd_data, &global_rtd_config);

                printf("\n[Monitor #%d] Temp: %.2f°C (Raw: 0x%06X)\n",
                       measurement_count, last_temperature, rtd_data);

                // Indicate successful measurement with LED
                pico_set_led(true);
                sleep_ms(50);
                pico_set_led(false);
            }

            last_print_time = get_absolute_time();
        }

        // Small delay to prevent busy-waiting
        sleep_ms(10);
    }
}