#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "ad1724.h"
#include "sdi12.h"
#include "adg708.h"

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
const uint PIN_SDI12_DATA = 10; // GPIO10 for SDI-12 data line (needs level shifter to 5V)

// Global RTD config for SDI-12 callback
static rtd_config_t global_rtd_config;
static float last_temperature = 0.0f;

// Multi-channel RTD storage (RTDs 1-7)
#define NUM_RTDS 7
// Support for RTDs 1-7

static float rtd_temperatures[NUM_RTDS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static float rtd_resistances[NUM_RTDS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

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
// measurement_index: 0 = aM!, 1 = aM1!, 2 = aM2!, 3 = aM3!, 8 = aM8!, 9 = aM9!
bool sdi12_measurement_callback(uint8_t measurement_index, sdi12_measurement_t *data) {
    printf("SDI-12 measurement requested for index %d\n", measurement_index);

    // Handle temperature sensor commands
    if (measurement_index == 8) {
        // M8! - Raspberry Pi Pico internal temperature
        printf("Reading Raspberry Pi Pico internal temperature\n");
        float temperature = pico_read_internal_temperature();

        if (temperature > -100.0f) {  // Valid temperature
            data->values[0] = temperature;
            data->num_values = 1;
            data->time_seconds = 0;
            printf("Pico Temperature: %.2f°C\n", temperature);
            return true;
        } else {
            printf("ERROR: Failed to read Pico temperature\n");
            return false;
        }
    } else if (measurement_index == 9) {
        // M9! - AD7124 ADC internal temperature
        printf("Reading AD7124 internal temperature\n");
        float temperature = adc_read_internal_temperature();

        if (temperature > -100.0f) {  // Valid temperature
            data->values[0] = temperature;
            data->num_values = 1;
            data->time_seconds = 0;
            printf("AD7124 Temperature: %.2f°C\n", temperature);
            return true;
        } else {
            printf("ERROR: Failed to read AD7124 temperature\n");
            return false;
        }
    }

    // Validate measurement index (1-7 for RTD1-RTD7)
    if (measurement_index < 1 || measurement_index > NUM_RTDS) {
        printf("Invalid measurement index: %d (valid: 1-7, 8-9)\n", measurement_index);
        return false;
    }

    // Get RTD number (convert from 1-based to 0-based index)
    uint8_t rtd_num = measurement_index - 1;
    uint8_t mux_channel = measurement_index; // Mux channels are 1-7
    uint8_t adc_channel_num = rtd_num;       // ADC channels are 0-6

    printf("Reading RTD %d (Mux Ch%d, ADC Ch%d)\n", measurement_index, mux_channel, adc_channel_num);

    // Step 1: Enable only this ADC channel
    adc_enable_single_channel(adc_channel_num);

    // Step 2: Switch mux to connect this RTD
    if (!adg708_select_channel(mux_channel)) {
        printf("ERROR: Failed to select mux channel %d\n", mux_channel);
        return false;
    }

    // Step 3: Wait for mux to settle and excitation current to stabilize
    sleep_ms(20);

    // Step 4: Start a single conversion
    adc_start_single_conversion();

    // Step 5: Wait for conversion to complete (poll RDY bit)
    uint8_t status;
    int timeout = 100;  // 100 attempts max
    bool conversion_ready = false;

    while (timeout > 0) {
        adc_reg_read(AD7124_STATUS_REG, &status, 1);
        if ((status & 0x80) == 0) {  // RDY bit is low = data ready
            conversion_ready = true;
            break;
        }
        sleep_ms(10);
        timeout--;
    }

    if (!conversion_ready) {
        printf("ERROR: Conversion timeout for RTD %d\n", measurement_index);
        return false;
    }

    // Step 6: Read the data
    uint32_t rtd_data;
    uint8_t read_channel;
    if (adc_read_rtd_data(&rtd_data, &read_channel)) {
        // Calculate resistance and temperature
        float resistance = adc_calculate_resistance(rtd_data, &global_rtd_config);
        float temperature = adc_calculate_temperature(rtd_data, &global_rtd_config);

        // Store in measurement data structure
        rtd_resistances[rtd_num] = resistance;
        rtd_temperatures[rtd_num] = temperature;

        // Fill in SDI-12 measurement response
        data->values[0] = temperature;
        data->num_values = 1;
        data->time_seconds = 0; // Data ready immediately

        printf("RTD %d: %.2fΩ, %.2f°C (Raw: 0x%06X)\n",
               measurement_index, resistance, temperature, rtd_data);

        return true;
    } else {
        printf("ERROR: Failed to read data for RTD %d\n", measurement_index);
        return false;
    }
}

int main(){
    stdio_init_all();

    // Initialize LED
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    // Initialize ADG708 analog mux
    printf("\n=== Initializing ADG708 Analog Mux ===\n");
    adg708_config_t mux_config = {
        .pin_en = 20,
        .pin_a0 = 21,
        .pin_a1 = 22,
        .pin_a2 = 23
    };
    adg708_init(&mux_config);

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

    // Configure RTD measurement (PT100, 4-wire ratiometric configuration)
    // Note: The ADC channels are already configured for all 3 RTDs in adc_configure_rtd()
    global_rtd_config.r_ref = 5030.0f;           // 5.03kΩ reference resistor (R19) - calibrated value
    global_rtd_config.r_rtd_0 = 100.0f;          // PT100 RTD (100Ω at 0°C) - using potentiometer for test
    global_rtd_config.alpha = 0.00385f;          // PT100 temperature coefficient
    global_rtd_config.excitation_current = AD7124_IOUT_50UA;  // 50µA excitation current
    global_rtd_config.rtd_ainp = AD7124_AIN1;    // Dummy - not used for multi-channel
    global_rtd_config.rtd_ainm = AD7124_AIN2;    // Dummy - not used for multi-channel
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
    printf("Will cycle through RTDs 1-7 using ADG708 mux and ADC channels\n");

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
    printf("\nSupported commands:\n");
    printf("  0I!    - Identify sensor\n");
    printf("  0M1!   - Measure RTD1 temperature\n");
    printf("  0M2!   - Measure RTD2 temperature\n");
    printf("  0M3!   - Measure RTD3 temperature\n");
    printf("  0M4!   - Measure RTD4 temperature\n");
    printf("  0M5!   - Measure RTD5 temperature\n");
    printf("  0M6!   - Measure RTD6 temperature\n");
    printf("  0M7!   - Measure RTD7 temperature\n");
    printf("  0M8!   - Measure Raspberry Pi Pico internal temperature\n");
    printf("  0M9!   - Measure AD7124 ADC internal temperature\n");
    printf("  0D0!   - Get measurement data (after aM command)\n");
    printf("\n=== Main Loop: SDI-12 Command Processing ===\n");
    printf("RTD measurements are only taken when requested via SDI-12\n");

    // Main loop - handle SDI-12 commands only
    while (true) {
        // Process SDI-12 commands
        sdi12_task();

        // Small delay to prevent busy-waiting
        sleep_ms(1);
    }
}