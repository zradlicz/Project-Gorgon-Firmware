#include "ad1724.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

// SPI Defines
extern spi_inst_t *SPI_PORT;
extern const uint PIN_CS;

void adc_reg_write(uint8_t reg_addr, uint8_t *data, size_t data_len) {
    uint8_t comm_cmd = reg_addr & 0x3F; // Write command (bit 6 = 0)

    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, &comm_cmd, 1);
    if (data != NULL && data_len > 0) {
        spi_write_blocking(SPI_PORT, data, data_len);
    }
    gpio_put(PIN_CS, 1);
}

void adc_reg_read(uint8_t reg_addr, uint8_t *data, size_t data_len) {
    uint8_t comm_cmd = 0x40 | (reg_addr & 0x3F); // Read command (bit 6 = 1)

    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, &comm_cmd, 1);
    if (data != NULL && data_len > 0) {
        spi_read_blocking(SPI_PORT, 0x00, data, data_len);
    }
    gpio_put(PIN_CS, 1);
}

void adc_reset(void) {
    printf("Performing ADC reset (64 consecutive 1s)...\n");

    // Send 64 consecutive 1s with CS low to reset the ADC
    gpio_put(PIN_CS, 0);
    uint8_t reset_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // 64 bits of 1s
    spi_write_blocking(SPI_PORT, reset_data, 8);
    gpio_put(PIN_CS, 1);

    // Wait for reset to complete
    sleep_ms(100);

    printf("ADC reset complete\n");
}

bool adc_verify_communication(void) {
    uint8_t id_response = 0;
    adc_reg_read(AD7124_ID_REG, &id_response, 1);
    printf("AD7124 ID register read: 0x%02X (expected: 0x%02X)\n", id_response, AD7124_ID_VALUE);
    return (id_response == AD7124_ID_VALUE);
}

bool adc_configure_rtd(const rtd_config_t *config) {
    printf("Configuring AD7124 for RTD measurement...\n");

    // Step 0: Perform proper ADC reset
    adc_reset();

    // Step 1: Configure excitation current
    printf("Setting excitation current to %d on AIN%d\n", config->excitation_current, config->iout_pin);
    uint8_t ioc1_data[3] = {0};
    uint32_t ioc1_val = (config->excitation_current << 8) | config->iout_pin;
    ioc1_data[0] = (ioc1_val >> 16) & 0xFF;
    ioc1_data[1] = (ioc1_val >> 8) & 0xFF;
    ioc1_data[2] = ioc1_val & 0xFF;
    adc_reg_write(AD7124_IOC_REG1, ioc1_data, 3);
    printf("IO_CONTROL_1 register written: 0x%06X\n", ioc1_val);

    // Step 2: Disable channels 3-15
    printf("Disabling channels 3-15...\n");
    uint8_t ch_disable[2] = {0x00, 0x01}; // Disabled, but keep default AINP/AINM
    for (int ch = 3; ch <= 15; ch++) {
        adc_reg_write(AD7124_CH0_MAP_REG + ch, ch_disable, 2);
    }

    // Step 3: Configure Channels 0-2 for RTD measurements (RTDs 1-3)
    // Initially, only enable channel 0. We'll dynamically enable channels later.
    // Channel 0: RTD 1 (AIN1/AIN2) - ENABLED
    printf("Configuring Channel 0: RTD 1 on AIN1(+)/AIN2(-) - ENABLED\n");
    uint8_t ch0_data[2] = {0};
    uint16_t ch0_val = AD7124_CH_MAP_REG_CH_ENABLE |
                       AD7124_CH_MAP_REG_SETUP(0) |
                       AD7124_CH_MAP_REG_AINP(AD7124_AIN1) |
                       AD7124_CH_MAP_REG_AINM(AD7124_AIN2);
    ch0_data[0] = (ch0_val >> 8) & 0xFF;
    ch0_data[1] = ch0_val & 0xFF;
    adc_reg_write(AD7124_CH0_MAP_REG, ch0_data, 2);
    printf("Channel 0 register written: 0x%04X\n", ch0_val);

    // Channel 1: RTD 2 (AIN3/AIN4) - DISABLED initially
    printf("Configuring Channel 1: RTD 2 on AIN3(+)/AIN4(-) - DISABLED\n");
    uint8_t ch1_data[2] = {0};
    uint16_t ch1_val = AD7124_CH_MAP_REG_SETUP(0) |  // No CH_ENABLE bit
                       AD7124_CH_MAP_REG_AINP(AD7124_AIN3) |
                       AD7124_CH_MAP_REG_AINM(AD7124_AIN4);
    ch1_data[0] = (ch1_val >> 8) & 0xFF;
    ch1_data[1] = ch1_val & 0xFF;
    adc_reg_write(AD7124_CH0_MAP_REG + 1, ch1_data, 2);
    printf("Channel 1 register written: 0x%04X\n", ch1_val);

    // Channel 2: RTD 3 (AIN5/AIN6) - DISABLED initially
    printf("Configuring Channel 2: RTD 3 on AIN5(+)/AIN6(-) - DISABLED\n");
    uint8_t ch2_data[2] = {0};
    uint16_t ch2_val = AD7124_CH_MAP_REG_SETUP(0) |  // No CH_ENABLE bit
                       AD7124_CH_MAP_REG_AINP(AD7124_AIN5) |
                       AD7124_CH_MAP_REG_AINM(AD7124_AIN6);
    ch2_data[0] = (ch2_val >> 8) & 0xFF;
    ch2_data[1] = ch2_val & 0xFF;
    adc_reg_write(AD7124_CH0_MAP_REG + 2, ch2_data, 2);
    printf("Channel 2 register written: 0x%04X\n", ch2_val);

    // Step 4: Configure Setup 0 for ratiometric measurement using REFIN1
    printf("Configuring Setup 0: REF=REFIN1, PGA=1, Buffers enabled (ratiometric)\n");
    uint8_t cfg0_data[2] = {0};
    uint16_t cfg0_val = AD7124_CFG_REG_REF_BUFP |      // Enable ref buffer +
                        AD7124_CFG_REG_REF_BUFM |      // Enable ref buffer -
                        AD7124_CFG_REG_AIN_BUFP |      // Enable analog input buffer +
                        AD7124_CFG_REG_AIN_BUFM |      // Enable analog input buffer -
                        AD7124_CFG_REG_REF_SEL(AD7124_REFIN1) | // Use REFIN1 as reference
                        AD7124_CFG_REG_PGA(AD7124_PGA_1);       // Gain = 1
    cfg0_data[0] = (cfg0_val >> 8) & 0xFF;
    cfg0_data[1] = cfg0_val & 0xFF;
    adc_reg_write(AD7124_CFG0_REG, cfg0_data, 2);
    printf("Config 0 register written: 0x%04X\n", cfg0_val);

    // Step 4: Configure Filter 0 for better stability and noise rejection
    printf("Configuring Filter 0: SINC4, 50/60Hz rejection, FS=192 (slower, more stable)\n");
    uint8_t filt0_data[3] = {0};
    uint32_t filt0_val = AD7124_FILT_REG_FILTER(AD7124_SINC4_FILTER) |
                         AD7124_FILT_REG_REJ60 |  // Enable 50/60Hz rejection
                         AD7124_FILT_REG_FS(192); // Slower for better stability
    filt0_data[0] = (filt0_val >> 16) & 0xFF;
    filt0_data[1] = (filt0_val >> 8) & 0xFF;
    filt0_data[2] = filt0_val & 0xFF;
    adc_reg_write(AD7124_FILTER0_REG, filt0_data, 3);
    printf("Filter 0 register written: 0x%06X\n", filt0_val);

    // Step 5: Set to standby mode (we'll use single conversion mode, triggered manually)
    printf("Setting ADC Control: External ref (REFIN1), standby mode, low power\n");
    uint8_t adc_ctrl_data[2] = {0};
    uint16_t adc_ctrl_val = AD7124_ADC_CTRL_DATA_STATUS |               // Enable status with data
                            AD7124_ADC_CTRL_POWER_MODE(0) |             // Low power mode (0=low, 1=mid, 2=full)
                            AD7124_ADC_CTRL_MODE(AD7124_MODE_STANDBY) | // Standby mode (we'll trigger single conversions)
                            AD7124_ADC_CTRL_CLK_SEL(0);                 // Internal clock
    // Note: REF_EN=0 because we're using external reference on REFIN1
    adc_ctrl_data[0] = (adc_ctrl_val >> 8) & 0xFF;
    adc_ctrl_data[1] = adc_ctrl_val & 0xFF;
    adc_reg_write(AD7124_ADC_CTRL_REG, adc_ctrl_data, 2);
    printf("ADC Control register written: 0x%04X\n", adc_ctrl_val);

    // Verify configuration by reading back registers
    printf("\n=== Verifying Configuration ===\n");

    // Read back IO_CONTROL_1
    uint8_t ioc1_readback[3];
    adc_reg_read(AD7124_IOC_REG1, ioc1_readback, 3);
    uint32_t ioc1_read = ((uint32_t)ioc1_readback[0] << 16) |
                         ((uint32_t)ioc1_readback[1] << 8) |
                         ioc1_readback[2];
    printf("IO_CONTROL_1 readback: 0x%06X\n", ioc1_read);

    // Read back Channel 0
    uint8_t ch0_readback[2];
    adc_reg_read(AD7124_CH0_MAP_REG, ch0_readback, 2);
    uint16_t ch0_read = ((uint16_t)ch0_readback[0] << 8) | ch0_readback[1];
    printf("Channel 0 readback: 0x%04X\n", ch0_read);

    // Read back Channel 1 to verify it's disabled
    uint8_t ch1_readback[2];
    adc_reg_read(AD7124_CH1_MAP_REG, ch1_readback, 2);
    uint16_t ch1_read = ((uint16_t)ch1_readback[0] << 8) | ch1_readback[1];
    printf("Channel 1 readback: 0x%04X (should be disabled)\n", ch1_read);

    // Read back Config 0
    uint8_t cfg0_readback[2];
    adc_reg_read(AD7124_CFG0_REG, cfg0_readback, 2);
    uint16_t cfg0_read = ((uint16_t)cfg0_readback[0] << 8) | cfg0_readback[1];
    printf("Config 0 readback: 0x%04X\n", cfg0_read);

    // Read back Filter 0
    uint8_t filt0_readback[3];
    adc_reg_read(AD7124_FILTER0_REG, filt0_readback, 3);
    uint32_t filt0_read = ((uint32_t)filt0_readback[0] << 16) |
                          ((uint32_t)filt0_readback[1] << 8) |
                          filt0_readback[2];
    printf("Filter 0 readback: 0x%06X\n", filt0_read);

    // Read back ADC Control
    uint8_t ctrl_readback[2];
    adc_reg_read(AD7124_ADC_CTRL_REG, ctrl_readback, 2);
    uint16_t ctrl_read = ((uint16_t)ctrl_readback[0] << 8) | ctrl_readback[1];
    printf("ADC Control readback: 0x%04X\n", ctrl_read);

    // Check error register
    uint8_t error_readback[3];
    adc_reg_read(AD7124_ERROR_REG, error_readback, 3);
    uint32_t error_read = ((uint32_t)error_readback[0] << 16) |
                          ((uint32_t)error_readback[1] << 8) |
                          error_readback[2];
    printf("Error register: 0x%06X\n", error_read);

    printf("================================\n\n");

    printf("RTD configuration complete!\n");

    return true;
}

void adc_enable_single_channel(uint8_t channel) {
    // Enable only the specified channel (0, 1, or 2), disable all others
    printf("Enabling only ADC channel %d...\n", channel);

    // Channel 0: RTD 1 (AIN1/AIN2)
    uint8_t ch0_data[2];
    uint16_t ch0_val;
    if (channel == 0) {
        ch0_val = AD7124_CH_MAP_REG_CH_ENABLE |
                  AD7124_CH_MAP_REG_SETUP(0) |
                  AD7124_CH_MAP_REG_AINP(AD7124_AIN1) |
                  AD7124_CH_MAP_REG_AINM(AD7124_AIN2);
    } else {
        ch0_val = AD7124_CH_MAP_REG_SETUP(0) |  // Disabled (no CH_ENABLE bit)
                  AD7124_CH_MAP_REG_AINP(AD7124_AIN1) |
                  AD7124_CH_MAP_REG_AINM(AD7124_AIN2);
    }
    ch0_data[0] = (ch0_val >> 8) & 0xFF;
    ch0_data[1] = ch0_val & 0xFF;
    adc_reg_write(AD7124_CH0_MAP_REG, ch0_data, 2);

    // Channel 1: RTD 2 (AIN3/AIN4)
    uint8_t ch1_data[2];
    uint16_t ch1_val;
    if (channel == 1) {
        ch1_val = AD7124_CH_MAP_REG_CH_ENABLE |
                  AD7124_CH_MAP_REG_SETUP(0) |
                  AD7124_CH_MAP_REG_AINP(AD7124_AIN3) |
                  AD7124_CH_MAP_REG_AINM(AD7124_AIN4);
    } else {
        ch1_val = AD7124_CH_MAP_REG_SETUP(0) |
                  AD7124_CH_MAP_REG_AINP(AD7124_AIN3) |
                  AD7124_CH_MAP_REG_AINM(AD7124_AIN4);
    }
    ch1_data[0] = (ch1_val >> 8) & 0xFF;
    ch1_data[1] = ch1_val & 0xFF;
    adc_reg_write(AD7124_CH0_MAP_REG + 1, ch1_data, 2);

    // Channel 2: RTD 3 (AIN5/AIN6)
    uint8_t ch2_data[2];
    uint16_t ch2_val;
    if (channel == 2) {
        ch2_val = AD7124_CH_MAP_REG_CH_ENABLE |
                  AD7124_CH_MAP_REG_SETUP(0) |
                  AD7124_CH_MAP_REG_AINP(AD7124_AIN5) |
                  AD7124_CH_MAP_REG_AINM(AD7124_AIN6);
    } else {
        ch2_val = AD7124_CH_MAP_REG_SETUP(0) |
                  AD7124_CH_MAP_REG_AINP(AD7124_AIN5) |
                  AD7124_CH_MAP_REG_AINM(AD7124_AIN6);
    }
    ch2_data[0] = (ch2_val >> 8) & 0xFF;
    ch2_data[1] = ch2_val & 0xFF;
    adc_reg_write(AD7124_CH0_MAP_REG + 2, ch2_data, 2);
}

void adc_start_single_conversion(void) {
    // Start a single conversion by writing to ADC_CONTROL register
    uint8_t adc_ctrl_data[2];
    uint16_t adc_ctrl_val = AD7124_ADC_CTRL_DATA_STATUS |               // Enable status with data
                            AD7124_ADC_CTRL_POWER_MODE(0) |             // Low power mode
                            AD7124_ADC_CTRL_MODE(AD7124_MODE_SINGLE) |  // Single conversion mode
                            AD7124_ADC_CTRL_CLK_SEL(0);                 // Internal clock
    adc_ctrl_data[0] = (adc_ctrl_val >> 8) & 0xFF;
    adc_ctrl_data[1] = adc_ctrl_val & 0xFF;
    adc_reg_write(AD7124_ADC_CTRL_REG, adc_ctrl_data, 2);
}

bool adc_read_rtd_data(uint32_t *rtd_data, uint8_t *channel) {
    // Check if data is ready
    uint8_t status;
    adc_reg_read(AD7124_STATUS_REG, &status, 1);

    printf("Status register: 0x%02X (RDY bit: %s)\n", status, (status & 0x80) ? "NOT READY" : "READY");

    if (status & 0x80) { // RDY bit is high, no data ready
        return false;
    }

    printf("Data ready! Reading data register...\n");

    // Since DATA_STATUS is enabled, read 4 bytes (24-bit data + 8-bit status)
    uint8_t data_bytes[4] = {0};
    adc_reg_read(AD7124_DATA_REG, data_bytes, 4);

    // Extract 24-bit ADC data (first 3 bytes)
    *rtd_data = ((uint32_t)data_bytes[0] << 16) |
                ((uint32_t)data_bytes[1] << 8) |
                data_bytes[2];

    // Get channel from appended status byte (4th byte)
    *channel = data_bytes[3] & 0x0F;

    printf("Raw data bytes: 0x%02X 0x%02X 0x%02X 0x%02X\n", data_bytes[0], data_bytes[1], data_bytes[2], data_bytes[3]);
    printf("RTD Data: 0x%06X (%d), Channel: %d (from data read)\n", *rtd_data, *rtd_data, *channel);

    return true;
}

float adc_calculate_resistance(uint32_t rtd_data, const rtd_config_t *config) {
    // Convert 24-bit unsigned to voltage ratio (ratiometric measurement)
    float ratio = (float)rtd_data / 16777216.0f; // 2^24

    // Calculate RTD resistance: R_rtd = R_ref * ratio
    float r_rtd = config->r_ref * ratio;

    printf("RTD resistance calculated: %.2f ohms (ratio: %.6f)\n", r_rtd, ratio);

    return r_rtd;
}

float adc_calculate_temperature(uint32_t rtd_data, const rtd_config_t *config) {
    // Calculate resistance first
    float r_rtd = adc_calculate_resistance(rtd_data, config);

    // Calculate temperature using simplified RTD equation: R(T) = R0(1 + α*T)
    // Therefore: T = (R(T)/R0 - 1) / α
    float temperature = (r_rtd / config->r_rtd_0 - 1.0f) / config->alpha;

    printf("Temperature calculated: %.2f°C\n", temperature);

    return temperature;
}