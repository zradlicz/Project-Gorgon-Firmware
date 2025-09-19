#include "ad1724.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

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

bool adc_verify_communication(void) {
    uint8_t id_response = 0;
    adc_reg_read(ADC_ID_REG, &id_response, 1);
    return (id_response == ADC_ID_VALUE);
}