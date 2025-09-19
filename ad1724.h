#ifndef AD1724_H
#define AD1724_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// ADC Register Definitions
#define ADC_COMMS_REG   0x00
#define ADC_ID_REG      0x05
#define ADC_ID_VALUE    0x14

// Function declarations
void adc_reg_write(uint8_t reg_addr, uint8_t *data, size_t data_len);
void adc_reg_read(uint8_t reg_addr, uint8_t *data, size_t data_len);
bool adc_verify_communication(void);

#endif // AD1724_H