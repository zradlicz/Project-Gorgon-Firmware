#ifndef AD7124_H
#define AD7124_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// AD7124 Register Addresses
#define AD7124_COMMS_REG        0x00
#define AD7124_STATUS_REG       0x00
#define AD7124_ADC_CTRL_REG     0x01
#define AD7124_DATA_REG         0x02
#define AD7124_IOC_REG1         0x03
#define AD7124_IOC_REG2         0x04
#define AD7124_ID_REG           0x05
#define AD7124_ERROR_REG        0x06
#define AD7124_ERR_EN_REG       0x07
#define AD7124_MCLK_CNT_REG     0x08
#define AD7124_CH0_MAP_REG      0x09
#define AD7124_CH1_MAP_REG      0x0A
#define AD7124_CFG0_REG         0x19
#define AD7124_CFG1_REG         0x1A
#define AD7124_FILTER0_REG      0x21
#define AD7124_FILTER1_REG      0x22
#define AD7124_OFFS0_REG        0x29
#define AD7124_OFFS1_REG        0x2A
#define AD7124_GAIN0_REG        0x31
#define AD7124_GAIN1_REG        0x32

#define AD7124_ID_VALUE         0x17

// ADC Control Register bits
#define AD7124_ADC_CTRL_DOUT_RDY_DEL    (1 << 12)
#define AD7124_ADC_CTRL_CONT_READ       (1 << 11)
#define AD7124_ADC_CTRL_DATA_STATUS     (1 << 10)
#define AD7124_ADC_CTRL_CS_EN           (1 << 9)
#define AD7124_ADC_CTRL_REF_EN          (1 << 8)
#define AD7124_ADC_CTRL_POWER_MODE(x)   (((x) & 0x3) << 6)
#define AD7124_ADC_CTRL_MODE(x)         (((x) & 0xF) << 2)
#define AD7124_ADC_CTRL_CLK_SEL(x)      (((x) & 0x3) << 0)

// Operating modes
#define AD7124_MODE_CONTINUOUS          0x0
#define AD7124_MODE_SINGLE              0x1
#define AD7124_MODE_STANDBY             0x2
#define AD7124_MODE_POWER_DOWN          0x3
#define AD7124_MODE_IDLE                0x4
#define AD7124_MODE_INT_ZERO_CAL        0x5
#define AD7124_MODE_INT_FULL_CAL        0x6
#define AD7124_MODE_SYS_ZERO_CAL        0x7
#define AD7124_MODE_SYS_FULL_CAL        0x8

// Channel map register bits
#define AD7124_CH_MAP_REG_CH_ENABLE     (1 << 15)
#define AD7124_CH_MAP_REG_SETUP(x)      (((x) & 0x7) << 12)
#define AD7124_CH_MAP_REG_AINP(x)       (((x) & 0x1F) << 5)
#define AD7124_CH_MAP_REG_AINM(x)       (((x) & 0x1F) << 0)

// Analog input pins
#define AD7124_AIN0     0x00
#define AD7124_AIN1     0x01
#define AD7124_AIN2     0x02
#define AD7124_AIN3     0x03
#define AD7124_AIN4     0x04
#define AD7124_AIN5     0x05
#define AD7124_AIN6     0x06
#define AD7124_AIN7     0x07
#define AD7124_AIN8     0x08
#define AD7124_TEMP     0x10  // Internal temperature sensor
#define AD7124_AVSS     0x11

// Configuration register bits
#define AD7124_CFG_REG_BIPOLAR          (1 << 11)
#define AD7124_CFG_REG_BURNOUT(x)       (((x) & 0x3) << 9)
#define AD7124_CFG_REG_REF_BUFP         (1 << 8)
#define AD7124_CFG_REG_REF_BUFM         (1 << 7)
#define AD7124_CFG_REG_AIN_BUFP         (1 << 6)
#define AD7124_CFG_REG_AIN_BUFM         (1 << 5)
#define AD7124_CFG_REG_REF_SEL(x)       (((x) & 0x3) << 3)
#define AD7124_CFG_REG_PGA(x)           (((x) & 0x7) << 0)

// Reference selection
#define AD7124_REFIN1   0x0
#define AD7124_REFIN2   0x1
#define AD7124_INT_REF  0x2
#define AD7124_AVDD     0x3

// PGA gain settings
#define AD7124_PGA_1    0x0
#define AD7124_PGA_2    0x1
#define AD7124_PGA_4    0x2
#define AD7124_PGA_8    0x3
#define AD7124_PGA_16   0x4
#define AD7124_PGA_32   0x5
#define AD7124_PGA_64   0x6
#define AD7124_PGA_128  0x7

// IO Control Register 1 bits
#define AD7124_IOC1_IOUT1(x)           (((x) & 0x7) << 11)
#define AD7124_IOC1_IOUT0(x)           (((x) & 0x7) << 8)
#define AD7124_IOC1_IOUT1_CH(x)        (((x) & 0xF) << 4)
#define AD7124_IOC1_IOUT0_CH(x)        (((x) & 0xF) << 0)

// Excitation current values
#define AD7124_IOUT_OFF     0x0
#define AD7124_IOUT_50UA    0x1
#define AD7124_IOUT_100UA   0x2
#define AD7124_IOUT_250UA   0x3
#define AD7124_IOUT_500UA   0x4
#define AD7124_IOUT_750UA   0x5
#define AD7124_IOUT_1000UA  0x6

// Filter register bits
#define AD7124_FILT_REG_FILTER(x)       (((x) & 0x7) << 21)
#define AD7124_FILT_REG_REJ60           (1 << 20)
#define AD7124_FILT_REG_POST_FILTER(x)  (((x) & 0x7) << 17)
#define AD7124_FILT_REG_SINGLE_CYCLE    (1 << 16)
#define AD7124_FILT_REG_FS(x)           (((x) & 0x7FF) << 0)

// Filter types
#define AD7124_SINC4_FILTER     0x0
#define AD7124_SINC3_FILTER     0x2
#define AD7124_FAST_FILTER      0x4
#define AD7124_POST_FILTER      0x7

// RTD configuration structure
typedef struct {
    float r_ref;            // Reference resistor value in ohms (e.g., 4990.0)
    float r_rtd_0;          // RTD resistance at 0°C in ohms (e.g., 100.0 for PT100)
    float alpha;            // RTD temperature coefficient (e.g., 0.00385 for PT100)
    uint8_t excitation_current; // Excitation current setting
    uint8_t rtd_ainp;       // RTD positive input pin
    uint8_t rtd_ainm;       // RTD negative input pin
    uint8_t ref_ainp;       // Reference positive input pin
    uint8_t ref_ainm;       // Reference negative input pin
    uint8_t iout_pin;       // Excitation current output pin
} rtd_config_t;

// Function declarations
void adc_reg_write(uint8_t reg_addr, uint8_t *data, size_t data_len);
void adc_reg_read(uint8_t reg_addr, uint8_t *data, size_t data_len);
void adc_reset(void);
bool adc_verify_communication(void);
bool adc_configure_rtd(const rtd_config_t *config);
void adc_enable_single_channel(uint8_t channel);
void adc_start_single_conversion(void);
bool adc_read_rtd_data(uint32_t *rtd_data, uint8_t *channel);
float adc_calculate_temperature(uint32_t rtd_data, const rtd_config_t *config);
float adc_calculate_resistance(uint32_t rtd_data, const rtd_config_t *config);
float adc_read_internal_temperature(void);
float pico_read_internal_temperature(void);

#endif // AD7124_H