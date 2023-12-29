#ifndef __SHT3x_H__
#define __SHT3x_H__

#define SHT3x_SOFT_RESET                0x30A2
#define SHT3x_CLEAR_STATUS              0x3041

/* Single Shot Measurement */
#define SHT3x_MEASURE_HIGHREP_STRETCH   0x2C06
#define SHT3x_MEASURE_MEDIUMREP_STRETCH 0x2C0D
#define SHT3x_MEASURE_LOWREP_STRETCH    0x2C0D

#define SHT3x_MEASURE_HIGHREP           0x2400
#define SHT3x_MEASURE_MEDIUMREP         0x240B
#define SHT3x_MEASURE_LOWREP            0x2416

/* Periodic Measurement */
#define SHT3x_MEASURE_HIGHREP_10HZ      0x2737
#define SHT3x_MEASURE_LOWREP_10HZ       0x272A

/* Break command Periodic measurement */
#define SHT3x_BREAK_MEASURE             0x3093

#define SHT3x_HEATER_ENABLE             0x306D
#define SHT3x_HEATER_DISABLE            0x3066

#define SHT3x_FETCH_DATA                0xE000
#define SHT3x_READ_STATUS               0xF32D

uint32_t sht3x_init(void);
uint32_t sht3x_reset(void);
uint32_t sht3x_read_status_register(uint8_t* status);
uint32_t sht3x_read_temperature_humidity(float* temperature, float* humidity);
uint32_t sht3x_clear_status_register(void);
uint32_t sht3x_set_heater_enable(uint8_t enable);

#endif
