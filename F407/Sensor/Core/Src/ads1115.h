#ifndef __ADS1115_H__
#define __ADS1115_H__

/* ADS1115 Definitions */
#define ADS1115_OS             (0b1 << 7) // Default

#define ADS1115_MUX_AIN0       (0b100 << 4) // Analog input 1
#define ADS1115_MUX_AIN1       (0b101 << 4) // Analog input 2
#define ADS1115_MUX_AIN2       (0b110 << 4) // Analog input 3
#define ADS1115_MUX_AIN3       (0b111 << 4) // Analog input 4

#define ADS1115_PGA_TWOTHIRDS  (0b000 << 1) // 2/3x Gain	-- 0.1875 mV by one bit		MAX: +- VDD + 0.3V
#define ADS1115_PGA_ONE        (0b001 << 1) // 1x Gain		-- 0.125 mV by one bit		MAX: +- VDD + 0.3V
#define ADS1115_PGA_TWO        (0b010 << 1) // 2x Gain		-- 0.0625 mV by one bit		MAX: +- 2.048 V
#define ADS1115_PGA_FOUR       (0b011 << 1) // 4x Gain		-- 0.03125 mV by one bit	MAX: +- 1.024 V
#define ADS1115_PGA_EIGHT      (0b100 << 1) // 8x Gain		-- 0.015625 mV by one bit	MAX: +- 0.512 V
#define ADS1115_PGA_SIXTEEN    (0b111 << 1) // 16x Gain		-- 0.0078125 mV by one bit	MAX: +- 0.256 V

#define ADS1115_MODE           (0b1) // Default : 0b1

#define ADS1115_DATA_RATE_8    (0b000 << 5) // 8SPS
#define ADS1115_DATA_RATE_16   (0b001 << 5) // 16SPS
#define ADS1115_DATA_RATE_32   (0b010 << 5) // 32SPS
#define ADS1115_DATA_RATE_64   (0b011 << 5) // 64SPS
#define ADS1115_DATA_RATE_128  (0b100 << 5) // 128SPS
#define ADS1115_DATA_RATE_250  (0b101 << 5) // 250SPS
#define ADS1115_DATA_RATE_475  (0b110 << 5) // 475SPS
#define ADS1115_DATA_RATE_860  (0b111 << 5) // 860SPS

#define ADS1115_COMP_MODE      (0b0 << 4) // Default
#define ADS1115_COMP_POL       (0b0 << 3) // Default
#define ADS1115_COMP_LAT       (0b0 << 2) // Default
#define ADS1115_COMP_QUE       (0b11)     // Default (ALERT disable)

/* ADS1115 register configurations */
#define ADS1115_CONVER_REG     0x00
#define ADS1115_CONFIG_REG     0x01
#define ADS1115_LOWTHRESH_REG  0x02
#define ADS1115_HIGHTHRESH_REG 0x03

/* TIMEOUT */
#define ADS1115_TIMEOUT        1 // Timeout for HAL I2C functions.

/* EVAR Defines */
typedef enum {
  CONFIG_FSR_6144V = 0,
  CONFIG_FSR_4096V = 1,
  CONFIG_FSR_2048V = 2,
  CONFIG_FSR_1024V = 3,
  CONFIG_FSR_0512V = 4,
  CONFIG_FSR_0256V = 5,
} ads1115_fsr_e;

typedef enum {
  CONFIG_DATA_RATE_8 = 0,
  CONFIG_DATA_RATE_16 = 1,
  CONFIG_DATA_RATE_32 = 2,
  CONFIG_DATA_RATE_64 = 3,
  CONFIG_DATA_RATE_128 = 4,
  CONFIG_DATA_RATE_250 = 5,
  CONFIG_DATA_RATE_475 = 6,
  CONFIG_DATA_RATE_860 = 7,
} ads1115_ds_e;

uint32_t ads1115_init(void);
uint32_t ads1115_set_config(uint8_t fsr, uint8_t sample);
uint32_t ads1115_set_threshold(int16_t low, int16_t high);
uint32_t ads1115_set_temperature(uint8_t port);
uint32_t ads1115_get_temperature(float* value);
uint32_t ads1115_read_temperature(uint8_t port, float* value);

#endif
