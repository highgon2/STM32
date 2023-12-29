#include "main.h"
#include "ads1115.h"
#include "math.h"

static uint8_t pga = ADS1115_PGA_TWO;             // default
static uint8_t data_rate = ADS1115_DATA_RATE_128; // default

static float ads1115_get_volate(uint8_t pga)
{
  switch (pga) {
  case ADS1115_PGA_TWOTHIRDS:
    return 0.1875;
  case ADS1115_PGA_ONE:
    return 0.125;
  case ADS1115_PGA_TWO:
    return 0.0625;
  case ADS1115_PGA_FOUR:
    return 0.03125;
  case ADS1115_PGA_EIGHT:
    return 0.015625;
  case ADS1115_PGA_SIXTEEN:
    return (float)0.007815;
  }

  return 0;
}

uint32_t ads1115_set_config(uint8_t fsr, uint8_t sample)
{
  switch (fsr) {
  case CONFIG_FSR_6144V:
    pga = ADS1115_PGA_TWOTHIRDS;
    break;
  case CONFIG_FSR_4096V:
    pga = ADS1115_PGA_ONE;
    break;
  case CONFIG_FSR_2048V:
    pga = ADS1115_PGA_TWO;
    break;
  case CONFIG_FSR_1024V:
    pga = ADS1115_PGA_FOUR;
    break;
  case CONFIG_FSR_0512V:
    pga = ADS1115_PGA_EIGHT;
    break;
  case CONFIG_FSR_0256V:
    pga = ADS1115_PGA_SIXTEEN;
    break;

  default:
    printf("WARN :: %s() : invalid fsr : %d\r\n", __func__, fsr);
    return 1;
  }

  switch (sample) {
  case CONFIG_DATA_RATE_8:
    data_rate = ADS1115_DATA_RATE_8;
    break;
  case CONFIG_DATA_RATE_16:
    data_rate = ADS1115_DATA_RATE_16;
    break;
  case CONFIG_DATA_RATE_32:
    data_rate = ADS1115_DATA_RATE_32;
    break;
  case CONFIG_DATA_RATE_64:
    data_rate = ADS1115_DATA_RATE_64;
    break;
  case CONFIG_DATA_RATE_128:
    data_rate = ADS1115_DATA_RATE_128;
    break;
  case CONFIG_DATA_RATE_250:
    data_rate = ADS1115_DATA_RATE_250;
    break;
  case CONFIG_DATA_RATE_475:
    data_rate = ADS1115_DATA_RATE_475;
    break;
  case CONFIG_DATA_RATE_860:
    data_rate = ADS1115_DATA_RATE_860;
    break;

  default:
    printf("WARN :: %s() : invalid sample : %d\r\n", __func__, fsr);
    return 1;
  }

  return 0;
}

uint32_t ads1115_set_threshold(int16_t low, int16_t high)
{
  uint8_t data[2];
  uint32_t error_code = 0;

  if (low < 0x8000 || high > 0x7FFF) {
    return 1;
  }

  data[0] = (low & 0xFF00) >> 8;
  data[1] = (low & 0x00FF);
  error_code |= i2c_write(TEMP_SENSOR, ADS1115_LOWTHRESH_REG, 1, data, 2, 10);

  data[0] = (high & 0xFF00) >> 8;
  data[1] = (high & 0x00FF);
  error_code |= i2c_write(TEMP_SENSOR, ADS1115_HIGHTHRESH_REG, 1, data, 2, 10);

  return error_code;
}

uint32_t ads1115_set_temperature(uint8_t port)
{
  uint8_t mux_port = 0;
  uint8_t config[2] = {
      0,
  };

  switch (port) {
  case 0:
    mux_port = ADS1115_MUX_AIN3;
    break;
  case 1:
    mux_port = ADS1115_MUX_AIN2;
    break;
  case 2:
    mux_port = ADS1115_MUX_AIN1;
    break;
  case 3:
    mux_port = ADS1115_MUX_AIN0;
    break;
  default:
    printf("ERROR :: %s() : invalid port : %d\r\n", __func__, port);
    return 1;
  }

  config[0] = ADS1115_OS | mux_port | pga | ADS1115_MODE;
  config[1] = data_rate | ADS1115_COMP_MODE | ADS1115_COMP_POL | ADS1115_COMP_LAT | ADS1115_COMP_QUE;

  if (i2c_write(TEMP_SENSOR, ADS1115_CONFIG_REG, 1, config, 2, 10) != 0) {
    return 1;
  }

  return 0;
}

uint32_t ads1115_get_temperature(float* value)
{
  uint8_t data[2];
  int16_t adc_value;
  float voltage = 0;
  float resistor = 0;

  if (i2c_read(TEMP_SENSOR, ADS1115_CONVER_REG, 1, data, 2, 10) != 0) {
    return 1;
  }

  /**
   * NTC 온도센서 저항(R2) 계산 (https://openstory.tistory.com/230)
   *   R1   = 10Kohm
   *   V1   = 5267 (입력전압)
   *   Vout = 저항을 거친 출력전압
   *
   *   R2 = (R1 * Vout) / (V1 - Vout)
   *      = ((10 * 1000) * voltage) / (5267 - voltage)
   *
   *  NTC 저항(R2)에 따른 온도(T2) 계산 (https://m.blog.naver.com/sst-korea/221875675521)
   *    B  = 3435 (NTCLE413E2103F520L)
   *    T1 = 기준온도 25
   *    TK = 기준온도의 절대온도 값 273 + 25 = 298
   *    R1 = 10Kohm
   *
   *    T2 = B / ((B / TK) - log(R1 / R2))
   *       = 3435 / ((3435 / 298) - log((10 * 1000) / resistor))
   */
  adc_value = (int16_t)(data[0] << 8) | data[1];
  voltage = adc_value * ads1115_get_volate(pga); // mV

  if (voltage > 5200) {
    *value = 0;
  }
  else {
    resistor = ((10 * 1000) * voltage) / (5267 - voltage);
    *value = (3435.0 / ((3435.0 / 298.0) - log((10 * 1000) / resistor))) - 273;
  }

  return 0;
}

uint32_t ads1115_read_temperature(uint8_t port, float* value)
{
  if (ads1115_set_temperature(port) == 0) {
    HAL_Delay(10); /* minimum conversion time */
    ads1115_get_temperature(value);
  }

  return 1;
}

uint32_t ads1115_init(void)
{
  return ads1115_set_config(CONFIG_FSR_6144V, CONFIG_DATA_RATE_128);
}
