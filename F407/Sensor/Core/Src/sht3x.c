#include "main.h"

#include "sht3x.h"

static uint8_t sht3x_calculate_crc(uint8_t* data, uint16_t length)
{
  uint8_t crc = 0xFF;

  for (int i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if ((crc & 0x80) != 0) {
        crc = (uint8_t)((uint8_t)(crc << 1) ^ 0x31);
      }
      else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

uint32_t sht3x_reset(void)
{
  uint8_t dummy = 0x00;
  return i2c_write(HUMI_SENSOR, SHT3x_SOFT_RESET, 2, &dummy, sizeof(dummy), 20);
}

uint32_t sht3x_read_status_register(uint8_t* status)
{
  uint8_t crc = 0;
  uint8_t data[3] = {
      0,
  };

  if (i2c_read(HUMI_SENSOR, SHT3x_READ_STATUS, 2, data, sizeof(data), 10) != 0) {
    return 1;
  }

  crc = sht3x_calculate_crc(data, 2);
  if (crc != data[2]) {
    return 1;
  }

  if (status != NULL) {
    memcpy(status, data, 2);
  }

  return 0;
}

uint32_t sht3x_read_temperature_humidity(float* temperature, float* humidity)
{
  uint8_t data[6] = {0, };
  uint8_t crc_temper;
  uint8_t crc_humid;
  uint16_t raw_temper;
  uint16_t raw_humid;

  if (i2c_read(HUMI_SENSOR, SHT3x_MEASURE_HIGHREP_STRETCH, 2, data, sizeof(data), 20) != 0) {
    return 1;
  }

  crc_temper = sht3x_calculate_crc(data, 2);
  crc_humid = sht3x_calculate_crc(&data[3], 2);
  if (crc_temper != data[2] || crc_humid != data[5]) {
    return 1;
  }

  raw_temper = (uint16_t)((data[0] << 8) | data[1]);
  raw_humid = (uint16_t)((data[3] << 8) | data[4]);

  *temperature = -45.0f + (175.0f * (raw_temper / 65535.0f));
  *humidity = 100.0f * (raw_humid / 65535.0f);

  return 0;
}

uint32_t sht3x_clear_status_register(void)
{
  uint8_t dummy = 0;
  return i2c_write(HUMI_SENSOR, SHT3x_CLEAR_STATUS, 2, &dummy, sizeof(dummy), 20);
}

uint32_t sht3x_set_heater_enable(uint8_t enable)
{
  uint8_t dummy = 0;
  uint16_t command = SHT3x_HEATER_DISABLE;

  if (enable) {
    command = SHT3x_HEATER_ENABLE;
  }
  return i2c_write(HUMI_SENSOR, command, 2, &dummy, sizeof(dummy), 50);
}

uint32_t sht3x_init(void)
{
  if (sht3x_reset() != 0) {
    return 1;
  }

  return sht3x_read_status_register(NULL);
}
