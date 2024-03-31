#include "main.h"
#include "sht3x.h"

#include "sht3x.h"

#define I2C_ADDR_SHT3X         (0x44)
#define SHT3X_MEM_SIZE         (2)
#define SHT3X_STATUS_SIZE      (3)
#define SHT3X_TEMPERHUMID_SIZE (6)

static uint8_t crc8(const uint8_t* buf, size_t size)
{
  uint8_t crc = 0xFF;

  for (int i = 0; i < size; i++) {
    crc ^= buf[i];
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
  uint8_t dummy[2];

  if (i2c_write(I2C_ADDR_SHT3X, SHT3x_SOFT_RESET, SHT3X_MEM_SIZE, dummy, sizeof(dummy), 20) != 0) {
    return -1;
  }
  return 0;
}

uint32_t sht3x_set_heater(uint8_t enable)
{
  uint8_t dummy[2];
  uint16_t command = enable ? SHT3x_HEATER_ENABLE : SHT3x_HEATER_DISABLE;

  if (i2c_write(I2C_ADDR_SHT3X, command, SHT3X_MEM_SIZE, dummy, sizeof(dummy), 20) != 0) {
    return -1;
  }
  return 0;
}

uint32_t sht3x_clear_status(void)
{
  uint8_t dummy[2];

  if (i2c_write(I2C_ADDR_SHT3X, SHT3x_CLEAR_STATUS, SHT3X_MEM_SIZE, dummy, sizeof(dummy), 20) != 0) {
    return -1;
  }
  return 0;
}

uint32_t sht3x_init(void)
{
  if (sht3x_reset() != 0) {
    return -1;
  }
  return 0;
}

/**
 * STATUS
 */
uint32_t sht3x_read_status(uint8_t* status)
{
  uint8_t crc;
  uint8_t buf[SHT3X_STATUS_SIZE] = {0, };

  if (i2c_read(I2C_ADDR_SHT3X, SHT3x_READ_STATUS, SHT3X_MEM_SIZE, buf, SHT3X_STATUS_SIZE, 10) != 0) {
    return -1;
  }

  crc = crc8(buf, SHT3X_MEM_SIZE);
  if (crc != buf[2]) {
    printf("ERROR :: %s() : status register crc error : 0x%02x expected(0x%02x)\r\n", __func__, crc, buf[2]);
    return -2;
  }

  memcpy(status, buf, SHT3X_MEM_SIZE);
  return 0;
}

/**
 * temperature: 24.770348, humidity: 37.392231
 *
 * temp_centi: 2477 = 24.77 C
 * humid 3739 = 37 %
 */

static uint8_t buf[SHT3X_TEMPERHUMID_SIZE] = {0, };
static float temperature;
static float humidity;

uint32_t sht3x_rx_callback(void)
{
  uint8_t crc_temper = crc8(buf, SHT3X_MEM_SIZE);
  uint8_t crc_humid = crc8(&buf[3], SHT3X_MEM_SIZE);

  if (crc_temper != buf[2] || crc_humid != buf[5]) {
    printf("ERROR :: %s() crc error : temperature 0x%02x, expected(0x%02x), humidity 0x%02x, expected(0x%02x)\r\n", __func__, crc_temper, buf[2], crc_humid, buf[5]);
    return 1;
  }

  /** REF: SHT3x-DIS: Datasheelt 4.13 Conversion of Singed Output */
  temperature = -45.0f + (175.0f * ((uint16_t)((buf[0] << 8) | buf[1]) / 65535.0f));
  humidity = 100.0f * ((uint16_t)((buf[3] << 8) | buf[4]) / 65535.0f);

  return 0;
}

uint32_t sht3x_read_temperature_humidity(void)
{
  memset(buf, 0, SHT3X_TEMPERHUMID_SIZE);
  if (i2c_read_dma(I2C_ADDR_SHT3X, SHT3x_MEASURE_HIGHREP_STRETCH, SHT3X_MEM_SIZE, buf, SHT3X_TEMPERHUMID_SIZE) != 0) {
    return -1;
  }

  return 0;
}
