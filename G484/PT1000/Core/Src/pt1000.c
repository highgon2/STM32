#include "main.h"
#include "math.h"

#define VREF                 (3000.0)
#define ADC_12BIT_RESOLUTION (4096.0)

#define PT1000_A             (3.9083e-3)
#define PT1000_B             (-5.775e-7)
#define PT1000_C             (-4.183e-12)
#define PT1000_R0            (1000)
#define OHM_MAGINE           (5)

static float get_resistor_value(float adc_volt)
{
  uint32_t resistor = (uint32_t)(((PT1000_R0 - OHM_MAGINE) * adc_volt / (VREF - adc_volt)) * 100);
  return resistor / 100.0; 
}

static float get_adc_out_volt(uint8_t index)
{
  uint16_t value = adc_get_value(index);

  if (value == 0) {
    return 0;
  }

  return ((value * VREF) / ADC_12BIT_RESOLUTION);
}

static float calc_temperature(float Cur_R)
{
  /**
   * PT1000 온도 계산
   *
   *  A     = 3.9083e-3
   *  B     = -5.775e-7
   *  C     = -4.183e-12
   *  R0    = 1000
   *  Cur_R = PT1000 저항값
   *
   *  T = (sqrt(A^2 - (4 * B * (1-(Cur_R / R0))) - A) / (2 * B)
   */

  // float Cur_R = 1077.95; // 20도
  // float Cur_R = 1143.81; // 37도
  // float Cur_R = 723.34; // -70도

  // Cur_R = 723.34;

  float t1 = PT1000_A * PT1000_A;
  float t2 = 4 * PT1000_B * (1 - (Cur_R / PT1000_R0));
  float t3 = 2 * PT1000_B;
  float temp = (sqrt(t1 - t2) - PT1000_A) / t3;
  // debug("Cur_R = %.2f, Temp = %.2f", Cur_R, temp);

  return temp;  
}

int16_t pt1000_get_temperature(uint8_t index)
{
  float out_volt = get_adc_out_volt(index);
  float temperature = calc_temperature(get_resistor_value(out_volt));
  printf("index = %d, out_volt = % 4.2f, r = % 4.2f, temp = %.2f\r\n", index, out_volt, get_resistor_value(out_volt), temperature);
  return (int16_t)(temperature * 100);
 
  return 0;
}
