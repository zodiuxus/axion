#ifndef INFO_H
#define INFO_H

#include "soc/gpio_num.h"

// DS18B20 (1-Wire)
#define PIN_DS18B20 GPIO_NUM_4

#ifdef __cplusplus
extern "C" {
#endif

extern float temp_values[8];  // Declare the global variable

#ifdef __cplusplus
}
#endif

#endif  // INFO_H

