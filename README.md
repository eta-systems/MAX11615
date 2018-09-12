# MAX11615 C Library

C library for the MAX11615EEE A/D multiplexer using STM32 HAL.

This library is made for the STM32 HAL (Hardware Abstraction Library) platform. The example code is for STM32CubeMX and Keil uVision 5 IDE.

---

### Usage

```c
/* USER CODE BEGIN Includes */
#include "max11615.h"          // include the library
```

```c
/* USER CODE BEGIN 0 */
#define ADDRESS_MAX11615  0x66
MAX11615 adcDriver_1;
/* USER CODE END 0 */

int main(void)
{
    /* USER CODE BEGIN 2 */

    // initialize the adcDriver_1 with the I2C handle, the I2C address \
       and the settings for analog reference according to the datasheet
    // analog ref: (internal) + (reference not connected) \
                    + (internal reference always on)
    MAX11615_Init(&adcDriver_1, &hi2c1, ADDRESS_MAX11615, 4+2+1);

    uint8_t adc_value = 0;
    MAX11615_ADC_Read(&adcDriver_1, 0, adc_value); 
    // read adc value from channel 0 and print to console
    printf("ADC Value CH0: %d\n", adc_value);

}
```

---

### Restrictions

The library was developed and tested using a MAX11615 chip.
You might however use it with the other chips from this family too.

- MAX11612
- MAX11613
- MAX11614
- **MAX11615**
- MAX11616
- MAX11617

---

### Sources

MAX11612 Datasheet: [maximintegrated.com](https://datasheets.maximintegrated.com/en/ds/MAX11612-MAX11617.pdf)

MAX11609EEE Breakout Board Arduino Library [github.com/AllAboutEE](https://github.com/AllAboutEE/MAX11609EEE-Breakout-Board/tree/master/Software/Arduino/AllAboutEE-MAX11609-Library)

---

### License

MIT License

Copyright (c) 2018 ETA Systems

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.



