/*

    Arduino library for INA3221 current and voltage sensor.

    MIT License

    Copyright (c) 2020 Beast Devices, Andrejs Bondarevs

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.

*/

#ifndef _INA3221_H_
#define _INA3221_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>


/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define INA3221_ADDRESS                         (0x40<<1)    // 1000000 (A0+A1=GND)
#define INA3221_READ                            (0x01)
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER (R/W)
    -----------------------------------------------------------------------*/
#define INA3221_REG_CONFIG                      (0x00)
#define INA3221_CONFIG_RESET                    (0x8000)  // Reset Bit

#define INA3221_CONFIG_ENABLE_CHAN1             (0x4000)  // Enable Channel 1
#define INA3221_CONFIG_ENABLE_CHAN2             (0x2000)  // Enable Channel 2
#define INA3221_CONFIG_ENABLE_CHAN3             (0x1000)  // Enable Channel 3

#define INA3221_CONFIG_AVG2                     (0x0800)  // AVG Samples Bit 2 - See table 3 spec
#define INA3221_CONFIG_AVG1                     (0x0400)  // AVG Samples Bit 1 - See table 3 spec
#define INA3221_CONFIG_AVG0                     (0x0200)  // AVG Samples Bit 0 - See table 3 spec

#define INA3221_CONFIG_VBUS_CT2                 (0x0100)  // VBUS bit 2 Conversion time - See table 4 spec
#define INA3221_CONFIG_VBUS_CT1                 (0x0080)  // VBUS bit 1 Conversion time - See table 4 spec
#define INA3221_CONFIG_VBUS_CT0                 (0x0040)  // VBUS bit 0 Conversion time - See table 4 spec

#define INA3221_CONFIG_VSH_CT2                  (0x0020)  // Vshunt bit 2 Conversion time - See table 5 spec
#define INA3221_CONFIG_VSH_CT1                  (0x0010)  // Vshunt bit 1 Conversion time - See table 5 spec
#define INA3221_CONFIG_VSH_CT0                  (0x0008)  // Vshunt bit 0 Conversion time - See table 5 spec

#define INA3221_CONFIG_MODE_2                   (0x0004)  // Operating Mode bit 2 - See table 6 spec
#define INA3221_CONFIG_MODE_1                   (0x0002)  // Operating Mode bit 1 - See table 6 spec
#define INA3221_CONFIG_MODE_0                   (0x0001)  // Operating Mode bit 0 - See table 6 spec
/*=========================================================================*/

/*=========================================================================
    SHUNT VOLTAGE REGISTER (R)
    -----------------------------------------------------------------------*/
#define INA3221_REG_SHUNTVOLTAGE_1                (0x01)
/*=========================================================================*/

/*=========================================================================
    BUS VOLTAGE REGISTER (R)
    -----------------------------------------------------------------------*/
#define INA3221_REG_BUSVOLTAGE_1                  (0x02)
/*=========================================================================*/
#define SHUNT_100m  (0.1)   // default shunt resistor value of 0.1 Ohm
#define SHUNT_10m  (0.01)   // default shunt resistor value of 0.01 Ohm
#define DC                            (0x00)
#define AC                            (0x01)
typedef struct {
    uint8_t i2caddr;
    float shuntresistor[3];
    uint8_t enable[3];
    uint8_t acdc[3];
} INA3221;
void INA3221_wireWriteRegister(INA3221 *ina3221, uint8_t reg, uint16_t value);
void INA3221_wireReadRegister(INA3221 *ina3221, uint8_t reg, uint16_t *value);
void INA3221SetConfig(INA3221 *ina3221);
void INA3221_init(I2C_HandleTypeDef *_td);
void INA3221_begin(INA3221 *ina3221);
int16_t INA3221_getBusVoltage_raw(INA3221 *ina3221, int channel);
int16_t INA3221_getShuntVoltage_raw(INA3221 *ina3221, int channel);
float INA3221_getShuntVoltage_mV(INA3221 *ina3221, int channel);
float INA3221_getBusVoltage_V(INA3221 *ina3221, int channel);
float INA3221_getCurrent_mA(INA3221 *ina3221, int channel);
int INA3221_getManufID(INA3221 *ina3221);

#ifdef __cplusplus
}
#endif

#endif /* _INA3221_H_ */
