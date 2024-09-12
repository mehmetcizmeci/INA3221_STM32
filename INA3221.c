#include "INA3221.h"
I2C_HandleTypeDef i2cty;

void INA3221_wireWriteRegister(INA3221 *ina3221, uint8_t reg, uint16_t value) {
	uint8_t cmd_buff[3];
    cmd_buff[0] =reg;
	cmd_buff[1] = ((value >> 8) & 0xFF);       // Upper 8-bits
	cmd_buff[2] = (value & 0xFF);

	HAL_I2C_Master_Transmit(&i2cty,ina3221->i2caddr,(uint8_t*)cmd_buff,3,1000);
}

void INA3221_wireReadRegister(INA3221 *ina3221, uint8_t reg, uint16_t *value) {
	uint8_t cmd_buff[2];
	HAL_I2C_Master_Transmit(&i2cty, ina3221->i2caddr, &reg, 1, 1000);
	HAL_I2C_Master_Receive(&i2cty, ina3221->i2caddr, cmd_buff, 2, 1000);

    *value = ((cmd_buff[0] << 8) | cmd_buff[1]);


}

void INA3221SetConfig(INA3221 *ina3221) {
    uint16_t config = INA3221_CONFIG_ENABLE_CHAN1 |
                      INA3221_CONFIG_ENABLE_CHAN2 |
                      INA3221_CONFIG_ENABLE_CHAN3 |
                      INA3221_CONFIG_AVG1 |
                      INA3221_CONFIG_VBUS_CT2 |
                      INA3221_CONFIG_VSH_CT2 |
                      INA3221_CONFIG_MODE_2 |
                      INA3221_CONFIG_MODE_1 |
                      INA3221_CONFIG_MODE_0;
    INA3221_wireWriteRegister(ina3221, INA3221_REG_CONFIG, config);
}

void INA3221_init(I2C_HandleTypeDef *_td) {

    i2cty=*_td;
}

void INA3221_begin(INA3221 *ina3221) {
    INA3221SetConfig(ina3221);
}

int16_t INA3221_getBusVoltage_raw(INA3221 *ina3221, int channel) {
    uint16_t value;
    INA3221_wireReadRegister(ina3221, INA3221_REG_BUSVOLTAGE_1 + (channel - 1) * 2, &value);
    return (int16_t)(value);
}

int16_t INA3221_getShuntVoltage_raw(INA3221 *ina3221, int channel) {
    uint16_t value;
    INA3221_wireReadRegister(ina3221, INA3221_REG_SHUNTVOLTAGE_1 + (channel - 1) * 2, &value);
    return value;
}

float INA3221_getShuntVoltage_mV(INA3221 *ina3221, int channel) {
	return INA3221_getShuntVoltage_raw(ina3221, channel)* 0.005;
}

float INA3221_getBusVoltage_V(INA3221 *ina3221, int channel) {
	return INA3221_getBusVoltage_raw(ina3221, channel)* 0.001;
}

float INA3221_getCurrent_mA(INA3221 *ina3221, int channel) {
	return INA3221_getShuntVoltage_mV(ina3221, channel) / ina3221->shuntresistor[channel];
}

int INA3221_getManufID(INA3221 *ina3221) {
    uint16_t value;
    INA3221_wireReadRegister(ina3221, 0xFE, &value);
    return value;
}

