#include "dfrobot_bloodoxygen.h"
#include <string.h>

static I2C_HandleTypeDef *hI2C;
static uint8_t devAddress;
#define BOD_TIMEOUT 1000

bool BoodOxy_Init_I2C(I2C_HandleTypeDef *hi2c, uint8_t address) {
    hI2C = hi2c;
    devAddress = address << 1;  // HAL expects 8-bit address
    return (HAL_I2C_IsDeviceReady(hI2C, devAddress, 3, BOD_TIMEOUT) == HAL_OK);
}

static HAL_StatusTypeDef writeReg(uint8_t reg, uint8_t *data, uint8_t len) {
    uint8_t buf[1 + len];
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return HAL_I2C_Master_Transmit(hI2C, devAddress, buf, len + 1, BOD_TIMEOUT);
}

static HAL_StatusTypeDef readReg(uint8_t reg, uint8_t *data, uint8_t len) {
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(hI2C, devAddress, &reg, 1, BOD_TIMEOUT);
    if (ret != HAL_OK) return ret;
    return HAL_I2C_Master_Receive(hI2C, devAddress, data, len, BOD_TIMEOUT);
}

HAL_StatusTypeDef BoodOxy_GetHeartbeatSPO2(BoodOxyData *data) {
    uint8_t buf[8];
    HAL_StatusTypeDef ret = readReg(0x0C, buf, 8);
    if (ret != HAL_OK) return ret;
    data->spo2 = (buf[0] != 0) ? buf[0] : -1;
    data->heartbeat = ((uint32_t)buf[2] << 24) |
                      ((uint32_t)buf[3] << 16) |
                      ((uint32_t)buf[4] << 8)  |
                      ((uint32_t)buf[5]);
    if (data->heartbeat == 0) data->heartbeat = -1;
    return HAL_OK;
}

HAL_StatusTypeDef BoodOxy_GetTemperature_C(float *temperature) {
    uint8_t buf[2];
    HAL_StatusTypeDef ret = readReg(0x14, buf, 2);
    if (ret != HAL_OK) return ret;
    *temperature = buf[0] + buf[1] / 100.0f;
    return HAL_OK;
}

HAL_StatusTypeDef BoodOxy_SetBaudrate(uint16_t bautrate) {
    uint8_t buf[2] = { (uint8_t)(bautrate >> 8), (uint8_t)bautrate };
    HAL_StatusTypeDef ret = writeReg(0x06, buf, 2);
    if (ret != HAL_OK) return ret;
    HAL_Delay(100);
    buf[0] = 0; buf[1] = 1;
    return writeReg(0x1A, buf, 2);
}

HAL_StatusTypeDef BoodOxy_GetBaudrate(uint32_t *bautrate) {
    uint8_t buf[2];
    HAL_StatusTypeDef ret = readReg(0x06, buf, 2);
    if (ret != HAL_OK) return ret;
    uint16_t type = (buf[0] << 8) | buf[1];
    switch (type) {
        case 0:  *bautrate = 1200;   break;
        case 1:  *bautrate = 2400;   break;
        case 3:  *bautrate = 9600;   break;
        case 5:  *bautrate = 19200;  break;
        case 6:  *bautrate = 38400;  break;
        case 7:  *bautrate = 57600;  break;
        case 8:  *bautrate = 115200; break;
        default: *bautrate = 9600;   break;
    }
    return HAL_OK;
}

HAL_StatusTypeDef BoodOxy_SensorStartCollect(void) {
    uint8_t buf[2] = { 0, 1 };
    return writeReg(0x20, buf, 2);
}

HAL_StatusTypeDef BoodOxy_SensorEndCollect(void) {
    uint8_t buf[2] = { 0, 2 };
    return writeReg(0x20, buf, 2);
}
