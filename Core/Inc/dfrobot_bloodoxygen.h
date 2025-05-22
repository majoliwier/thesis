#ifndef __DFROBOT_BLOODOXYGEN_H
#define __DFROBOT_BLOODOXYGEN_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Data structure for heart rate and SpO2
 */
typedef struct {
    int16_t spo2;
    int32_t heartbeat;
} BoodOxyData;

/**
 * @brief Initialize the sensor over I2C
 * @param hi2c Pointer to the HAL I2C handle
 * @param address 7-bit I2C address of the sensor (e.g., 0x57)
 * @return true if sensor acknowledges, false otherwise
 */
bool BoodOxy_Init_I2C(I2C_HandleTypeDef *hi2c, uint8_t address);

/**
 * @brief Read heart rate and SpO2
 * @param data Pointer to data struct to fill
 * @return HAL status
 */
HAL_StatusTypeDef BoodOxy_GetHeartbeatSPO2(BoodOxyData *data);

/**
 * @brief Read onboard temperature in °C
 * @param temperature Pointer to float to store temperature
 * @return HAL status
 */
HAL_StatusTypeDef BoodOxy_GetTemperature_C(float *temperature);

/**
 * @brief Set the sensor UART baudrate (for UART mode)
 * @param bautrate 16-bit code corresponding to enum value
 * @return HAL status
 */
HAL_StatusTypeDef BoodOxy_SetBaudrate(uint16_t bautrate);

/**
 * @brief Get the sensor UART baudrate
 * @param bautrate Pointer to uint32 to receive actual baudrate
 * @return HAL status
 */
HAL_StatusTypeDef BoodOxy_GetBaudrate(uint32_t *bautrate);

/**
 * @brief Begin data collection
 * @return HAL status
 */
HAL_StatusTypeDef BoodOxy_SensorStartCollect(void);

/**
 * @brief Stop data collection
 * @return HAL status
 */
HAL_StatusTypeDef BoodOxy_SensorEndCollect(void);

#endif // __DFROBOT_BLOODOXYGEN_H
