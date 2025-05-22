/* dfrobot_mlx90614.h */
#ifndef __DFROBOT_MLX90614_H
#define __DFROBOT_MLX90614_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Data structure for MLX90614 temperatures
 */
typedef struct {
    float ambient;  ///< temperatura otoczenia [°C]
    float object;   ///< temperatura obiektu [°C]
} MLX90614_Data;

/**
 * @brief Initialize MLX90614 over I2C
 * @param hi2c    Pointer to HAL I2C handle (np. &hi2c3)
 * @param address 7-bit sensor address (domyślnie 0x5A)
 * @return true jeśli urządzenie gotowe, false w przeciwnym razie
 */
bool MLX90614_Init_I2C(I2C_HandleTypeDef *hi2c, uint8_t address);

/**
 * @brief Read ambient temperature (°C)
 * @param temperature Pointer to float to receive value
 * @return HAL status
 */
HAL_StatusTypeDef MLX90614_ReadAmbientTemp(float *temperature);

/**
 * @brief Read object temperature (°C)
 * @param temperature Pointer to float to receive value
 * @return HAL status
 */
HAL_StatusTypeDef MLX90614_ReadObjectTemp(float *temperature);

#endif // __DFROBOT_MLX90614_H
