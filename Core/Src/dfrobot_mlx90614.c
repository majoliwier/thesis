/* dfrobot_mlx90614.c */
#include "dfrobot_mlx90614.h"

static I2C_HandleTypeDef *mlx_i2c;
static uint16_t          mlx_addr;      // 8-bit adres I²C (address<<1)
#define MLX90614_TIMEOUT  1000

/**
 * @brief Wewnętrzna funkcja do odczytu rejestru z MLX90614
 */
static HAL_StatusTypeDef MLX90614_ReadReg(uint8_t reg, uint8_t *buf, uint16_t len) {
    return HAL_I2C_Mem_Read(
        mlx_i2c,
        mlx_addr,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        len,
        MLX90614_TIMEOUT
    );
}

bool MLX90614_Init_I2C(I2C_HandleTypeDef *hi2c, uint8_t address) {
    mlx_i2c  = hi2c;
    mlx_addr = address << 1;
    return (HAL_I2C_IsDeviceReady(mlx_i2c, mlx_addr, 3, MLX90614_TIMEOUT) == HAL_OK);
}

HAL_StatusTypeDef MLX90614_ReadAmbientTemp(float *temperature) {
    uint8_t buf[3];
    HAL_StatusTypeDef ret = MLX90614_ReadReg(0x06, buf, sizeof(buf));
    if (ret != HAL_OK) return ret;
    uint16_t raw = ((uint16_t)buf[1] << 8) | buf[0];
    *temperature = (raw * 0.02f) - 273.15f;
    return HAL_OK;
}

HAL_StatusTypeDef MLX90614_ReadObjectTemp(float *temperature) {
    uint8_t buf[3];
    HAL_StatusTypeDef ret = MLX90614_ReadReg(0x07, buf, sizeof(buf));
    if (ret != HAL_OK) return ret;
    uint16_t raw = ((uint16_t)buf[1] << 8) | buf[0];
    *temperature = (raw * 0.02f) - 273.15f;
    return HAL_OK;
}
