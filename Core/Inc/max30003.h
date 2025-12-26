#ifndef MAX30003_H_
#define MAX30003_H_

#include "main.h"   // Wymagane dla HAL
//#include <stdint.h> // Wymagane dla uint8_t, uint32_t
#define MAX30003_REG_NO_OP        0x00
#define MAX30003_REG_STATUS       0x01
#define MAX30003_REG_EN_INT       0x02
#define MAX30003_REG_EN_INT2      0x03
#define MAX30003_REG_MNGR_INT     0x04
#define MAX30003_REG_MNGR_DYN     0x05
#define MAX30003_REG_SW_RST       0x08
#define MAX30003_REG_SYNCH        0x09
#define MAX30003_REG_FIFO_RST     0x0A
#define MAX30003_REG_INFO         0x0F
#define MAX30003_REG_CNFG_GEN     0x10
#define MAX30003_REG_CNFG_CAL     0x12
#define MAX30003_REG_CNFG_EMUX    0x14
#define MAX30003_REG_CNFG_ECG     0x15
#define MAX30003_REG_CNFG_RTOR1   0x1D
#define MAX30003_REG_CNFG_RTOR2   0x1E
#define MAX30003_REG_ECG_FIFO_BURST 0x20
#define MAX30003_REG_ECG_FIFO     0x21
#define MAX30003_REG_RTOR         0x25
#define MAX30003_REG_NO_OP_ALT    0x7F

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} MAX30003_Ctx;

void MAX30003_Init(MAX30003_Ctx *c, SPI_HandleTypeDef *h, GPIO_TypeDef *p, uint16_t pin);
void MAX30003_WriteReg(MAX30003_Ctx *c, uint8_t r, uint32_t d);
uint32_t MAX30003_ReadReg(MAX30003_Ctx *c, uint8_t r);

int32_t MAX30003_GetECG(MAX30003_Ctx *c);
float MAX30003_GetHR(MAX30003_Ctx *c);
uint32_t MAX30003_ReadDeviceID(MAX30003_Ctx *c);

#endif
