#include "max30003.h"

// === SPI Low-level ===

void MAX30003_WriteReg(MAX30003_Ctx *c, uint8_t addr, uint32_t data) {
    uint8_t tx[4];
    tx[0] = (addr << 1) & 0xFE;  // Write bit = 0
    tx[1] = (data >> 16) & 0xFF;
    tx[2] = (data >> 8) & 0xFF;
    tx[3] = data & 0xFF;

    HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(c->hspi, tx, 4, 100);
    HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_SET);
}

uint32_t MAX30003_ReadReg(MAX30003_Ctx *c, uint8_t addr) {
    uint8_t tx[4] = { (addr << 1) | 0x01, 0, 0, 0 };
    uint8_t rx[4] = {0};

    HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_RESET);
    for(volatile int i=0; i<10; i++); // Krótkie czekanie na ustabilizowanie linii
    HAL_SPI_TransmitReceive(c->hspi, tx, rx, 4, 100);
    HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_SET);

    return ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
}

// === Initialization (EXACT Protocentral begin()) ===

void MAX30003_Init(MAX30003_Ctx *c, SPI_HandleTypeDef *hspi, GPIO_TypeDef *port, uint16_t pin) {
    c->hspi = hspi;
    c->cs_port = port;
    c->cs_pin = pin;

    HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_SET);
    HAL_Delay(10);

    MAX30003_WriteReg(c, 0x08, 0x000000); // Soft Reset
    HAL_Delay(100);

    // CNFG_GEN (0x10):
    // Bit 1: EN_RBIAS = 1 (REZUSTANCYJNY BIAS - MUSI BYĆ!)
    // Bit 2: EN_ECG = 1
    // Bity 4:3: RBIASP/N = 1 (podciągnięcie obu wejść do VBIAS)
    // Wartość 0x00001F (włącza bias na oba piny i ustawia 100mV odniesienia)
    MAX30003_WriteReg(c, 0x10, 0x00001F);

    // CNFG_ECG (0x15):
    // Zostajemy przy 20V/V, ale upewnij się, że DHPF jest na 0.5Hz (01)
    MAX30003_WriteReg(c, 0x15, 0x805000);

    // CNFG_EMUX (0x14):
    // Upewnij się, że bity 23:22 (POL) są na 00.
    MAX30003_WriteReg(c, 0x14, 0x000000);

    MAX30003_WriteReg(c, 0x04, 0x000004); // EFIT=1
    MAX30003_WriteReg(c, 0x02, 0x800001); // EINT enable

    MAX30003_WriteReg(c, 0x00, 0x000000); // FIFO Reset
    MAX30003_WriteReg(c, 0x09, 0x000000); // SYNCH
    HAL_Delay(50);
}

uint32_t MAX30003_ReadDeviceID(MAX30003_Ctx *c) {
    return MAX30003_ReadReg(c, 0x0F);  // INFO register
}

// === Read ECG Sample (CORRECT - based on Protocentral readEcgSample) ===

uint8_t MAX30003_GetECG_Sample(MAX30003_Ctx *c, int32_t *sample_out) {
    uint32_t data = MAX30003_ReadReg(c, 0x21);

    // Wyciągnij 18 bitów danych (od bitu 6 do 23)
    int32_t ecg = (int32_t)((data >> 6) & 0x3FFFF);

    // Rozszerzenie znaku dla 18-bitowej liczby (U2)
    if (ecg & 0x20000) {
        ecg |= 0xFFFC0000;
    }

    *sample_out = ecg;
    return (((data >> 3) & 0x07) != 0x07); // Zwróć 1, jeśli FIFO nie jest puste
}
// === Heart Rate (from RTOR register) ===

float MAX30003_GetHR(MAX30003_Ctx *c) {
    uint32_t rtor_reg = MAX30003_ReadReg(c, 0x25);

    // RTOR value in bits [23:10]
    uint16_t rtor = (rtor_reg >> 10) & 0x3FFF;

    if (rtor == 0) {
        return 0.0f;
    }

    // Heart rate calculation:
    // fs = 128 Hz
    // RR interval = rtor / 128 seconds
    // HR = 60 / (rtor / 128) = 7680 / rtor
    return 7680.0f / (float)rtor;
}

// === Legacy compatibility ===

int32_t MAX30003_GetECG(MAX30003_Ctx *c) {
    int32_t sample = 0;
    MAX30003_GetECG_Sample(c, &sample);
    return sample;
}

uint8_t MAX30003_ReadStatus(MAX30003_Ctx *c, uint8_t *status_out) {
    uint32_t status = MAX30003_ReadReg(c, 0x01);
    *status_out = (status & 0x800000) ? 1 : 0;
    return *status_out;
}
