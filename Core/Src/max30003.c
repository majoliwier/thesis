#include "max30003.h"
#include <stdio.h>

// === KONFIGURACJA PINÓW SPI4 (PE2, PE5, PE6, PB1) ===
// Sprawdź czy kable są dokładnie tutaj!
#define SW_CS_PORT    GPIOB
#define SW_CS_PIN     GPIO_PIN_1

#define SW_SCK_PORT   GPIOE
#define SW_SCK_PIN    GPIO_PIN_2

#define SW_MISO_PORT  GPIOE
#define SW_MISO_PIN   GPIO_PIN_5

#define SW_MOSI_PORT  GPIOE
#define SW_MOSI_PIN   GPIO_PIN_6

// === MAKRA STERUJĄCE (Bit-Banging) ===
#define CS_L()    HAL_GPIO_WritePin(SW_CS_PORT, SW_CS_PIN, GPIO_PIN_RESET)
#define CS_H()    HAL_GPIO_WritePin(SW_CS_PORT, SW_CS_PIN, GPIO_PIN_SET)
#define SCK_L()   HAL_GPIO_WritePin(SW_SCK_PORT, SW_SCK_PIN, GPIO_PIN_RESET)
#define SCK_H()   HAL_GPIO_WritePin(SW_SCK_PORT, SW_SCK_PIN, GPIO_PIN_SET)
#define MOSI_L()  HAL_GPIO_WritePin(SW_MOSI_PORT, SW_MOSI_PIN, GPIO_PIN_RESET)
#define MOSI_H()  HAL_GPIO_WritePin(SW_MOSI_PORT, SW_MOSI_PIN, GPIO_PIN_SET)
#define MISO_RD() HAL_GPIO_ReadPin(SW_MISO_PORT, SW_MISO_PIN)

// Opóźnienie - zwolnione tempo dla pewności
void SW_Delay(void) {
    for (volatile int i = 0; i < 50; i++);
}

// Transfer 1 bajtu (Ręcznie, bit po bicie)
uint8_t SW_Transfer(uint8_t b) {
    uint8_t rx = 0;
    for (int i = 0; i < 8; i++) {
        // 1. Zapisz MOSI (MSB first)
        if (b & 0x80) MOSI_H(); else MOSI_L();
        b <<= 1;
        SW_Delay();

        // 2. Zegar w GÓRĘ (MAX czyta dane)
        SCK_H();
        SW_Delay();

        // 3. Odczytaj MISO
        rx <<= 1;
        if (MISO_RD()) rx |= 1;

        // 4. Zegar w DÓŁ (MAX zmienia dane)
        SCK_L();
        SW_Delay();
    }
    return rx;
}

// Funkcja Zapisu
void MAX30003_WriteReg(MAX30003_Ctx *c, uint8_t addr, uint32_t data) {
    CS_L(); SW_Delay();
    SW_Transfer((addr << 1) & 0xFE); // Adres zapisu
    SW_Transfer((data >> 16) & 0xFF);
    SW_Transfer((data >> 8) & 0xFF);
    SW_Transfer(data & 0xFF);
    SW_Delay(); CS_H(); SW_Delay();
}

// Funkcja Odczytu
uint32_t MAX30003_ReadReg(MAX30003_Ctx *c, uint8_t addr) {
    CS_L(); SW_Delay();
    SW_Transfer((addr << 1) | 0x01); // Adres odczytu
    uint32_t d = 0;
    d |= ((uint32_t)SW_Transfer(0xFF) << 16);
    d |= ((uint32_t)SW_Transfer(0xFF) << 8);
    d |= (uint32_t)SW_Transfer(0xFF);
    SW_Delay(); CS_H(); SW_Delay();
    return d;
}

// === INIT (GENERATOR TESTOWY) ===
void MAX30003_Init(MAX30003_Ctx *c, SPI_HandleTypeDef *hspi, GPIO_TypeDef *port, uint16_t pin) {
    // 1. Konfiguracja PINÓW (Taka sama jak była - działa)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    HAL_GPIO_WritePin(SW_CS_PORT, SW_CS_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = SW_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SW_CS_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SW_SCK_PIN | SW_MOSI_PIN;
    HAL_GPIO_Init(SW_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SW_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SW_MISO_PORT, &GPIO_InitStruct);

    SCK_L();

    // 2. RESET
    MAX30003_WriteReg(c, 0x08, 0x000000);
    HAL_Delay(100);

    // Sprawdzenie ID (Dla pewności)
    uint32_t id = MAX30003_ReadReg(c, 0x0F);
    printf("MAX30003 ID: %06lX\r\n", id);

    // ==========================================================
    // === KONFIGURACJA DO PRAWDZIWEGO EKG (LIVE MODE) ===
    // ==========================================================

    // 3. CNFG_GEN (0x10)
    // 0x08041F -> Włącz EKG, Włącz Bias (napięcie odniesienia dla ciała)
    MAX30003_WriteReg(c, 0x10, 0x08041F);
    HAL_Delay(10);

    // 4. CNFG_CAL (0x12) - WAŻNA ZMIANA
    // 0x000000 -> WYŁĄCZAMY Generator Testowy!
    MAX30003_WriteReg(c, 0x12, 0x000000);

    // 5. CNFG_EMUX (0x14) - WAŻNA ZMIANA
    // 0x000000 -> Otwieramy wejścia na elektrody (Open Switches)
    // To podłącza piny ECGP/ECGN do przetwornika.
    MAX30003_WriteReg(c, 0x14, 0x000000);

    // 6. CNFG_ECG (0x15) - WAŻNA ZMIANA (GAIN)
    // Wcześniej mieliśmy 0x83... (160x). To za dużo dla człowieka (nasyci się).
    // Ustawiamy 0x805000 (Gain = 20x) - Standardowe ustawienie.
    // Jeśli sygnał będzie za mały, zmienisz na 0x815000 (40x).
    MAX30003_WriteReg(c, 0x15, 0x805000);

    // 7. Reszta ustawień (RTOR i Przerwania)
    MAX30003_WriteReg(c, 0x1D, 0x3FC600);
    MAX30003_WriteReg(c, 0x04, 0x000004);

    // 8. START
    MAX30003_WriteReg(c, 0x00, 0x000000); // FIFO Reset
    MAX30003_WriteReg(c, 0x09, 0x000000); // SYNCH

    printf("LIVE ECG MODE STARTED. Podlacz elektrody!\r\n");
}

uint32_t MAX30003_ReadDeviceID(MAX30003_Ctx *c) { return MAX30003_ReadReg(c, 0x0F); }

uint8_t MAX30003_GetECG_Sample(MAX30003_Ctx *c, int32_t *sample_out) {
    uint32_t data = MAX30003_ReadReg(c, 0x21);

    // Sprawdzamy tag (czy dane sa poprawne)
    uint8_t tag = (data >> 3) & 0x07;
    if (tag == 0x07 || tag == 0x06) return 0; // Puste

    int32_t ecg = (int32_t)((data >> 6) & 0x3FFFF);
    if (ecg & 0x20000) ecg |= 0xFFFC0000;
    *sample_out = ecg;
    return 1;
}

// Dummy functions
float MAX30003_GetHR(MAX30003_Ctx *c) { return 0; }
int32_t MAX30003_GetECG(MAX30003_Ctx *c) { return 0; }
