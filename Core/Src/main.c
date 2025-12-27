/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dfrobot_bloodoxygen.h"
#include "dfrobot_mlx90614.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ts.h"
#include "max30003.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_RETRIES 3

#define MAX30003_CS_GPIO_Port GPIOC
#define MAX30003_CS_Pin       GPIO_PIN_4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
uint32_t rtc_base_time = 0;
osThreadId myTaskHandle;
osThreadId touchTaskHandle;
osThreadId renderTaskHandle;

float CFG_TEMP_HIGH = 37.5f;
float CFG_TEMP_LOW  = 35.0f;

int32_t CFG_SPO2_LOW = 92;
int32_t CFG_HR_HIGH  = 120;
int32_t CFG_HR_LOW   = 40;

int32_t CFG_GSR_STRESS = 600;
int32_t CFG_GSR_NEUTRAL = 1000;

volatile uint8_t systemState = 1;
volatile uint8_t systemStartupComplete = 0;

#define BLOOD_OXY_BUFFER_SIZE 5
typedef struct {
    int16_t spo2_values[BLOOD_OXY_BUFFER_SIZE];
    int32_t hr_values[BLOOD_OXY_BUFFER_SIZE];
    uint8_t valid_count;
    int16_t last_valid_spo2;
    int32_t last_valid_hr;
} BloodOxyBuffer;

typedef struct {
    uint32_t value;
    char mood[16];
} GSR_Data_t;

volatile GSR_Data_t gsrData;

static BloodOxyBuffer bloodOxyBuffer = {0};
volatile uint8_t measurementActive = 1;
volatile uint8_t updateScreenFlag = 0;
osMessageQId ecgQueueHandle;
osMessageQId tempQueueHandle;
osMessageQId oxyQueueHandle;
osMessageQId gsrQueueHandle;

MAX30003_Ctx max30003;

#define BUTTON_AREA_Y_START 270
#define BUTTON_HEIGHT       50
#define BUTTON_WIDTH        80
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_DAC_Init(void);
static void MX_UART5_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI4_Init(void);
void StartDefaultTask(const void *argument);

/* USER CODE BEGIN PFP */
uint8_t calcChecksum(const char* str, size_t len);
bool waitForAck(UART_HandleTypeDef* uart, uint32_t timeout_ms);

void StartTouchTask(const void *argument);
void StartRenderTask(const void *argument);

void DrawHeader(void);
void DrawBottomButtons(void);
uint8_t MAX30003_GetECG_Sample(MAX30003_Ctx *ctx, int32_t *sample);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
#else
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
#endif

volatile uint8_t currentSensor = 0;

extern uint32_t rtc_base_time;

MLX90614_Data mlxData;
BoodOxyData oxyData;
extern uint16_t lcdHeight;


void StartRenderTask(const void *argument)
{
	while (systemStartupComplete == 0) {
	        osDelay(50);
	    }
    // Start systemu
    BSP_LCD_DisplayOn();
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    DrawHeader();
    DrawBottomButtons();

    const uint16_t GRAPH_TOP = 35;
    const uint16_t GRAPH_BOT = 270;
    static uint16_t x = 0;
    static uint16_t lastY = 160;
    static uint8_t lastSensor = 255;
    static uint8_t lastSystemState = 1;

    for (;;)
    {
        // ==========================================================
        // 1. ZARZĄDZANIE ENERGIĄ (SUSPEND / RESUME)
        // ==========================================================

        // --- PRZEJŚCIE W STAN OFF ---
        if (systemState == 0)
        {
            if (lastSystemState == 1)
            {
                // 1. Zamaluj ekran na czarno (blokada światła)
                BSP_LCD_Clear(LCD_COLOR_BLACK);

                // 2. ZATRZYMAJ INNE TASKI (Usypiamy sensory i dotyk)
                // Dzięki temu myTask nie mieli w kółko I2C, a touchTask nie mieli SPI.
                osThreadSuspend(myTaskHandle);
                osThreadSuspend(touchTaskHandle);

                lastSystemState = 0;
            }

            // RenderTask też idzie spać (czeka na zmianę flagi), odświeżanie co 200ms
            osDelay(200);
            continue;
        }

        // --- PRZEJŚCIE W STAN ON ---
        else if (systemState == 1 && lastSystemState == 0)
        {
            // 1. WZNÓW INNE TASKI (Budzimy je)
            osThreadResume(myTaskHandle);
            osThreadResume(touchTaskHandle);

            // 2. Przywróć ekran
            BSP_LCD_DisplayOn();
            osDelay(100);
            BSP_LCD_Clear(LCD_COLOR_BLACK);
            DrawHeader();
            DrawBottomButtons();

            BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
            BSP_LCD_SetTextColor(0xFFFFFFFF); // Biały
            BSP_LCD_SetFont(&Font24);
            BSP_LCD_DisplayStringAt(0, 120, (uint8_t*)"SYSTEM WAKEUP", CENTER_MODE);

            // 3. Reset zmiennych, żeby wykresy nie wariowały
            x = 0;
            osDelay(500);
            updateScreenFlag = 1; // Wymuś narysowanie właściwego ekranu
            lastSystemState = 1;
        }

        // ==========================================================
        // 2. NORMALNA PRACA (Reszta kodu bez zmian)
        // ==========================================================

        // Zmiana Sensora
        if (currentSensor != lastSensor)
        {
            BSP_LCD_Clear(LCD_COLOR_BLACK);
            DrawHeader();
            DrawBottomButtons();

            if (measurementActive) {
                BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
                BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
                BSP_LCD_SetFont(&Font16);

                if(currentSensor == 1) BSP_LCD_DisplayStringAt(0, 100, (uint8_t*)"Place Finger...", CENTER_MODE);
                else if(currentSensor == 3) BSP_LCD_DisplayStringAt(0, 100, (uint8_t*)"Initializing ECG...", CENTER_MODE);
                else BSP_LCD_DisplayStringAt(0, 100, (uint8_t*)"Measuring...", CENTER_MODE);
            }
            lastSensor = currentSensor;
            updateScreenFlag = 0;
            x = 0;
        }
        // Start / Stop (Pauza na ekranie)
        else if (updateScreenFlag)
        {
            if (!measurementActive) // PAUZA
            {
                DrawBottomButtons();
                BSP_LCD_SetTextColor(LCD_COLOR_BLACK); BSP_LCD_FillRect(50, 110, 140, 60);
                BSP_LCD_SetTextColor(LCD_COLOR_WHITE); // Ramka (Czerwona)

                BSP_LCD_DrawRect(50, 110, 140, 60);
                BSP_LCD_DrawRect(51, 111, 138, 58);

                BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
                BSP_LCD_SetTextColor(LCD_COLOR_RED);
                BSP_LCD_SetFont(&Font24);
                BSP_LCD_DisplayStringAt(0, 128, (uint8_t*)"PAUSED", CENTER_MODE);
            }
            else
            {
                BSP_LCD_Clear(LCD_COLOR_BLACK); DrawHeader(); DrawBottomButtons();
                BSP_LCD_SetBackColor(LCD_COLOR_BLACK); BSP_LCD_SetTextColor(LCD_COLOR_YELLOW); BSP_LCD_SetFont(&Font16);
                if(currentSensor == 1) BSP_LCD_DisplayStringAt(0, 100, (uint8_t*)"Place Finger...", CENTER_MODE);
                else BSP_LCD_DisplayStringAt(0, 100, (uint8_t*)"Measuring...", CENTER_MODE);
                x = 0;
            }
            updateScreenFlag = 0;
        }

        if (currentSensor == 0) {
                     osEvent evt = osMessageGet(tempQueueHandle, 10);
                     if (evt.status == osEventMessage) {
                         MLX90614_Data *pData = (MLX90614_Data*)evt.value.v;

                         // Tło
                         BSP_LCD_SetTextColor(LCD_COLOR_BLACK); BSP_LCD_FillRect(0, 80, 240, 60);
                         BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

                         // Wyświetlanie Ambient (zazwyczaj mniej ważne)
                         BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
                         BSP_LCD_SetFont(&Font16);
                         char buf[32];
                         snprintf(buf, sizeof(buf), "Ambient: %.2f C", pData->ambient);
                         BSP_LCD_DisplayStringAt(0, 80, (uint8_t*)buf, CENTER_MODE);

                         // --- LOGIKA ALARMU WIZUALNEGO ---
                         if (pData->object >= CFG_TEMP_HIGH || pData->object <= CFG_TEMP_LOW) {
                             BSP_LCD_SetTextColor(LCD_COLOR_RED); // ALARM!
                         } else {
                             BSP_LCD_SetTextColor(LCD_COLOR_GREEN); // OK
                         }
                         // -------------------------------

                         snprintf(buf, sizeof(buf), "Object : %.2f C", pData->object);
                         BSP_LCD_DisplayStringAt(0, 110, (uint8_t*)buf, CENTER_MODE);
                     }
                }
        else if (currentSensor == 1) {
                     osEvent evt = osMessageGet(oxyQueueHandle, 10);
                     if (evt.status == osEventMessage) {
                         BoodOxyData *pData = (BoodOxyData*)evt.value.v;
                         BSP_LCD_SetTextColor(LCD_COLOR_BLACK); BSP_LCD_FillRect(0, 80, 240, 120);
                         BSP_LCD_SetBackColor(LCD_COLOR_BLACK); BSP_LCD_SetFont(&Font16);
                         char buf[32];

                         // --- LOGIKA ALARMU DLA SPO2 ---
                         if (pData->spo2 < CFG_SPO2_LOW) BSP_LCD_SetTextColor(LCD_COLOR_RED);
                         else BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

                         snprintf(buf, sizeof(buf), "SpO2 : %d %%", pData->spo2);
                         BSP_LCD_DisplayStringAt(0, 80, (uint8_t*)buf, CENTER_MODE);

                         // --- LOGIKA ALARMU DLA HR ---
                         if (pData->heartbeat > CFG_HR_HIGH || pData->heartbeat < CFG_HR_LOW)
                             BSP_LCD_SetTextColor(LCD_COLOR_RED);
                         else
                             BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

                         snprintf(buf, sizeof(buf), "HR   : %ld bpm", pData->heartbeat);
                         BSP_LCD_DisplayStringAt(0, 110, (uint8_t*)buf, CENTER_MODE);

                         BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
                         BSP_LCD_DisplayStringAt(0, 140, (uint8_t*)"RESULT READY", CENTER_MODE);
                     }
                }
        else if (currentSensor == 2) {
             osEvent evt = osMessageGet(gsrQueueHandle, 10);
             if (evt.status == osEventMessage) {
                 GSR_Data_t *pData = (GSR_Data_t*)evt.value.v;
                 BSP_LCD_SetTextColor(LCD_COLOR_BLACK); BSP_LCD_FillRect(0, 80, 240, 60);
                 BSP_LCD_SetBackColor(LCD_COLOR_BLACK); BSP_LCD_SetTextColor(0xFFFFFFFF); BSP_LCD_SetFont(&Font16);
                 char buf[32];
                 snprintf(buf, sizeof(buf), "GSR Avg: %lu", pData->value);
                 BSP_LCD_DisplayStringAt(0, 80, (uint8_t*)buf, CENTER_MODE);

                 if (strcmp((char*)pData->mood, "Stressed") == 0) BSP_LCD_SetTextColor(LCD_COLOR_RED); // Czerwony
                 else if (strcmp((char*)pData->mood, "Relaxed") == 0) BSP_LCD_SetTextColor(LCD_COLOR_GREEN); // Zielony
                 else BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

                 snprintf(buf, sizeof(buf), "Mood: %s", pData->mood);
                 BSP_LCD_DisplayStringAt(0, 110, (uint8_t*)buf, CENTER_MODE);
             }
        }
        else if (currentSensor == 3) {
             osEvent evt = osMessageGet(ecgQueueHandle, 2);
             if (evt.status == osEventMessage) {
                if(x==0) { BSP_LCD_SetTextColor(LCD_COLOR_BLACK); BSP_LCD_FillRect(0, 80, 240, 40); }
                int32_t val = evt.value.v;
                int16_t newY = 160 - (val / 35);
                if (newY < GRAPH_TOP) {
                    newY = GRAPH_TOP;
                }
                if (newY > GRAPH_BOT) {
                    newY = GRAPH_BOT;
                }
                BSP_LCD_SetTextColor(LCD_COLOR_BLACK); BSP_LCD_FillRect(x+1, GRAPH_TOP, 10, GRAPH_BOT-GRAPH_TOP);
                BSP_LCD_SetTextColor(LCD_COLOR_GREEN); // Zielony wykres
                if(x>0) BSP_LCD_DrawLine(x-1, lastY, x, newY);
                lastY = newY; x++;
                if (x >= 240) x = 0;
             }
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // =========================================================
    // 1. OBSŁUGA KABLA WIFI (PC1) - TO JEST NOWE
    // =========================================================
    if (GPIO_Pin == GPIO_PIN_1)
    {
        // Sprawdzamy czy stan jest NISKI (0V = Brak WiFi)
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET)
        {
            printf("!!! HARDWARE SIGNAL: WIFI LOST !!!\r\n");

            // 1. Zatrzymaj pomiary i wyczyść bufory
            measurementActive = 0;
            memset(&bloodOxyBuffer, 0, sizeof(BloodOxyBuffer));

            // 2. Rysuj CZERWONY EKRAN (NO WIFI)
            BSP_LCD_Clear(LCD_COLOR_BLUE);
            BSP_LCD_SetBackColor(LCD_COLOR_RED);
            BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

            BSP_LCD_SetFont(&Font24);
            BSP_LCD_DisplayStringAt(0, 120, (uint8_t*)"NO WIFI", CENTER_MODE);

            BSP_LCD_SetFont(&Font16);
            BSP_LCD_DisplayStringAt(0, 160, (uint8_t*)"Connection Lost...", CENTER_MODE);

            while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET)
            {
                HAL_GPIO_TogglePin(GPIOG, LD3_Pin);

                for(volatile int i=0; i<1000000; i++);
            }

            printf("!!! HARDWARE SIGNAL: WIFI RESTORED !!!\r\n");

            BSP_LCD_Clear(LCD_COLOR_BLACK);
            DrawHeader();
            DrawBottomButtons();
            updateScreenFlag = 1;
        }
    }

    // =========================================================
    // 2. OBSŁUGA PRZYCISKU (PA0) - STARY KOD
    // =========================================================
    if (GPIO_Pin == GPIO_PIN_0) {
		static uint32_t last_tick = 0;
		if (HAL_GetTick() - last_tick > 200)
		{
			systemState = !systemState;

			if (systemState) {
				printf("SYSTEM: POWER ON\r\n");
				HAL_GPIO_WritePin(GPIOG, LD3_Pin, GPIO_PIN_SET);
			} else {
				printf("SYSTEM: POWER OFF (Standby)\r\n");
				HAL_GPIO_WritePin(GPIOG, LD3_Pin, GPIO_PIN_RESET);

				measurementActive = 0;
				memset(&bloodOxyBuffer, 0, sizeof(BloodOxyBuffer));
			}

			updateScreenFlag = 1;
			last_tick = HAL_GetTick();
		}
    }
}

void myStartDefaultTask(const void *argument);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(20);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_DAC_Init();
  MX_UART5_Init();
  MX_ADC1_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOG, LD3_Pin, GPIO_PIN_SET);
    #define LAYER0_ADDRESS  0xD0000000
    #define LAYER1_ADDRESS  (0xD0000000 + 0x25800)


//    LTDC_LayerCfgTypeDef pLayerCfg = {0};

    BSP_LCD_MspInit();

    hltdc.Instance = LTDC;
    hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
    hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
    hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
    hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
    hltdc.Init.HorizontalSync = 9;
    hltdc.Init.VerticalSync = 1;
    hltdc.Init.AccumulatedHBP = 29;
    hltdc.Init.AccumulatedVBP = 3;
    hltdc.Init.AccumulatedActiveW = 269;
    hltdc.Init.AccumulatedActiveH = 323;
    hltdc.Init.TotalWidth = 279;
    hltdc.Init.TotalHeigh = 327;

    hltdc.Init.Backcolor.Blue = 0;
    hltdc.Init.Backcolor.Green = 0;
    hltdc.Init.Backcolor.Red = 0;

    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
    PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
    PeriphClkInitStruct.PLLSAI.PLLSAIR = 4;
    PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    HAL_LTDC_Init(&hltdc);
    ili9341_Init();
    BSP_SDRAM_Init();


    memset((void*)LAYER0_ADDRESS, 0, 240 * 320 * 2 * 2);


    BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(0, LAYER0_ADDRESS);
    BSP_LCD_SelectLayer(0);
    BSP_LCD_SetLayerVisible(0, ENABLE);
    BSP_LCD_SetColorKeying(0, LCD_COLOR_BLACK);
    HAL_LTDC_DisableColorKeying(&hltdc, 0);
    BSP_LCD_Clear(LCD_COLOR_BLACK);


    BSP_LCD_LayerDefaultInit(1, LAYER1_ADDRESS);
    BSP_LCD_SelectLayer(1);
    BSP_LCD_SetLayerVisible(1, ENABLE);
    BSP_LCD_SetTransparency(1, 255);

    HAL_LTDC_DisableColorKeying(&hltdc, 1);

    ili9341_DisplayOn();

    BSP_LCD_SelectLayer(1);

  BSP_LCD_Clear(LCD_COLOR_BLACK);


  BSP_LCD_SetTextColor(LCD_COLOR_RED);
  BSP_LCD_FillRect(10, 100, 220, 120);

      BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
      BSP_LCD_DrawRect(10, 100, 220, 120);

      BSP_LCD_SetBackColor(LCD_COLOR_GREEN);
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

      BSP_LCD_SetFont(&Font20);

      BSP_LCD_DisplayStringAt(0, 130, (uint8_t*)"SYSTEM START UP", CENTER_MODE);

      BSP_LCD_SetFont(&Font16);
      BSP_LCD_DisplayStringAt(0, 160, (uint8_t*)"Please wait...", CENTER_MODE);

      if (BSP_TS_Init(240, 320) == TS_OK) {
          printf("TS init OK\r\n");
      }
    /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  osMessageQDef(ecgQueue, 64, int32_t);
  ecgQueueHandle = osMessageCreate(osMessageQ(ecgQueue), NULL);

  osMessageQDef(tempQueue, 1, uint32_t);
  tempQueueHandle = osMessageCreate(osMessageQ(tempQueue), NULL);

  osMessageQDef(oxyQueue, 1, uint32_t);
  oxyQueueHandle = osMessageCreate(osMessageQ(oxyQueue), NULL);

  osMessageQDef(gsrQueue, 1, uint32_t);
  gsrQueueHandle = osMessageCreate(osMessageQ(gsrQueue), NULL);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(myTask, myStartDefaultTask, osPriorityLow, 0, 4096);
  myTaskHandle = osThreadCreate(osThread(myTask), NULL);


  osThreadDef(touchTask, StartTouchTask, osPriorityLow, 0, 2048);
  touchTaskHandle = osThreadCreate(osThread(touchTask), NULL);

  osThreadDef(renderTask, StartRenderTask, osPriorityBelowNormal, 0, 512);
  renderTaskHandle = osThreadCreate(osThread(renderTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 8;
  hltdc.Init.VerticalSync = 0;
  hltdc.Init.AccumulatedHBP = 37;
  hltdc.Init.AccumulatedVBP = 3;
  hltdc.Init.AccumulatedActiveW = 277;
  hltdc.Init.AccumulatedActiveH = 323;
  hltdc.Init.TotalWidth = 287;
  hltdc.Init.TotalHeigh = 327;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 239;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 319;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0xD0000000;
  pLayerCfg.ImageWidth = 240;
  pLayerCfg.ImageHeight = 320;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin CSX_Pin OTG_FS_PSO_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT1_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MEMS_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT2_Pin TP_INT1_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin|TP_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ACP_RST_Pin */
  GPIO_InitStruct.Pin = ACP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OC_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TE_Pin */
  GPIO_InitStruct.Pin = TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RDX_Pin WRX_DCX_Pin */
  GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1; // Używamy PC1
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; // Reaguj na zmianę 0->1 i 1->0
	GPIO_InitStruct.Pull = GPIO_PULLDOWN; // Domyślnie w dół (jeśli kabel się urwie to brak wifi)
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// 3. Włącz przerwanie w NVIC (kontroler przerwań)
	// PC1 korzysta z linii EXTI1
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0); // Najwyższy priorytet!
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void DrawHeader(void)
{
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_FillRect(0, 0, 240, 32);

    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font16);

    switch (currentSensor)
    {
        case 0:
            BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"Thermometer", CENTER_MODE);
            break;
        case 1:
            BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"SpO2 & HR", CENTER_MODE);
            break;
        case 2:
            BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"Stress Level", CENTER_MODE);
            break;
        case 3:
            BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"Live ECG", CENTER_MODE);
            break;
    }

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawHLine(0, 32, 240);

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
}

void DrawBottomButtons(void)
{
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_FillRect(0, BUTTON_AREA_Y_START, 240, BUTTON_HEIGHT);

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawVLine(80, BUTTON_AREA_Y_START, BUTTON_HEIGHT);
    BSP_LCD_DrawVLine(160, BUTTON_AREA_Y_START, BUTTON_HEIGHT);

    BSP_LCD_DrawHLine(0, BUTTON_AREA_Y_START, 240);

    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font16);

    BSP_LCD_DisplayStringAt(11, BUTTON_AREA_Y_START + 15, (uint8_t *)"< PREV", LEFT_MODE);

    if (measurementActive) {
        BSP_LCD_SetTextColor(LCD_COLOR_RED);
        BSP_LCD_DisplayStringAt(4, BUTTON_AREA_Y_START + 15, (uint8_t *)"STOP", CENTER_MODE);
    } else {
        BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
        BSP_LCD_DisplayStringAt(4, BUTTON_AREA_Y_START + 15, (uint8_t *)"START", CENTER_MODE);
    }

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

    BSP_LCD_DisplayStringAt(0, BUTTON_AREA_Y_START + 15, (uint8_t *)"NEXT >", RIGHT_MODE);
}

void StartTouchTask(const void *argument)
{
    TS_StateTypeDef ts;
    uint8_t wasTouched = 0;

    printf("Touch task started\r\n");
    osDelay(500);

    for (;;)
    {

        BSP_TS_GetState(&ts);

        if (ts.TouchDetected)
        {
            if (!wasTouched)
            {
                uint16_t y = 320 - ts.Y;
                uint16_t x = 240 - ts.X;

                if (y >= BUTTON_AREA_Y_START)
                {
                    if (x < 80)
                    {
                        currentSensor = (currentSensor + 1) % 4;
                        printf("BTN: PREV -> Sensor %d\r\n", currentSensor);

                        memset(&bloodOxyBuffer, 0, sizeof(BloodOxyBuffer));
                    }
                    else if (x < 160)
                    {
                        measurementActive = !measurementActive;
                        printf("BTN: %s\r\n", measurementActive ? "START" : "STOP");

                        if (measurementActive == 0) {
                            memset(&bloodOxyBuffer, 0, sizeof(BloodOxyBuffer));
                        }
                    }
                    else
                    {
                        currentSensor = (currentSensor + 3) % 4;
                        printf("BTN: NEXT -> Sensor %d\r\n", currentSensor);

                        memset(&bloodOxyBuffer, 0, sizeof(BloodOxyBuffer));
                    }

                    updateScreenFlag = 1;
                }
            }
            wasTouched = 1;
        }
        else
        {
            wasTouched = 0;
        }

        osDelay(50);
    }
}
void GetTimeFromESP(void)
{
    const char* cmd = "GET_TIME\n";
    static uint8_t rxBuf[128]; // Mały bufor wystarczy na czas
    rtc_base_time = 0;

    printf("--- SYNC: TIME ---\r\n");

    while (rtc_base_time == 0)
    {
        // 1. Flush
        if(__HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE)) {
             volatile uint8_t d = (uint8_t)(huart5.Instance->DR & 0xFF); (void)d;
        }
        __HAL_UART_CLEAR_OREFLAG(&huart5);
        memset(rxBuf, 0, sizeof(rxBuf));

        // 2. Wyślij
        HAL_UART_Transmit(&huart5, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);

        // 3. Odbierz (Czekaj na '{')
        uint8_t ch = 0;
        if (HAL_UART_Receive(&huart5, &ch, 1, 2000) == HAL_OK)
        {
            if (ch == '{') {
                int idx = 0;
                rxBuf[idx++] = ch;

                // Szybka pętla odbioru
                uint32_t to = 1000000;
                while (to-- > 0) {
                    if (huart5.Instance->SR & UART_FLAG_RXNE) {
                        uint8_t d = (uint8_t)(huart5.Instance->DR & 0xFF);
                        if (d == '\r') continue;
                        rxBuf[idx++] = d;
                        if (d == '\n') break;
                        if (idx >= 127) break;
                        to = 1000000; // Reset timeoutu po znaku
                    }
                }
                rxBuf[idx] = '\0';

                // Parsowanie
                char *ptr = strstr((char*)rxBuf, "\"rtc\":");
                if (ptr) {
                    rtc_base_time = atoi(ptr + 6);
                    if (rtc_base_time > 1600000000) {
                        printf(">> TIME OK: %lu\r\n", rtc_base_time);
                        return; // SUKCES
                    }
                }
            }
        }
        printf("Retry Time...\r\n");
        osDelay(500);
    }
}

void GetConfigFromESP(void)
{
    const char* cmd = "GET_CONFIG\n";
    static uint8_t rxBuf[512]; // Większy bufor na config
    uint8_t success = 0;

    printf("--- SYNC: CONFIG ---\r\n");

    while (!success)
    {
        // 1. Flush
        if(__HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE)) {
             volatile uint8_t d = (uint8_t)(huart5.Instance->DR & 0xFF); (void)d;
        }
        __HAL_UART_CLEAR_OREFLAG(&huart5);
        memset(rxBuf, 0, sizeof(rxBuf));

        // 2. Wyślij
        HAL_UART_Transmit(&huart5, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);

        // 3. Odbierz
        uint8_t ch = 0;
        if (HAL_UART_Receive(&huart5, &ch, 1, 2000) == HAL_OK)
        {
            if (ch == '{') {
                int idx = 0;
                rxBuf[idx++] = ch;

                uint32_t to = 2000000;
                while (to-- > 0) {
                    if (huart5.Instance->SR & UART_FLAG_RXNE) {
                        uint8_t d = (uint8_t)(huart5.Instance->DR & 0xFF);
                        if (d == '\r') continue;
                        rxBuf[idx++] = d;
                        if (d == '\n') break;
                        if (idx >= 511) break;
                        to = 2000000;
                    }
                }
                rxBuf[idx] = '\0';

                printf("RX CFG: %s", rxBuf);

                // Parsowanie Configu
                char *ptr;
                ptr = strstr((char*)rxBuf, "\"t_min\":"); if (ptr) { CFG_TEMP_LOW = strtof(ptr + 8, NULL); success=1; }
                ptr = strstr((char*)rxBuf, "\"t_max\":"); if (ptr) { CFG_TEMP_HIGH = strtof(ptr + 8, NULL); success=1; }
                ptr = strstr((char*)rxBuf, "\"sp_min\":"); if (ptr) { CFG_SPO2_LOW = atoi(ptr + 9); success=1; }
                ptr = strstr((char*)rxBuf, "\"hr_min\":"); if (ptr) CFG_HR_LOW = atoi(ptr + 9);
                ptr = strstr((char*)rxBuf, "\"hr_max\":"); if (ptr) CFG_HR_HIGH = atoi(ptr + 9);
                ptr = strstr((char*)rxBuf, "\"gsr1\":"); if (ptr) CFG_GSR_STRESS = atoi(ptr + 7);
                ptr = strstr((char*)rxBuf, "\"gsr2\":"); if (ptr) CFG_GSR_NEUTRAL = atoi(ptr + 7);

                if (success) {
                    printf(">> CONFIG OK!\r\n");
                    return;
                }
            }
        }
        printf("Retry Config...\r\n");
        osDelay(500);
    }
}

#define ECG_BATCH_SIZE 50 // Wysyłamy co 50 próbek
static int32_t ecgBatchBuffer[ECG_BATCH_SIZE];
static uint16_t ecgBatchIndex = 0;

// Bufor na JSON (50 liczb * ~5 cyfr + nagłówek)
char ecgTxBuffer[512];

void myStartDefaultTask(const void *argument)
{
	osDelay(2000);
    const char* hello = "HELLO STM32 UART5\r\n";
    HAL_UART_Transmit(&huart5, (uint8_t*)hello, strlen(hello), HAL_MAX_DELAY);
    osDelay(200);

        // 1. Pobierz czas
	GetTimeFromESP();

	osDelay(200); // Mała przerwa między transakcjami

	// 2. Pobierz konfigurację
	GetConfigFromESP();


    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET)
        {
            printf("STARTUP CHECK: NO WIFI SIGNAL DETECTED!\r\n");

            // Rysuj Czerwony Ekran
            BSP_LCD_Clear(LCD_COLOR_BLACK);
            BSP_LCD_SetBackColor(LCD_COLOR_RED);
            BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
            BSP_LCD_SetFont(&Font24);
            BSP_LCD_DisplayStringAt(0, 120, (uint8_t*)"NO WIFI", CENTER_MODE);
            BSP_LCD_SetFont(&Font16);
            BSP_LCD_DisplayStringAt(0, 160, (uint8_t*)"Check Connection...", CENTER_MODE);

            // Zablokuj system dopóki nie pojawi się stan WYSOKI (WiFi wróci)
            while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET)
            {
                HAL_GPIO_TogglePin(GPIOG, LD3_Pin); // Miganie diodą
                osDelay(200); // Tutaj możemy użyć osDelay bo nie jesteśmy w przerwaniu
            }

            // WiFi wróciło - czyścimy ekran i idziemy dalej
            printf("STARTUP CHECK: WIFI DETECTED!\r\n");
            BSP_LCD_Clear(LCD_COLOR_BLACK);
            DrawHeader();
            DrawBottomButtons();
        }

    printf("Board powering on and starting tasks\r\n");

    if (MLX90614_Init_I2C(&hi2c3, 0x5A)) {
        printf("MLX90614 sensor ready\r\n");
    } else {
        printf("MLX90614 sensor not found\r\n");
    }

    if (BoodOxy_Init_I2C(&hi2c3, 0x57)) {
        printf("BloodOxy sensor ready\r\n");
        BoodOxy_SensorStartCollect();
    } else {
        printf("BloodOxy sensor not found\r\n");
    }
    MAX30003_Init(&max30003, &hspi4, MAX30003_CS_GPIO_Port, MAX30003_CS_Pin);

    HAL_Delay(100);


	if (!MAX30003_ReadDeviceID(&max30003)) {
		printf("MAX30003 ID error (Check Wiring!)\r\n");
	} else {
		printf("MAX30003 ready\r\n");
	}

	systemStartupComplete = 1;
	measurementActive = 0;
	DrawBottomButtons();

	char uart5Msg[256];

        for (;;) {

        	if (measurementActive == 0) {
    			osDelay(100);
    			continue;
    		}

            switch (currentSensor) {

            // ====================================================================
            // CASE 0: TEMPERATURA
            // ====================================================================
            case 0:
                if (MLX90614_ReadAmbientTemp(&mlxData.ambient) == HAL_OK) {
                    if (MLX90614_ReadObjectTemp(&mlxData.object) != HAL_OK) {
                        mlxData.object = 0.0f;
                    }
                    osMessagePut(tempQueueHandle, (uint32_t)&mlxData, 0);

                    uint8_t isAlarm = 0;
                    if (mlxData.object >= CFG_TEMP_HIGH || mlxData.object <= CFG_TEMP_LOW) {
                        isAlarm = 1;
                    }

                    uint32_t timestamp = rtc_base_time + HAL_GetTick() / 1000;
                    char payload[96];
                    snprintf(payload, sizeof(payload),
                             "{\"timestamp\":%lu,\"ambient\":%.2f,\"object\":%.2f,\"alarm\":%d}",
                             timestamp, mlxData.ambient, mlxData.object, isAlarm);

                    uint8_t crc = calcChecksum(payload, strlen(payload));
                    snprintf(uart5Msg, sizeof(uart5Msg), "%s,\"crc\":%d}\r\n", payload, crc);

                    printf("Timestamp: %lu, Ambient: %.2f C, Object: %.2f C, CRC: %d\r\n",
                           timestamp, mlxData.ambient, mlxData.object, crc);

                    // --- POPRAWIONA PĘTLA RETRY ---
                    for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
                        printf("Sent (try %d): %s", attempt + 1, uart5Msg);

                        // 1. Wysyłamy
                        HAL_UART_Transmit(&huart5, (uint8_t*)uart5Msg, strlen(uart5Msg), HAL_MAX_DELAY);

                        // 2. Czekamy na ACK (500ms timeout)
                        if (waitForAck(&huart5, 500)) {
                            printf("ACK received\r\n\n");
                            break; // Sukces, wychodzimy z pętli
                        } else {
                            printf("NACK or Timeout. Retrying...\r\n");
                            osDelay(100); // Krótka przerwa przed ponowieniem
                        }
                    }
                    // ------------------------------

                } else {
                    printf("Temperature read error\r\n");
                }
                osDelay(2000);
                break;

            // ====================================================================
            // CASE 1: SPO2 i PULS
            // ====================================================================
            case 1:
            {
                BoodOxyData tempData;
                if (BoodOxy_GetHeartbeatSPO2(&tempData) == HAL_OK) {
                    oxyData = tempData;

                    if (tempData.spo2 > 0 && tempData.spo2 <= 100 &&
                        tempData.heartbeat > 0 && tempData.heartbeat <= 250) {

                        for (int i = BLOOD_OXY_BUFFER_SIZE - 1; i > 0; i--) {
                            bloodOxyBuffer.spo2_values[i] = bloodOxyBuffer.spo2_values[i-1];
                            bloodOxyBuffer.hr_values[i] = bloodOxyBuffer.hr_values[i-1];
                        }
                        bloodOxyBuffer.spo2_values[0] = tempData.spo2;
                        bloodOxyBuffer.hr_values[0] = tempData.heartbeat;

                        if (bloodOxyBuffer.valid_count < BLOOD_OXY_BUFFER_SIZE) {
                            bloodOxyBuffer.valid_count++;
                        }
                    }

                    if (bloodOxyBuffer.valid_count >= 3) {
                        int32_t avg_spo2 = 0;
                        int32_t avg_hr = 0;
                        for (int i = 0; i < 3; i++) {
                            avg_spo2 += bloodOxyBuffer.spo2_values[i];
                            avg_hr += bloodOxyBuffer.hr_values[i];
                        }
                        avg_spo2 /= 3;
                        avg_hr /= 3;
                        oxyData.spo2 = avg_spo2;
    					oxyData.heartbeat = avg_hr;

    					osMessagePut(oxyQueueHandle, (uint32_t)&oxyData, 0);
    					uint8_t isAlarm = 0;
    					if (avg_spo2 < CFG_SPO2_LOW || avg_hr > CFG_HR_HIGH || avg_hr < CFG_HR_LOW) {
    					    isAlarm = 1;
    					}

                        uint32_t timestamp = rtc_base_time + HAL_GetTick() / 1000;
                        char payload[96];
                        snprintf(payload, sizeof(payload),
                                 "{\"timestamp\":%lu,\"spo2\":%ld,\"hr\":%ld,\"alarm\":%d}",
                                 timestamp, avg_spo2, avg_hr, isAlarm);

                        uint8_t crc = calcChecksum(payload, strlen(payload));
                        snprintf(uart5Msg, sizeof(uart5Msg), "%s,\"crc\":%d}\r\n", payload, crc);

                        printf("Timestamp: %lu, SpO2: %ld%%, HR: %ld bpm, CRC: %d\r\n",
                               timestamp, avg_spo2, avg_hr, crc);

                        // --- POPRAWIONA PĘTLA RETRY ---
                        for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
                            printf("Sent (try %d): %s", attempt + 1, uart5Msg);

                            HAL_UART_Transmit(&huart5, (uint8_t*)uart5Msg, strlen(uart5Msg), HAL_MAX_DELAY);

                            if (waitForAck(&huart5, 500)) {
                                printf("ACK received\r\n\n");
                                break;
                            } else {
                                printf("NACK or Timeout. Retrying...\r\n");
                                osDelay(100);
                            }
                        }
                        // ------------------------------
                        osDelay(5000); // Dłuższa przerwa po udanym pomiarze
                    } else {
                        printf("Waiting for more valid samples (%d/3)...\r\n", bloodOxyBuffer.valid_count);
                        osDelay(1000);
                    }
                } else {
                    printf("BloodOxy read error\r\n");
                    osDelay(1000);
                }
                osDelay(2000);
                break;
            }

            // ====================================================================
            // CASE 2: GSR (STRES)
            // ====================================================================
            case 2:
            {
                ADC_ChannelConfTypeDef sConfig = {0};
                sConfig.Channel = ADC_CHANNEL_13;
                sConfig.Rank = 1;
                sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

                if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

                uint32_t gsrSum = 0;
                for (int i = 0; i < 10; i++) {
                    HAL_ADC_Start(&hadc1);
                    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
                        gsrSum += HAL_ADC_GetValue(&hadc1);
                    }
                }
                uint32_t gsrAvg = gsrSum / 10;

                const char* mood = "Unknown";
                if (gsrAvg < CFG_GSR_STRESS) mood = "Stressed";
                else if (gsrAvg >= CFG_GSR_STRESS && gsrAvg <= CFG_GSR_NEUTRAL) mood = "Neutral";
                else mood = "Relaxed";

                uint8_t isAlarm = 0;
                // Jeśli GSR wskazuje na wysoki stres ("Stressed"), traktujemy to jako alarm
                if (strcmp(mood, "Stressed") == 0) {
                    isAlarm = 1;
                }


                strncpy((char*)gsrData.mood, mood, 16);
                gsrData.value = gsrAvg;
                osMessagePut(gsrQueueHandle, (uint32_t)&gsrData, 0);

                uint32_t timestamp = rtc_base_time + HAL_GetTick() / 1000;
                char payload[128];
                snprintf(payload, sizeof(payload),
                         "{\"timestamp\":%lu,\"gsr\":%lu,\"mood\":\"%s\",\"alarm\":%d}",
                         timestamp, gsrAvg, mood, isAlarm);

                uint8_t crc = calcChecksum(payload, strlen(payload));
                snprintf(uart5Msg, sizeof(uart5Msg), "%s,\"crc\":%d}\r\n", payload, crc);

                printf("Timestamp: %lu, GSR: %lu, Mood: %s, CRC: %d\r\n", timestamp, gsrAvg, mood, crc);

                // --- POPRAWIONA PĘTLA RETRY ---
                for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
                    printf("Sent (try %d): %s", attempt + 1, uart5Msg);

                    HAL_UART_Transmit(&huart5, (uint8_t*)uart5Msg, strlen(uart5Msg), HAL_MAX_DELAY);

                    if (waitForAck(&huart5, 500)) {
                        printf("ACK received\r\n\n");
                        break;
                    } else {
                        printf("NACK or Timeout. Retrying...\r\n");
                        osDelay(100);
                    }
                }
                // ------------------------------
            }
            osDelay(1000);
            break;

            // ====================================================================
            // CASE 3: EKG (STREAMING)
            // ====================================================================
            case 3:
            {
                uint32_t status = MAX30003_ReadReg(&max30003, 0x01);

                if (status & 0x800000) { // Dane dostępne
                    int32_t ecg_val;
                    while(MAX30003_GetECG_Sample(&max30003, &ecg_val)) {

                        osMessagePut(ecgQueueHandle, ecg_val, 0); // Wykres

                        if (ecgBatchIndex < ECG_BATCH_SIZE) {
                            ecgBatchBuffer[ecgBatchIndex++] = ecg_val;
                        }

                        // WYSYŁKA PACZKI
                        if (ecgBatchIndex >= ECG_BATCH_SIZE) {
                            uint32_t timestamp = rtc_base_time + HAL_GetTick() / 1000;

                            // Budowanie JSON
                            int len = snprintf(ecgTxBuffer, sizeof(ecgTxBuffer), "{\"timestamp\":%lu,\"ecg\":[", timestamp);
                            for (int i = 0; i < ECG_BATCH_SIZE; i++) {
                                if (len >= sizeof(ecgTxBuffer) - 20) break;
                                len += snprintf(ecgTxBuffer + len, sizeof(ecgTxBuffer) - len, "%ld%s", ecgBatchBuffer[i], (i < ECG_BATCH_SIZE - 1) ? "," : "");
                            }
                            len += snprintf(ecgTxBuffer + len, sizeof(ecgTxBuffer) - len, "]}");

                            uint8_t crc = calcChecksum(ecgTxBuffer, len);
                            snprintf(ecgTxBuffer + len, sizeof(ecgTxBuffer) - len, ",\"crc\":%d}\r\n", crc);

                            // --- WYSYŁKA EKG Z RETRY ---
                            // Dla EKG retry jest ryzykowne (może zatkać bufor), ale spróbujmy raz lub dwa
                            for (int attempt = 0; attempt < 2; attempt++) {
                                HAL_UART_Transmit(&huart5, (uint8_t*)ecgTxBuffer, strlen(ecgTxBuffer), HAL_MAX_DELAY);

                                // Krótszy timeout dla EKG, bo to strumień
                                if (waitForAck(&huart5, 200)) {
                                    break;
                                }
                                // Jeśli NACK, próbujemy jeszcze raz (pętla)
                            }
                            // ---------------------------

                            ecgBatchIndex = 0; // Reset bufora
                        }
                    }
                }

                // Obsługa błędu FIFO
                if (status & 0x400000) {
                    MAX30003_WriteReg(&max30003, 0x00, 0x000000);
                    MAX30003_WriteReg(&max30003, 0x09, 0x000000);
                    ecgBatchIndex = 0;
                }
                osDelay(2);
                break;
            }

    		} // End Switch
    	} // End For
}

uint8_t calcChecksum(const char* str, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += (uint8_t)str[i];
    }
    return sum;
}


bool waitForAck(UART_HandleTypeDef* uart, uint32_t timeout_ms) {
    // !!! WAŻNE: USUWAMY CZYSZCZENIE BUFORA !!!
    // ESP32 jest teraz tak szybkie, że ACK już tu prawdopodobnie czeka w buforze sprzętowym STM32.
    // Jeśli zrobimy flush/read dummy, to skasujemy właściwy ACK.

    uint8_t ch;
    char ackBuf[32] = {0}; // Zwiększyłem bufor
    int idx = 0;
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms) {
        // Krótki timeout na znak, żeby nie blokować pętli, ale pętla główna trwa timeout_ms
        if (HAL_UART_Receive(uart, &ch, 1, 5) == HAL_OK) {

            // Ignorujemy znak powrotu karetki '\r'
            if (ch == '\r') continue;

            // Jeśli koniec linii '\n' -> kończymy odbieranie
            if (ch == '\n') {
                break;
            }

            // Zbieramy znaki (zabezpieczenie przed przepełnieniem)
            if (idx < sizeof(ackBuf) - 1) {
                ackBuf[idx++] = ch;
            }
        }
    }

    ackBuf[idx] = '\0'; // Zamykamy string

    // Opcjonalnie: Debugowanie, odkomentuj jeśli nadal nie działa
    // if (idx > 0) printf("[DEBUG ACK] Recv: '%s'\r\n", ackBuf);

    // Sprawdzamy czy odebrano ACK
    // Używamy strstr zamiast strcmp, na wypadek gdyby na początku były jakieś śmieci (np. 0x00)
    if (strstr(ackBuf, "ACK") != NULL) return true;

    // Obsługa NACK
    if (strstr(ackBuf, "NACK") != NULL) return false;

    return false;
}


/* To wklej do pliku stm32f4xx_it.c na samym dole */



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(const void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
