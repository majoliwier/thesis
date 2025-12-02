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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_RETRIES 3
#define LCD_FB_START_ADDRESS 0xD0000000
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

uint16_t lcdWidth  = 0;
uint16_t lcdHeight = 0;


// Buffer for blood oxygen sensor
#define BLOOD_OXY_BUFFER_SIZE 5
typedef struct {
    int16_t spo2_values[BLOOD_OXY_BUFFER_SIZE];
    int32_t hr_values[BLOOD_OXY_BUFFER_SIZE];
    uint8_t valid_count;
    int16_t last_valid_spo2;
    int32_t last_valid_hr;
} BloodOxyBuffer;

static BloodOxyBuffer bloodOxyBuffer = {0};
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
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
uint8_t calcChecksum(const char* str, size_t len);
bool waitForAck(UART_HandleTypeDef* uart, uint32_t timeout_ms);

void StartTouchTask(void const * argument);
void StartRenderTask(void const * argument);
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



void RenderSensorDataToLCD(void)
{
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

    char buf[64];

    switch (currentSensor)
    {
    case 0:
        BSP_LCD_DisplayStringAt(0, 20, (uint8_t *)"TEMPERATURA", CENTER_MODE);

        snprintf(buf, sizeof(buf), "Ambient: %.2f C", mlxData.ambient);
        BSP_LCD_DisplayStringAt(0, 50, (uint8_t*)buf, CENTER_MODE);

        snprintf(buf, sizeof(buf), "Object : %.2f C", mlxData.object);
        BSP_LCD_DisplayStringAt(0, 70, (uint8_t*)buf, CENTER_MODE);
        break;

    case 1:
        BSP_LCD_DisplayStringAt(0, 20, (uint8_t *)"PULSOKSYMETR", CENTER_MODE);

        snprintf(buf, sizeof(buf), "SpO2 : %d %%", oxyData.spo2);
        BSP_LCD_DisplayStringAt(0, 50, (uint8_t*)buf, CENTER_MODE);

        snprintf(buf, sizeof(buf), "HR   : %ld bpm", oxyData.heartbeat);
        BSP_LCD_DisplayStringAt(0, 70, (uint8_t*)buf, CENTER_MODE);
        break;

    case 2:
        BSP_LCD_DisplayStringAt(0, 20, (uint8_t *)"GSR", CENTER_MODE);

        snprintf(buf, sizeof(buf), "GSR not shown");
        BSP_LCD_DisplayStringAt(0, 50, (uint8_t*)buf, CENTER_MODE);
        break;

    default:
        BSP_LCD_DisplayStringAt(0, 50, (uint8_t*)"Invalid sensor", CENTER_MODE);
        break;
    }
}
void StartRenderTask(void const * argument)
{
    printf("Render task START\r\n");
    osDelay(3000);

    while (1)
    {

        RenderSensorDataToLCD();

        osDelay(500);
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) {
        currentSensor = (currentSensor + 1) % 3;
        printf("Przycisk wcisniety - aktualny sensor: %d\r\n", currentSensor);
        HAL_GPIO_TogglePin(GPIOG, LD3_Pin);
    }
}

void myStartDefaultTask(void const * argument);
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
  /* USER CODE BEGIN 2 */
  #define LCD_FRAMEBUFFER       ((uint32_t)0xD0000000)
  #define LCD_WIDTH             240
  #define LCD_HEIGHT            320

  #define COLOR_BLACK           0x0000
  #define COLOR_WHITE           0xFFFF
  #define COLOR_GREEN           0x07E0

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

 

  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity        = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity        = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity        = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity        = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync    = 9;
  hltdc.Init.VerticalSync      = 1;
  hltdc.Init.AccumulatedHBP    = 29;
  hltdc.Init.AccumulatedVBP    = 3;
  hltdc.Init.AccumulatedActiveW = 269;
  hltdc.Init.AccumulatedActiveH = 323;
  hltdc.Init.TotalWidth        = 279;
  hltdc.Init.TotalHeigh        = 327;
  hltdc.Init.Backcolor.Blue    = 0;
  hltdc.Init.Backcolor.Green   = 0;
  hltdc.Init.Backcolor.Red     = 0;

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  PeriphClkInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN          = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR          = 4;
  PeriphClkInitStruct.PLLSAIDivR              = RCC_PLLSAIDIVR_8;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  BSP_LCD_MspInit();

  HAL_LTDC_Init(&hltdc);

  ili9341_Init();

  BSP_SDRAM_Init();

  pLayerCfg.WindowX0        = 0;
  pLayerCfg.WindowX1        = LCD_WIDTH;
  pLayerCfg.WindowY0        = 0;
  pLayerCfg.WindowY1        = LCD_HEIGHT;
  pLayerCfg.PixelFormat     = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha           = 255;
  pLayerCfg.Alpha0          = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress   = LCD_FRAMEBUFFER;
  pLayerCfg.ImageWidth      = LCD_WIDTH;
  pLayerCfg.ImageHeight     = LCD_HEIGHT;
  pLayerCfg.Backcolor.Blue  = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red   = 0;
  HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0);

  ili9341_DisplayOn();
  HAL_Delay(100);

  uint16_t *framebuffer = (uint16_t*)LCD_FRAMEBUFFER;
  for (uint32_t i = 0; i < (LCD_WIDTH * LCD_HEIGHT); ++i) {
      framebuffer[i] = COLOR_BLACK;
  }
  HAL_Delay(50);

  lcdWidth  = LCD_WIDTH;
  lcdHeight = LCD_HEIGHT;
  printf("LCD: %u x %u\r\n", lcdWidth, lcdHeight);

  for (uint16_t y = 80; y < 80 + 60; ++y) {
      for (uint16_t x = 20; x < 20 + 200; ++x) {
          framebuffer[y * LCD_WIDTH + x] = COLOR_GREEN;
      }
  }

  if (BSP_TS_Init(lcdWidth, lcdHeight) == TS_OK) {
      printf("TS init OK\r\n");
  } else {
      printf("TS init ERROR\r\n");
  }


  BSP_LCD_Init();

  BSP_LCD_LayerDefaultInit(0, LCD_FRAMEBUFFER);

  BSP_LCD_SelectLayer(0);

  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font16);

  HAL_Delay(10000);

  BSP_LCD_Clear(LCD_COLOR_BLACK);

  BSP_LCD_DisplayStringAt(0, 20, (uint8_t *)"BSP TEXT TEST", CENTER_MODE);

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
  hltdc.Init.AccumulatedActiveW = 269;
  hltdc.Init.AccumulatedActiveH = 323;
  hltdc.Init.TotalWidth = 279;
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
  huart5.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
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

  /*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin TP_INT1_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin|MEMS_INT2_Pin|TP_INT1_Pin;
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

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void StartTouchTask(void const * argument)
{
    TS_StateTypeDef ts;
    uint8_t lastTouch = 0;

    printf("Touch task started\r\n");
    osDelay(200);

    for (;;)
    {
        BSP_TS_GetState(&ts);

        if (ts.TouchDetected && !lastTouch)
        {
            uint16_t x = ts.X;
            uint16_t y = ts.Y;

            printf("Touch: X=%u Y=%u\r\n", x, y);
            currentSensor = (currentSensor + 1) % 3;
            printf("Sensor changed to %d\r\n", currentSensor);
        }

        lastTouch = ts.TouchDetected ? 1 : 0;

        osDelay(50);
    }
}

void RequestRTCfromESP32(void)
{
    const char* request = "GET_RTC\n";
    uint8_t rxBuffer[64] = {0};
    uint8_t ch;
    int idx = 0;

    // Wyślij żądanie
    HAL_UART_Transmit(&huart5, (uint8_t*)request, strlen(request), HAL_MAX_DELAY);
    printf("Requesting RTC from ESP32...\r\n");

    uint8_t dump;
    while (HAL_UART_Receive(&huart5, &dump, 1, 5) == HAL_OK);

    osDelay(100);

    // Czytaj znak po znaku aż do \n lub timeout
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 5000 && idx < sizeof(rxBuffer) - 1) {
        if (HAL_UART_Receive(&huart5, &ch, 1, 100) == HAL_OK) {
            rxBuffer[idx++] = ch;
            if (ch == '\n') break;
        }
    }

    rxBuffer[idx] = '\0';  // null-terminate

    if (idx > 0) {
    	printf("Received raw bytes: ");
    	for (int i = 0; i < idx; i++) {
    	    printf("[%02X]", rxBuffer[i]);
    	}
    	printf("\r\n");
        char *ptr = strstr((char*)rxBuffer, "\"rtc\":");
        if (ptr) {
        	const char* ack = "ACK\r\n";
			HAL_UART_Transmit(&huart5, (uint8_t*)ack, strlen(ack), HAL_MAX_DELAY);
            rtc_base_time = atoi(ptr + 6);
            printf("RTC base time received: %lu\r\n", rtc_base_time);

        } else {
            printf("RTC format invalid.\r\n");
        }
    } else {
        printf("Timeout: no response from ESP32.\r\n");
    }
}

void myStartDefaultTask(void const * argument)
{
    osDelay(1000);
    const char* hello = "HELLO STM32 UART5\r\n";
    HAL_UART_Transmit(&huart5, (uint8_t*)hello, strlen(hello), HAL_MAX_DELAY);
    osDelay(200);
    RequestRTCfromESP32();

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

    char uart5Msg[128];
    for (;;) {
        switch (currentSensor) {

        case 0:
            if (MLX90614_ReadAmbientTemp(&mlxData.ambient) == HAL_OK) {
                if (MLX90614_ReadObjectTemp(&mlxData.object) != HAL_OK) {
                    printf("MLX90614 object read failed\r\n");
                    mlxData.object = 0.0f;
                }

                uint32_t timestamp = rtc_base_time + HAL_GetTick() / 1000;
                char payload[64];
                snprintf(payload, sizeof(payload),
                         "{\"timestamp\":%lu,\"ambient\":%.2f,\"object\":%.2f}",
                         timestamp, mlxData.ambient, mlxData.object);

                uint8_t crc = calcChecksum(payload, strlen(payload));
                printf("Timestamp: %lu, Ambient: %.2f C, Object: %.2f C, CRC: %d\r\n",
                       timestamp, mlxData.ambient, mlxData.object, crc);

                snprintf(uart5Msg, sizeof(uart5Msg),
                         "%s,\"crc\":%d}\r\n", payload, crc);

                HAL_UART_Transmit(&huart5, (uint8_t*)uart5Msg, strlen(uart5Msg), HAL_MAX_DELAY);
                for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
                    if (attempt == 0) osDelay(10);
                    HAL_UART_Transmit(&huart5, (uint8_t*)uart5Msg, strlen(uart5Msg), HAL_MAX_DELAY);
                    printf("Sent (try %d): %s", attempt + 1, uart5Msg);

                    if (waitForAck(&huart5, 500)) {
                        printf("ACK received\r\n\n");
                        break;
                    } else {
                        printf("NACK or no response — retrying...\r\n");
                    }
                }
            } else {
                printf("Temperature read error\r\n");
            }
            osDelay(5000);
            break;

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

                    bloodOxyBuffer.last_valid_spo2 = tempData.spo2;
                    bloodOxyBuffer.last_valid_hr = tempData.heartbeat;
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

                    uint32_t timestamp = rtc_base_time + HAL_GetTick() / 1000;
                    char payload[64];
                    snprintf(payload, sizeof(payload),
                             "{\"timestamp\":%lu,\"spo2\":%ld,\"hr\":%ld}",
                             timestamp, avg_spo2, avg_hr);

                    uint8_t crc = calcChecksum(payload, strlen(payload));
                    printf("Timestamp: %lu, SpO2: %ld%%, HR: %ld bpm, CRC: %d\r\n",
                           timestamp, avg_spo2, avg_hr, crc);

                    snprintf(uart5Msg, sizeof(uart5Msg),
                             "%s,\"crc\":%d}\r\n", payload, crc);

                    HAL_UART_Transmit(&huart5, (uint8_t*)uart5Msg, strlen(uart5Msg), HAL_MAX_DELAY);
                    for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
                        if (attempt == 0) osDelay(10);
                        HAL_UART_Transmit(&huart5, (uint8_t*)uart5Msg, strlen(uart5Msg), HAL_MAX_DELAY);
                        printf("Sent (try %d): %s", attempt + 1, uart5Msg);

                        if (waitForAck(&huart5, 500)) {
                            printf("ACK received\r\n\n");
                            break;
                        } else {
                            printf("NACK or no response — retrying...\r\n");
                        }
                    }
                    osDelay(5000);
                } else {
                    printf("Waiting for more valid samples (%d/3)...\r\n", bloodOxyBuffer.valid_count);
                    osDelay(1000);
                }
            } else {
                printf("BloodOxy read error\r\n");
                osDelay(1000);
            }
            break;
        }

        case 2:
        {
            ADC_ChannelConfTypeDef sConfig = {0};
            sConfig.Channel = ADC_CHANNEL_13;
            sConfig.Rank = 1;
            sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

            if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
                Error_Handler();
            }

            uint32_t gsrSum = 0;
            for (int i = 0; i < 10; i++) {
                HAL_ADC_Start(&hadc1);
                if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
                    gsrSum += HAL_ADC_GetValue(&hadc1);
                }
            }
            uint32_t gsrAvg = gsrSum / 10;

            const char* mood = "Unknown";
            if (gsrAvg < 400) {
                mood = "Stressed";
            } else if (gsrAvg >= 400 && gsrAvg <= 1000) {
                mood = "Neutral";
            } else {
                mood = "Relaxed";
            }

            uint32_t timestamp = rtc_base_time + HAL_GetTick() / 1000;
            char payload[96];
            snprintf(payload, sizeof(payload),
                     "{\"timestamp\":%lu,\"gsr\":%lu,\"mood\":\"%s\"}",
                     timestamp, gsrAvg, mood);

            uint8_t crc = calcChecksum(payload, strlen(payload));
            snprintf(uart5Msg, sizeof(uart5Msg), "%s,\"crc\":%d}\r\n", payload, crc);

            printf("Timestamp: %lu, GSR: %lu, Mood: %s, CRC: %d\r\n",
                   timestamp, gsrAvg, mood, crc);

            HAL_UART_Transmit(&huart5, (uint8_t*)uart5Msg, strlen(uart5Msg), HAL_MAX_DELAY);

            for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
                if (attempt == 0) osDelay(10);
                HAL_UART_Transmit(&huart5, (uint8_t*)uart5Msg, strlen(uart5Msg), HAL_MAX_DELAY);
                printf("Sent (try %d): %s", attempt + 1, uart5Msg);

                if (waitForAck(&huart5, 500)) {
                    printf("ACK received\r\n\n");
                    break;
                } else {
                    printf("NACK or no response — retrying...\r\n");
                }
            }
        }
        osDelay(1000);
        break;
        }
    }
}

uint8_t calcChecksum(const char* str, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += (uint8_t)str[i];
    }
    return sum;
}


bool waitForAck(UART_HandleTypeDef* uart, uint32_t timeout_ms) {
    uint8_t dummy;
    while (HAL_UART_Receive(uart, &dummy, 1, 1) == HAL_OK);

    uint8_t ch;
    char ackBuf[16] = {0};
    int idx = 0;
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms && idx < sizeof(ackBuf) - 1) {
        if (HAL_UART_Receive(uart, &ch, 1, 10) == HAL_OK) {
            if (ch == '\r') continue;
            if (ch == '\n') break;
            ackBuf[idx++] = ch;
        }
    }

    ackBuf[idx] = '\0';


    if (strcmp(ackBuf, "ACK") == 0) return true;
    if (strcmp(ackBuf, "NACK") == 0) return false;

    return false;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
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
