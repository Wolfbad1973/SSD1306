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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Адрес дисплея SSD1306 на шине I2C
#define SSD1306_I2C_ADDRESS 0x3C

// Размеры дисплея
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
// Буфер для хранения данных дисплея
uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  uint8_t a=0;
  uint8_t b=0;
  // Функция для отправки команды на SSD1306
  void SSD1306_WriteCommand(I2C_HandleTypeDef *hi2c, uint8_t command) {
      uint8_t data[2] = {0x00, command};  // 0x00 — контрольный байт для команды
      HAL_I2C_Master_Transmit(hi2c, SSD1306_I2C_ADDRESS << 1, data, 2, HAL_MAX_DELAY);
  }

  // Функция для отправки данных на SSD1306
  void SSD1306_WriteData(I2C_HandleTypeDef *hi2c, uint8_t *data, uint16_t size) {
      HAL_I2C_Mem_Write(hi2c, SSD1306_I2C_ADDRESS << 1, 0x40, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
  }

  // �?нициализация дисплея SSD1306
  void SSD1306_Init(I2C_HandleTypeDef *hi2c) {
      SSD1306_WriteCommand(hi2c, 0xAE);  // Выключить дисплей
      SSD1306_WriteCommand(hi2c, 0xD5);  // Настройка частоты обновления
      SSD1306_WriteCommand(hi2c, 0x80);
      SSD1306_WriteCommand(hi2c, 0xA8);  // Настройка мультиплексирования
      SSD1306_WriteCommand(hi2c, 0x3F);
      SSD1306_WriteCommand(hi2c, 0xD3);  // Настройка смещения дисплея
      SSD1306_WriteCommand(hi2c, 0x00);
      SSD1306_WriteCommand(hi2c, 0x40);  // Начальная строка
      SSD1306_WriteCommand(hi2c, 0x8D);  // Настройка зарядового насоса
      SSD1306_WriteCommand(hi2c, 0x14);
      SSD1306_WriteCommand(hi2c, 0x20);  // Режим адресации
      SSD1306_WriteCommand(hi2c, 0x00);
      SSD1306_WriteCommand(hi2c, 0xA1);  // Сегментное отображение
      SSD1306_WriteCommand(hi2c, 0xC8);  // Обратное сканирование COM
      SSD1306_WriteCommand(hi2c, 0xDA);  // Настройка конфигурации COM
      SSD1306_WriteCommand(hi2c, 0x12);
      SSD1306_WriteCommand(hi2c, 0x81);  // Настройка контрастности
      SSD1306_WriteCommand(hi2c, 0xCF);
      SSD1306_WriteCommand(hi2c, 0xD9);  // Настройка предзаряда
      SSD1306_WriteCommand(hi2c, 0xF1);
      SSD1306_WriteCommand(hi2c, 0xDB);  // Настройка VCOMH
      SSD1306_WriteCommand(hi2c, 0x40);
      SSD1306_WriteCommand(hi2c, 0xA4);  // Отображение RAM
      SSD1306_WriteCommand(hi2c, 0xA6);  // Нормальное отображение (не инвертированное)
      SSD1306_WriteCommand(hi2c, 0xAF);  // Включить дисплей
  }

  // Функция для установки пикселя в определённой точке
  void SSD1306_SetPixel(uint8_t x, uint8_t y, uint8_t color)
  {
      if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;  // Проверка границ

      if (color) {
          SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));  // Включить пиксель
      } else {
          SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8)); // Выключить пиксель
      }
  }

  // Функция для обновления дисплея
  void SSD1306_UpdateDisplay(I2C_HandleTypeDef *hi2c) {
      for (uint8_t i = 0; i < 8; i++) {
          SSD1306_WriteCommand(hi2c, 0xB0 + i);  // Установка страницы
          SSD1306_WriteCommand(hi2c, 0x00);      // Младший байт столбца
          SSD1306_WriteCommand(hi2c, 0x10);      // Старший байт столбца

          // Отправка данных для текущей страницы
          SSD1306_WriteData(hi2c, &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);
      }
  }



  // �?нициализация дисплея
  SSD1306_Init(&hi2c1);

  // Очистка буфера
  memset(SSD1306_Buffer, 0, sizeof(SSD1306_Buffer));

  // Установка пикселя в точке (10, 10)

  SSD1306_SetPixel(1, 1, 1);
  SSD1306_SetPixel(1, 10, 1);


  // Обновление дисплея
     SSD1306_UpdateDisplay(&hi2c1);
     uint16_t adc = 0;
     HAL_ADCEx_Calibration_Start(&hadc1);
     uint32_t  timme = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      HAL_ADC_Start(&hadc1); // запускаем преобразование сигнала АЦП
      HAL_ADC_PollForConversion(&hadc1, 100); // ожидаем окончания преобразования
      adc = HAL_ADC_GetValue(&hadc1); // читаем полученное значение в переменную adc
	  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	  if((HAL_GetTick() - timme) > 1000) // интервал 1000мс = 1сек
	          {
		     SSD1306_UpdateDisplay(&hi2c1);

	                  // что-то делаем
	                  timme = HAL_GetTick();
	          }



      a=(a+1)%127;
      b=adc/5;
	  SSD1306_SetPixel(a, b, 1);


	  if (a>125)
	  {
		  memset(SSD1306_Buffer, 0, sizeof(SSD1306_Buffer));


	  }


	  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);


//	  	HAL_Delay(1000);

//	  	HAL_Delay(1000);
//	  	SSD1306_UpdateDisplay(&hi2c1);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
