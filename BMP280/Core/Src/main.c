/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include"stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP280_REG_ID         0xD0
#define BMP280_REG_RESET      0xE0
#define BMP280_REG_STATUS     0xF3
#define BMP280_REG_CTRL_MEAS  0xF4
#define BMP280_REG_CONFIG     0xF5
#define BMP280_REG_PRESS_MSB  0xF7
#define BMP280_REG_PRESS_LSB  0xF8
#define BMP280_REG_PRESS_XLSB 0xF9
#define BMP280_REG_TEMP_MSB   0xFA
#define BMP280_REG_TEMP_LSB   0xFB
#define BMP280_REG_TEMP_XLSB  0xFC
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
uint8_t BMP280_ReadRegister8(uint8_t reg);
void BMP280_WriteRegister8(uint8_t reg, uint8_t value);
void BMP280_ReadCalibrationData(void);
uint16_t BMP280_Read16_LE(uint8_t reg);
int16_t BMP280_ReadS16_LE(uint8_t reg);
int32_t BMP280_ReadTemperature(void);
uint32_t BMP280_ReadPressure(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart6, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}
uint8_t counter = 0;
uint8_t bmp280_id;
int32_t temperature;
uint32_t pressure;

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
  MX_USART6_UART_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2.0 */
    uint8_t BMP280_ReadRegister8(uint8_t reg) {
      uint8_t value;
      uint8_t addr = reg | 0x80; // Set MSB to 1 for read operation

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // CS pin low
        HAL_SPI_Transmit(&hspi2, &addr, 1, HAL_MAX_DELAY);
        HAL_SPI_Receive(&hspi2, &value, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // CS pin high

        return value;
    }

  void BMP280_WriteRegister8(uint8_t reg, uint8_t value) {
        uint8_t addr = reg & 0x7F; // Set MSB to 0 for write operation

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // CS pin low
        HAL_SPI_Transmit(&hspi2, &addr, 1, HAL_MAX_DELAY);
        HAL_SPI_Transmit(&hspi2, &value, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // CS pin high
    }

    uint16_t BMP280_Read16_LE(uint8_t reg) {
        return (BMP280_ReadRegister8(reg) | (BMP280_ReadRegister8(reg + 1) << 8));
    }

    int16_t BMP280_ReadS16_LE(uint8_t reg) {
        return (int16_t)BMP280_Read16_LE(reg);
    }

  void BMP280_ReadCalibrationData(void) {
        dig_T1 = BMP280_Read16_LE(0x88);
        dig_T2 = BMP280_ReadS16_LE(0x8A);
        dig_T3 = BMP280_ReadS16_LE(0x8C);
        dig_P1 = BMP280_Read16_LE(0x8E);
        dig_P2 = BMP280_ReadS16_LE(0x90);
        dig_P3 = BMP280_ReadS16_LE(0x92);
        dig_P4 = BMP280_ReadS16_LE(0x94);
        dig_P5 = BMP280_ReadS16_LE(0x96);
        dig_P6 = BMP280_ReadS16_LE(0x98);
        dig_P7 = BMP280_ReadS16_LE(0x9A);
        dig_P8 = BMP280_ReadS16_LE(0x9C);
        dig_P9 = BMP280_ReadS16_LE(0x9E);
    }

    int32_t BMP280_ReadTemperature(void) {
        int32_t var1, var2;
        int32_t adc_T = (BMP280_ReadRegister8(BMP280_REG_TEMP_MSB) << 12) | (BMP280_ReadRegister8(BMP280_REG_TEMP_LSB) << 4) | (BMP280_ReadRegister8(BMP280_REG_TEMP_XLSB) >> 4);

        var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
        var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
        int32_t t_fine = var1 + var2;
        return (t_fine * 5 + 128) >> 8;
    }

    uint32_t BMP280_ReadPressure(void) {
        int32_t var1, var2;
        int32_t adc_P = (BMP280_ReadRegister8(BMP280_REG_PRESS_MSB) << 12) | (BMP280_ReadRegister8(BMP280_REG_PRESS_LSB) << 4) | (BMP280_ReadRegister8(BMP280_REG_PRESS_XLSB) >> 4);
        int32_t t_fine = BMP280_ReadTemperature(); // call readTemperature to get t_fine value

        var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
        var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dig_P6);
        var2 = var2 + ((var1 * ((int32_t)dig_P5)) << 1);
        var2 = (var2 >> 2) + (((int32_t)dig_P4) << 16);
        var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dig_P2) * var1) >> 1)) >> 18;
        var1 = ((((32768 + var1)) * ((int32_t)dig_P1)) >> 15);
        if (var1 == 0) {
            return 0; // avoid exception caused by division by zero
        }
        uint32_t p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
        if (p < 0x80000000) {
            p = (p << 1) / ((uint32_t)var1);
        } else {
            p = (p / (uint32_t)var1) * 2;
        }
        var1 = (((int32_t)dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
        var2 = (((int32_t)(p >> 2)) * ((int32_t)dig_P8)) >> 13;
        p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
        return p;
    }
    /* USER CODE END 2.0 */

  /* USER CODE BEGIN 2 */
  bmp280_id = BMP280_ReadRegister8(BMP280_REG_ID);
  if (bmp280_id != 0x58) {
    printf("BMP280 not found! ID = 0x%02X\r\n", bmp280_id);
    Error_Handler();
  } else {
    printf("Happy BMP280 found! ID = 0x%02X\r\n", bmp280_id);
  }

  BMP280_WriteRegister8(BMP280_REG_CTRL_MEAS, 0x27); // Normal mode, temp and press oversampling 1
  BMP280_WriteRegister8(BMP280_REG_CONFIG, 0xA0);    // Standby time 1000ms

  BMP280_ReadCalibrationData();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	    temperature = BMP280_ReadTemperature();
	    pressure = BMP280_ReadPressure();
	    printf("Temperature: %ld.%02ld°C\r\n", temperature / 100, temperature % 100);
	    printf("Pressure: %lu \r\n", pressure);
	    printf("Counter value %d \r\n",counter);
	    counter++;
	    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
	    HAL_Delay(1000);
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
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
