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

#define ADXL357_CS_PIN GPIO_PIN_9
#define ADXL357_CS_PORT GPIOB
#define SENSITIVITY_10G 51200.0

// Memory register addresses
#define DEVID_AD  0x00
#define DEVID_MST 0x01
#define PARTID 0x02
#define XDATA3 0x08
#define XDATA2 0x09
#define XDATA1 0x0A
#define YDATA3 0x0B
#define YDATA2 0x0C
#define YDATA1 0x0D
#define ZDATA3 0x0E
#define ZDATA2 0x0F
#define ZDATA1 0x10
#define RANGE  0x2C
#define POWER_CTL 0x2D

// Device values
#define RANGE_10G 0x01
#define MEASURE_MODE 0x06  // Only accelerometer

// Operations
#define READ_BYTE 0x01
#define WRITE_BYTE 0x00

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
void writeRegister(uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t reg);
void readMultipleData(uint8_t* addresses, uint8_t dataSize, uint8_t* readData);
void initADXL357(void);
void readADXL357Data(void);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart6, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}
/* USER CODE END 0 */

uint8_t count = 0;

/* USER CODE BEGIN 1 */
void writeRegister(uint8_t reg, uint8_t value) {
    uint8_t dataToSend = (reg << 1) | WRITE_BYTE;
    HAL_GPIO_WritePin(ADXL357_CS_PORT, ADXL357_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &dataToSend, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi2, &value, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ADXL357_CS_PORT, ADXL357_CS_PIN, GPIO_PIN_SET);
  }

  uint8_t readRegister(uint8_t reg) {
    uint8_t result = 0;
    uint8_t dataToSend = (reg << 1) | READ_BYTE;

    HAL_GPIO_WritePin(ADXL357_CS_PORT, ADXL357_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &dataToSend, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, &result, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ADXL357_CS_PORT, ADXL357_CS_PIN, GPIO_PIN_SET);

    return result;
  }

  void readMultipleData(uint8_t* addresses, uint8_t dataSize, uint8_t* readData) {
    HAL_GPIO_WritePin(ADXL357_CS_PORT, ADXL357_CS_PIN, GPIO_PIN_RESET);
    for (int i = 0; i < dataSize; i++) {
      uint8_t dataToSend = (addresses[i] << 1) | READ_BYTE;
      HAL_SPI_Transmit(&hspi2, &dataToSend, 1, HAL_MAX_DELAY);
      HAL_SPI_Receive(&hspi2, &readData[i], 1, HAL_MAX_DELAY);
    }
    HAL_GPIO_WritePin(ADXL357_CS_PORT, ADXL357_CS_PIN, GPIO_PIN_SET);
  }

  void initADXL357(void) {
    HAL_GPIO_WritePin(ADXL357_CS_PORT, ADXL357_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(1000);  // Wait for ADXL357 to settle

    uint8_t devid_ad = readRegister(DEVID_AD); // Device ID
    uint8_t devid_mst = readRegister(DEVID_MST); // MEMS ID
    uint8_t part_id = readRegister(PARTID);   // PART ID

    printf("Device ID AD: 0x%X ", devid_ad);
    printf("Device ID MST: 0x%X ", devid_mst);
    printf("Part ID: 0x%X ", part_id);

    if (devid_ad != 0xAD || devid_mst != 0x1D || part_id != 0xED) {
      printf("Error: Device ID mismatch\n");
      while (1);
    } else {
      printf("Device ID matched successfully.\n");
    }

    writeRegister(RANGE, RANGE_10G); // Set range to ±10g
    writeRegister(POWER_CTL, MEASURE_MODE); // Enable measure mode

    // Give the sensor time to set up
    HAL_Delay(100);
  }

  void readADXL357Data(void) {
    uint8_t addresses[] = {XDATA3, XDATA2, XDATA1, YDATA3, YDATA2, YDATA1, ZDATA3, ZDATA2, ZDATA1};
    uint8_t data[9];
    int32_t xData, yData, zData;

    readMultipleData(addresses, 9, data);

    // Combine data to form 20-bit values
    xData = ((data[0] << 16) | (data[1] << 8) | data[2]) >> 4;
    yData = ((data[3] << 16) | (data[4] << 8) | data[5]) >> 4;
    zData = ((data[6] << 16) | (data[7] << 8) | data[8]) >> 4;

    // Apply two's complement for 20-bit signed integers
    if (xData & (1 << 19)) xData |= 0xFFF00000;
    if (yData & (1 << 19)) yData |= 0xFFF00000;
    if (zData & (1 << 19)) zData |= 0xFFF00000;

    // Convert raw data to g-units
    float xAccel = xData / SENSITIVITY_10G;
    float yAccel = yData / SENSITIVITY_10G;
    float zAccel = zData / SENSITIVITY_10G;

    printf("X: %.2f g, Y: %.2f g, Z: %.2f g\n", xAccel, yAccel, zAccel);
  }

  /* USER CODE END 1 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  MX_SPI2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize ADXL357
  initADXL357();

  /* USER CODE END 2 */

  /* USER CODE BEGIN 3 */
    /* USER CODE END 3 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	    readADXL357Data();
	    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
	    printf("\r\n");
	    count++;
	    HAL_Delay(100);

  }

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
