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
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ugvMain.h"
#include "crc15.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

HAL_StatusTypeDef status;
uint32_t tickTime = 0;

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  init_PEC15_Table();

  LoraData loraTxTry =
  {
		  .azimuth = 50,
		  .latitudeDegree = 49,
		  .latitudeMinute = 30,
		  .latitudeSecond = 3.14f,
		  .longitudeDegree = 55,
		  .longitudeMinute = 20,
		  .longitudeSecond = 3.14f,
		  .numberOfSatellite = 4,
		  .speed = 3.14f,
		  .gpsState = WRONG_DATA,
		  .ledState = FRONT_LED_ON,
		  .crcLsb = 0,
		  .crcMsb = 0,
		  .carriage = '\r',
		  .newline = '\n'
  };

  char dataBuffer[25];
  uint32_t u32TempVar;

  dataBuffer[0] = (loraTxTry.azimuth >> 0) & 0xFF;
  dataBuffer[1] = (loraTxTry.azimuth >> 8) & 0xFF;
  dataBuffer[2] = loraTxTry.latitudeDegree;
  dataBuffer[3] = loraTxTry.latitudeMinute;
  u32TempVar = *((uint32_t*)&loraTxTry.latitudeSecond);
  dataBuffer[4] = (u32TempVar >> 0) & 0xFF;
  dataBuffer[5] = (u32TempVar >> 8) & 0xFF;
  dataBuffer[6] = (u32TempVar >> 16) & 0xFF;
  dataBuffer[7] = (u32TempVar >> 24) & 0xFF;
  dataBuffer[8] = loraTxTry.longitudeDegree;
  dataBuffer[9] = loraTxTry.longitudeMinute;
  u32TempVar = *((uint32_t*)&loraTxTry.longitudeSecond);
  dataBuffer[10] = (u32TempVar >> 0) & 0xFF;
  dataBuffer[11] = (u32TempVar >> 8) & 0xFF;
  dataBuffer[12] = (u32TempVar >> 16) & 0xFF;
  dataBuffer[13] = (u32TempVar >> 24) & 0xFF;
  dataBuffer[14] = loraTxTry.numberOfSatellite;
  u32TempVar = *((uint32_t*)&loraTxTry.speed);
  dataBuffer[15] = (u32TempVar >> 0) & 0xFF;
  dataBuffer[16] = (u32TempVar >> 8) & 0xFF;
  dataBuffer[17] = (u32TempVar >> 16) & 0xFF;
  dataBuffer[18] = (u32TempVar >> 24) & 0xFF;
  dataBuffer[19] = loraTxTry.ledState;
  dataBuffer[20] = loraTxTry.gpsState;

  uint16_t pec = AE_pec15((uint8_t*)dataBuffer, 21);
  dataBuffer[21] = (pec >> 0) & 0xFF;
  dataBuffer[22] = (pec >> 8) & 0xFF;
  dataBuffer[23] = loraTxTry.carriage;
  dataBuffer[24] = loraTxTry.newline;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  uint32_t tick1 = HAL_GetTick();
	  status = HAL_UART_Transmit(&huart2, (uint8_t*)dataBuffer, sizeof(dataBuffer), 10);

	  uint32_t tick2 = HAL_GetTick();

	  tickTime = tick2 - tick1;
	  HAL_Delay(40);


//	  ugvMain();
//	  break;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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
