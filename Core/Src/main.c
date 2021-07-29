/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#define __ARM_FEATURE_DSP 1
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "arm_math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FormattedOutput.h"

//If the input buffer is of length N, the output buffer must have length 2*N.
//The input buffer is modified by this function.
#define DOUBLE_BUFFER_SIZE 2048
#define BUFFER_SIZE 1024
#define OUTPUT_BUFFER_SIZE 2048
#define SAMPLE_RATE 93750	//11.25MHz APB2
//1.44 = 12KHz
//11.25/


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
	q15_t analogBuffer[DOUBLE_BUFFER_SIZE];
	q15_t fftOutput[OUTPUT_BUFFER_SIZE];
	uint16_t SMALLanalogBuffer[1];
	uint32_t hashTransmit;
	uint8_t hashBuffer[32];
	
	
void PrintArray_q15_t(q15_t* arrayIn, uint32_t arraySizeIn)
{
	uint8_t numberBuffer[5];
	for(int i=0; i<arraySizeIn; i++)
	{
		NumToStr_int16(arrayIn[i], numberBuffer, 5);
		HAL_UART_Transmit(&huart2,numberBuffer,digitCount_int16_t(arrayIn[i]),1000);
	}
}
	
uint32_t FreqHash(q15_t* fftOut)
{
	uint32_t hashOut=0;
	for(int i=1; i<OUTPUT_BUFFER_SIZE; ++i)
	{
		if(fftOut[i]>fftOut[i-1] && fftOut[i]>fftOut[i+1])
		{
			hashOut |= 1<<(i/65);
		}
	}
	return hashOut;
}

	
	volatile uint8_t txDoneFlag = 1;
uint8_t bootHeader[]="Booted";
uint8_t setSpacer[]={0x22,0x22};

//FFT Items
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
arm_rfft_instance_q15  varInstRfftQ15;
uint32_t timeMissed=0, tempTimeMissed=0;
  arm_status status = ARM_MATH_SUCCESS;
  float32_t maxValue;

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	
  /* Process the data through the CFFT/CIFFT module */
	//perform the fft
  arm_rfft_q15(&varInstRfftQ15, analogBuffer, fftOutput);
	//transmit the fft
	
	//COMPUTE HASH
	hashTransmit = FreqHash(fftOutput);
	//TRANSMIT HASH
	
	NumToStr_uint32(hashTransmit,hashBuffer,32);
	HAL_UART_Transmit(&huart2,hashBuffer,digitCount_uint32_t(hashTransmit),1000);
	txDoneFlag = 0;
	//HAL_UART_Transmit_DMA(&huart2,(uint8_t *)fftOutput,sizeof(fftOutput));
	
	//START SECOND-HALF DMA
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)analogBuffer+BUFFER_SIZE,BUFFER_SIZE);
	
  //status=arm_rfft_32_fast_init_f32(&varInstCfftF32);
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	//perform the fft
  arm_rfft_q15(&varInstRfftQ15, analogBuffer+BUFFER_SIZE, fftOutput);
	//transmit the fft
	
	
	//COMPUTE HASH
	hashTransmit = FreqHash(fftOutput);
	//TRANSMIT HASH
	
	NumToStr_uint32(hashTransmit,hashBuffer,32);
	HAL_UART_Transmit(&huart2,hashBuffer,digitCount_uint32_t(hashTransmit),1000);
	txDoneFlag = 0;
	//HAL_UART_Transmit_DMA(&huart2,(uint8_t *)fftOutput,sizeof(fftOutput));
	
	//START SECOND-HALF DMA
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)analogBuffer,BUFFER_SIZE);
	
	/*
	tempTimeMissed=0;
	while(txDoneFlag == 0)
	{
		tempTimeMissed++;
		HAL_Delay(10);
	}
	timeMissed = tempTimeMissed;
	txDoneFlag = 0;
	*/
	
  //status=arm_rfft_32_fast_init_f32(&varInstCfftF32);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    txDoneFlag = 1;
}

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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
	HAL_UART_Transmit(&huart2, bootHeader, sizeof(bootHeader),1000);
  status = arm_rfft_init_q15(&varInstRfftQ15,BUFFER_SIZE,0,1);
	
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)analogBuffer,BUFFER_SIZE);
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

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
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
