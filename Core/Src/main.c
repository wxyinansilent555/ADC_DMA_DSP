/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "retarget.h"
#include "arm_const_structs.h"
#include "arm_math.h"
#include "dsp/window_functions.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_LENGTH 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
__IO uint8_t AdcConvEnd = 0;

uint16_t adcBuff_1[FFT_LENGTH], adcBuff_3[FFT_LENGTH];
float32_t fft_inputbuf_1[FFT_LENGTH * 2], fft_inputbuf_3[FFT_LENGTH * 2];
float32_t fft_outputbuf_1[FFT_LENGTH], fft_outputbuf_3[FFT_LENGTH];

float32_t Hanning_window[FFT_LENGTH];

float32_t theta_1,theta_3,theta;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart1);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuff_1, 1024); //让ADC1去采集200个数，存放到adc_buff数组里
  HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adcBuff_3, 1024); //让ADC1去采集200个数，存放到adc_buff数组里
  HAL_TIM_Base_Start(&htim3);                           //开启定时器3

  while (!AdcConvEnd)                                   //等待转换完毕
        ;
  for (int i = 0; i < FFT_LENGTH; i++)
  {
      fft_inputbuf_1[i * 2] = adcBuff_1[i] * 3.3 / 4096;//实部赋值，* 3.3 / 4096是为了将ADC采集到的值转换成实际电压
      fft_inputbuf_3[i * 2] = adcBuff_3[i] * 3.3 / 4096;//实部赋值，* 3.3 / 4096是为了将ADC采集到的值转换成实际电压
      fft_inputbuf_1[i * 2 + 1] = 0;//虚部赋值，固定为0.
      fft_inputbuf_3[i * 2 + 1] = 0;//虚部赋值，固定为0.
  }

  arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf_1, 0, 1);
  arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf_3, 0, 1);
  arm_cmplx_mag_f32(fft_inputbuf_1, fft_outputbuf_1, FFT_LENGTH);
  arm_cmplx_mag_f32(fft_inputbuf_3, fft_outputbuf_3, FFT_LENGTH);

  /*for(uint32_t i = 0; i < 350; i ++)
  {
      printf("%d:\t%f\n",i,fft_inputbuf_1[i]);
  }*/

  theta_1 = atan2(fft_inputbuf_1[321],fft_inputbuf_1[320]);
  theta_3 = atan2(fft_inputbuf_3[321],fft_inputbuf_3[320]);
  theta   = (theta_1 - theta_3) * 180.0 / 3.1415926;
  printf("theta:\t%.3f\n",theta);

  AdcConvEnd = 0;

  /*fft_outputbuf_1[0] /= 1024;
  fft_outputbuf_3[0] /= 1024;

  for (int i = 1; i < FFT_LENGTH; i++)//输出各次谐波幅值
  {
      fft_outputbuf_1[i] /= 512;
      fft_outputbuf_3[i] /= 512;
  }
  printf("FFT Result:\r\n");

  for (int i = 0; i < FFT_LENGTH/2; i++)//输出各次谐波幅值
  {
      printf("%d:\t%f\r\n", i, fft_outputbuf_3[i]);
  }*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuff_1, 1024); //让ADC1去采集200个数，存放到adc_buff数组里
      HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adcBuff_3, 1024); //让ADC1去采集200个数，存放到adc_buff数组里
      HAL_TIM_Base_Start(&htim3);                           //开启定时器3

      while (!AdcConvEnd)                                   //等待转换完毕
          ;
      for (int i = 0; i < FFT_LENGTH; i++)
      {
          fft_inputbuf_1[i * 2] = adcBuff_1[i] * 3.3 / 4096;//实部赋值，* 3.3 / 4096是为了将ADC采集到的值转换成实际电压
          fft_inputbuf_3[i * 2] = adcBuff_3[i] * 3.3 / 4096;//实部赋值，* 3.3 / 4096是为了将ADC采集到的值转换成实际电压
          fft_inputbuf_1[i * 2 + 1] = 0;//虚部赋值，固定为0.
          fft_inputbuf_3[i * 2 + 1] = 0;//虚部赋值，固定为0.
      }

      arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf_1, 0, 1);
      arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf_3, 0, 1);
      arm_cmplx_mag_f32(fft_inputbuf_1, fft_outputbuf_1, FFT_LENGTH);
      arm_cmplx_mag_f32(fft_inputbuf_3, fft_outputbuf_3, FFT_LENGTH);

      /*for(uint32_t i = 0; i < 350; i ++)
      {
          printf("%d:\t%f\n",i,fft_inputbuf_1[i]);
      }*/

      theta_1 = atan2(fft_inputbuf_1[321],fft_inputbuf_1[320]);
      theta_3 = atan2(fft_inputbuf_3[321],fft_inputbuf_3[320]);
      theta   = (theta_1 - theta_3) * 180.0 / 3.1415926;
      printf("theta:\t%.3f\n",theta);

      AdcConvEnd = 0;
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
  RCC_OscInitStruct.PLL.PLLN = 144;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
