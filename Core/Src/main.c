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
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "hx711.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define usTIM	TIM4
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float yuzgramdakazanc = 71.93f;								//	20kg	131.2f			100kg	48
float gram_degisim_ofset = 5;									// 	bu degisken kadar degisim varsa azalma durumunu kontrol eder agirlik sensörü ile alakasi yok
float olculen_agirlik=0;



		float ara_ofset=0;
		
		
	float ara_ofset2=0;
	float kalibre_agirligi=25;	// kalibrasyon gram
	

	HX711 HX711_test;
	

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Kalibrasyon(){
	
 
	HX711_test.offset=ara_ofset;							 			// baslangiç degeri varsa araofset o olsun
	
		 for (int i = 0; i < 25; i++)								// 25 adet örnek alip ortalamasini aliyoruz
    {
      ara_ofset += HX711_Value(HX711_test);		
			HAL_Delay(100);
    }
		HX711_test.offset = ara_ofset / 25;						// artik yeni offset degerini girmis olduk
 
		ara_ofset = 0 ;															// bu aradegeri tekrar kullanacagiz
	

		
		for (int i = 0; i < 25; i++)
    {
      ara_ofset += HX711_Value(HX711_test);
			HAL_Delay(100);
    }
		
		ara_ofset2 = ara_ofset / 25;
		
		yuzgramdakazanc = ara_ofset2 / kalibre_agirligi;
}
void usDelay(uint32_t uSec);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Speed of sound in cm/usec
const float speedOfSound = 0.0343/2;
float distance;
char uartBuf[100];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t numTicks = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	HX711_test.gain = 128;
	HX711_test.gpioData = GPIOA;
	HX711_test.pinData = GPIO_PIN_5;
	HX711_test.gpioSck = GPIOA;
	HX711_test.pinSck = GPIO_PIN_7;
	HX711_test.offset =209715200;
	//418116832
//	16724430
	//3.3 ile besle ilk gpio init çalissin offset 0 ile basla onun degerini bul
	
	   HX711_Init(HX711_test);
	// bos agirlikla bu
	
//		 for (int i = 0; i < 25; i++)
//    {
//      ara_ofset += HX711_Value(HX711_test);
//			HAL_Delay(100);
//    }
//		HX711_test.offset = ara_ofset / 25;


// kalibre agirligi ile bu

		for (int i = 0; i < 25; i++)
    {
      ara_ofset += HX711_Value(HX711_test);
			HAL_Delay(100);
    }
		
		ara_ofset2 = ara_ofset / 25;
		
		yuzgramdakazanc = ara_ofset2 / kalibre_agirligi;
	
     olculen_agirlik=0;
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
	// measuring weight
			olculen_agirlik = HX711_Value(HX711_test)*yuzgramdakazanc;
			HAL_Delay(1000);
		
		//the lamb part HAL_Delay(500);			
			if (olculen_agirlik>=1.5e+15)  //1e-15
		{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1, GPIO_PIN_RESET);	
		}
		else 
		{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1, GPIO_PIN_SET);
		}
		//ULTRASONIC PART
		//Set TRIG to LOW for few uSec
		HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
		usDelay(3);
		
		//*** START Ultrasonic measure routine ***//
		//1. Output 10 usec TRIG
		HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
		usDelay(10);
		HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
		
		//2. Wait for ECHO pin rising edge
		while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET);
		
		//3. Start measuring ECHO pulse width in usec
		numTicks = 0;
		while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET)
		{
			numTicks++;
			usDelay(2); //2.8usec
		};
		
		//4. Estimate distance in cm
		distance = (numTicks + 0.0f)*2.8*speedOfSound;
		//the buzzer part
		if (distance<=93.4)
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
			HAL_Delay(500);
		}
		else 
		{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0, GPIO_PIN_RESET);
		}
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
  RCC_OscInitStruct.PLL.PLLN = 180;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void usDelay(uint32_t uSec)
{
	if(uSec < 2) uSec = 2;
	usTIM->ARR = uSec - 1; 	/*sets the value in the auto-reload register*/
	usTIM->EGR = 1; 			/*Re-initialises the timer*/
	usTIM->SR &= ~1; 		//Resets the flag
	usTIM->CR1 |= 1; 		//Enables the counter
	while((usTIM->SR&0x0001) != 1);
	usTIM->SR &= ~(0x0001);
}
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
