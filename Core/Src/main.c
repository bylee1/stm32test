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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define TEMP30_CAL_ADDR      ((uint16_t*)((uint32_t)0x1FFF75A8))
#define TEMP130_CAL_ADDR   ((uint16_t*)((uint32_t)0x1FFF75CA))

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t value_adc=0;
float    value_volt=0.0f;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define TEMP_CAL2                          130
#define TEMP_CAL1                          30
#define TS_CAL1			                      ((uint16_t*) (0x1FFF75A8UL))
#define TS_CAL2			                      ((uint16_t*) (0x1FFF75CAUL))
#define VREFINTCAL							  ((uint16_t*)(0x1FFF75AAUL))

int ADCstep_temp=0;
int ADCstep_VBAT=0;
int ADCstep_VREF=0;
float tempf=0.0f;
int tempi=0;

float VAPLI=3.475;
float VCALIB=3.00;
float VBAT=0;
float VREF=0;
int inc=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* printf IO */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
   HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
   return ch;
}

static void select_ADC_VBAT(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_VBAT;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}



static void select_ADC_VREFINT(void)
{

	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}



static void select_ADC_TEMP(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	char msg_1[80];
		uint8_t power_mode = 1; //(Santiago D) 1 => 3.86mA, 2 =>  <10uA.
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_TIM_Base_Start(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  select_ADC_TEMP();
	  	HAL_ADC_Start(&hadc1);
	  	HAL_ADC_PollForConversion(&hadc1, ADC_TEMPSENSOR_DELAY_US); // Poll ADC1 Perihperal & TimeOut = 1mSec
	  	ADCstep_temp=HAL_ADC_GetValue(&hadc1); // Read The ADC Conversion Result & Map It To PWM DutyCycle
	  HAL_ADC_Stop(&hadc1);

	  	select_ADC_VBAT();
	  	HAL_ADC_Start(&hadc1);
	  	HAL_ADC_PollForConversion(&hadc1, ADC_TEMPSENSOR_DELAY_US); // Poll ADC1 Perihperal & TimeOut = 1mSec
	  	ADCstep_VBAT=HAL_ADC_GetValue(&hadc1); // Read The ADC Conversion Result & Map It To PWM DutyCycle
	  HAL_ADC_Stop(&hadc1);

	  	select_ADC_VREFINT();
	  	HAL_ADC_Start(&hadc1);
	  	HAL_ADC_PollForConversion(&hadc1, ADC_TEMPSENSOR_DELAY_US); // Poll ADC1 Perihperal & TimeOut = 1mSec
	  	ADCstep_VREF=HAL_ADC_GetValue(&hadc1); // Read The ADC Conversion Result & Map It To PWM DutyCycle
	  	HAL_ADC_Stop(&hadc1);

	  	// VREF
	  	VREF =3*(float)((int32_t) *(VREFINTCAL))/ADCstep_VREF;

	  	// VBAT
	  	VBAT =3*VREF*ADCstep_VBAT/4096;//Divider by 3

	  	float TC1=(float)((int32_t) *(TS_CAL1));
	  	float TC2=(float)((int32_t) *(TS_CAL2));

	  	tempf = ((float)TEMP_CAL2-(float)TEMP_CAL1) / (TC2 - TC1) * (ADCstep_temp*VREF/VCALIB - TC1) + 30.0F;


	printf(" temperature : %f\n", tempf);
	  	//uint8_t tempi = round(tempf);
	  	//snprintf(msg_1, sizeof(msg_1), "%0d\n\n", tempi);
	 // HAL_UART_Transmit(&huart2, (uint8_t *)msg_1, strlen(msg_1), 200);

	    	switch(power_mode)
	    	{
	    	case 1:
	    		HAL_Delay(1000);
	    		break;
	    	default:
	    		break;
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */



// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {

//    if(htim->Instance==TIM2)
//    {

//        printf("temperature : %f\n", adc_ReadInternalTemp());


//    }
// }
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
