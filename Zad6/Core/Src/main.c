/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <l496_mems.h>
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
const uint8_t segments[] = {
	SEG_A_PIN | SEG_B_PIN | SEG_C_PIN | SEG_D_PIN | SEG_E_PIN | SEG_F_PIN, // 0
	SEG_B_PIN | SEG_C_PIN, // 1
	SEG_A_PIN | SEG_B_PIN | SEG_G_PIN | SEG_E_PIN | SEG_D_PIN, // 2
	SEG_A_PIN | SEG_B_PIN | SEG_G_PIN | SEG_C_PIN | SEG_D_PIN, //3
	SEG_F_PIN | SEG_G_PIN | SEG_C_PIN | SEG_B_PIN, //4
	SEG_A_PIN | SEG_F_PIN | SEG_G_PIN | SEG_C_PIN | SEG_D_PIN, // 5
	SEG_A_PIN | SEG_F_PIN | SEG_E_PIN | SEG_D_PIN | SEG_C_PIN | SEG_G_PIN,// 6
	SEG_A_PIN | SEG_B_PIN | SEG_C_PIN, //7
	SEG_A_PIN | SEG_B_PIN | SEG_C_PIN | SEG_D_PIN | SEG_E_PIN | SEG_F_PIN | SEG_G_PIN, //8
	SEG_A_PIN | SEG_B_PIN | SEG_C_PIN | SEG_D_PIN | SEG_F_PIN | SEG_G_PIN, // 9
	SEG_B_PIN | SEG_F_PIN | SEG_G_PIN | SEG_E_PIN | SEG_C_PIN, // X
	SEG_B_PIN | SEG_F_PIN | SEG_G_PIN | SEG_C_PIN, // Y
	SEG_A_PIN | SEG_B_PIN | SEG_G_PIN | SEG_E_PIN | SEG_D_PIN // Z
};

const uint16_t digit[] = {DIG4_PIN, DIG3_PIN, DIG2_PIN, DIG1_PIN};
uint16_t display[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void Joy_Init(void);
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  Joy_Init();
  Dis7Seg_Init();
  (void)mems_init(MEMS_ACC_DATARATE_10HZ, MEMS_ACC_FULLSCALE_2G);

  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  MX_TIM2_Init();
  HAL_TIM_Base_Start_IT(&htim2);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/* USER CODE BEGIN 4 */
static void Joy_Init(void)
{
	GPIO_InitTypeDef joyInitStruct = {0};

	__HAL_RCC_GPIOE_CLK_ENABLE();

	joyInitStruct.Mode = GPIO_MODE_INPUT;
	joyInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	joyInitStruct.Pull = GPIO_NOPULL;
	joyInitStruct.Pin = JOY_UP_PIN |
					 	JOY_DOWN_PIN |
						JOY_LEFT_PIN |
						JOY_RIGHT_PIN |
						JOY_CENTER_PIN;

	HAL_GPIO_Init(GPIOE, &joyInitStruct);
}

void Dis7Seg_Init(void)
{
	GPIO_InitTypeDef initStruct = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	HAL_PWREx_EnableVddIO2();

	initStruct.Pin = DIG1_PIN | DIG2_PIN | DIG3_PIN | DIG4_PIN;
	initStruct.Mode = GPIO_MODE_OUTPUT_PP;
	initStruct.Pull = GPIO_NOPULL;
	initStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DIG_PORT, &initStruct);

	initStruct.Pin = SEG_A_PIN | SEG_B_PIN | SEG_C_PIN |
				   	 SEG_D_PIN | SEG_E_PIN | SEG_F_PIN |
					 SEG_G_PIN | SEG_DP_PIN;
	HAL_GPIO_Init(SEG_PORT, &initStruct);
}

void Dis7seg_display(uint8_t dotSecondPlace)
{
	static uint8_t dig = 0;

	HAL_GPIO_WritePin(DIG_PORT, DIG_MASK, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEG_PORT, SEG_MASK, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEG_PORT, display[dig], GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIG_PORT, digit[dig], GPIO_PIN_SET);

	if (dotSecondPlace && dig == 1)
	{
		HAL_GPIO_WritePin(SEG_PORT, SEG_DP_PIN, GPIO_PIN_SET);
	}
	else if (dig == 2)
	{
		HAL_GPIO_WritePin(SEG_PORT, SEG_DP_PIN, GPIO_PIN_SET);
	}

	dig = (dig + 1) & 3;
}

void Dis7Seg_SetAxisValue(const selectedAxisType * const selectedAxis, uint8_t * dotSecondPlace)
{
	float axisValue = 0;

	switch (*selectedAxis) {
		case X:
			axisValue = mems_acc_read_x();
			break;
		case Y:
			axisValue = mems_acc_read_y();
			break;
		case Z:
		default:
			axisValue = mems_acc_read_z();
			break;
	}

	axisValue *= 2.0F / MEMS_ACC_MAXVAL;

	if (axisValue < 0)
	{
		axisValue *= -1.0F;
	}

	if (axisValue / 10.0F < 1.0F)
	{
		display[2] = segments[(uint8_t)(axisValue / 1.0F)];
		axisValue -= (uint8_t)(axisValue / 1.0F);
		display[1] = segments[(uint8_t)(axisValue / 0.1F)];
		axisValue -= ((uint8_t)(axisValue / 0.1F) * 0.1F);
		display[0] = segments[(uint8_t)(axisValue / 0.01F)];
		axisValue -= ((uint8_t)(axisValue / 0.01F) * 0.01F);
		*dotSecondPlace = TRUE;
	}
	else
	{
		display[2] = segments[(uint8_t)(axisValue / 10.0F)];
		axisValue -= ((uint8_t)(axisValue / 10.0F * 10.0F));
		display[1] = segments[(uint8_t)(axisValue / 1.0F)];
		axisValue -= (uint8_t)(axisValue / 1.0F);
		display[0] = segments[(uint8_t)(axisValue / 0.1F)];
		*dotSecondPlace = FALSE;
	}

	display[3] = segments[*selectedAxis];
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static selectedAxisType selectedAxis = Z;
	uint8_t * dotSecondPlace = FALSE;

	Dis7Seg_SetAxisValue(&selectedAxis, dotSecondPlace);

	Dis7seg_display(*dotSecondPlace);

	if (DetectFallingEdge(JOY_PORT, JOY_UP_PIN))
	{
		if (++selectedAxis > Z)
		{
			selectedAxis = X;
		}
	}

	if (DetectFallingEdge(JOY_PORT, JOY_DOWN_PIN))
	{
		if (--selectedAxis < X)
		{
			selectedAxis = Z;
		}
	}
}

uint8_t DetectFallingEdge(GPIO_TypeDef * port, uint16_t pin)
{
	static GPIO_TypeDef * prevPort;
	static uint16_t prevPin;
	static uint8_t buttonPressed = 0U;
	uint8_t buttonStatus = (~port->IDR & pin ? 1 : 0);
	uint8_t retVal_u8 = 0U;

	if (buttonStatus)
	{
		buttonPressed = TRUE;
		prevPort = port;
		prevPin = pin;
	}
	else if (!buttonStatus && buttonPressed && prevPort == port && prevPin == pin)
	{
		buttonPressed = FALSE;
		retVal_u8++;
	}

	return retVal_u8;
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
