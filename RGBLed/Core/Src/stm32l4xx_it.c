/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kamami_l496_joy.h"
#include "kamami_l496_led_rgb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define JUMP 25
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  /* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	static uint8_t interrupt = 0;
	static uint8_t _red, _green, _blue= 0;
	static uint8_t _activeColor = 1;


	if(++interrupt == 50) // we've counted 50 interrupts
	{
		interrupt = 0; // reset the interrupt counter
		if (JOY_UP_DOWN)
		{
			if (_activeColor == 1)
			{
				if (_red < 255) _red++;
			}
			if (_activeColor == 2)
			{
				if (_green < 255) _green++;
			}
			if (_activeColor == 3)
			{
				if (_blue < 255) _blue++;
			}
		}
		if (JOY_DOWN_DOWN)
		{
			if (_activeColor == 1)
			{
				if (_red > 0) _red--;
			}
			if (_activeColor == 2)
			{
				if (_green > 0) _green--;
			}
			if (_activeColor == 3)
			{
				if (_blue > 0) _blue--;
			}
		}
		if (JOY_RIGHT_DOWN)
		{
			_activeColor++;
			if(_activeColor == 4) _activeColor = 1;
		}
		if (JOY_LEFT_DOWN)
		{
			_activeColor--;
			if(_activeColor == 0) _activeColor = 3;
		}
		if (JOY_OK_DOWN) _red = _green = _blue = 0;
		led_rgb_set_intensity(_red, _green, _blue);
	}

//	if(++interrupt == 50)
//	{
//		interrupt = 0;
//		if (JOY_RIGHT_DOWN)
//		{
//			_activeColor++;
//			if(_activeColor >= 3) _activeColor = 0;
//		}
//		if (JOY_LEFT_DOWN)
//		{
//			_activeColor--;
//			if(_activeColor <= 0) _activeColor = 2;
//		}
//
//		if (JOY_UP_DOWN)
//		{
//			if (_activeColor == 0)
//			{
//				if (_red < 255) _red++;
//			}
//			if (_activeColor == 1)
//			{
//				if (_green < 255) _green++;
//			}
//			if (_activeColor == 2)
//			{
//				if (_blue < 255) _blue++;
//			}
//		}
//		if (JOY_UP_DOWN)
//		{
//			if (_activeColor == 0)
//			{
//				if (_red > 0) _red--;
//			}
//			if (_activeColor == 1)
//			{
//				if (_green > 0) _green--;
//			}
//			if (_activeColor == 2)
//			{
//				if (_blue > 0) _blue--;
//			}
//		}
//		if (JOY_OK_DOWN) _red = _green = _blue = 0;
//
//		led_rgb_set_intensity(_red, _green, _blue);
//	}

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
