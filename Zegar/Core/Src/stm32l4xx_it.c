/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <joystick.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l4xx_it.h"
#include "stm32l4xx_hal.h"

#define DIG1_Pin GPIO_PIN_2
#define DIG1_GPIO_Port GPIOB
#define SEG_A_Pin GPIO_PIN_0
#define SEG_A_GPIO_Port GPIOG
#define SEG_B_Pin GPIO_PIN_1
#define SEG_B_GPIO_Port GPIOG
#define SEG_C_Pin GPIO_PIN_2
#define SEG_C_GPIO_Port GPIOG
#define SEG_D_Pin GPIO_PIN_3
#define SEG_D_GPIO_Port GPIOG
#define SEG_E_Pin GPIO_PIN_4
#define SEG_E_GPIO_Port GPIOG
#define SEG_F_Pin GPIO_PIN_5
#define SEG_F_GPIO_Port GPIOG
#define SEG_G_Pin GPIO_PIN_6
#define SEG_G_GPIO_Port GPIOG
#define SEG_DP_Pin GPIO_PIN_9
#define SEG_DP_GPIO_Port GPIOG
#define DIG2_Pin GPIO_PIN_3
#define DIG2_GPIO_Port GPIOB
#define DIG3_Pin GPIO_PIN_4
#define DIG3_GPIO_Port GPIOB
#define DIG4_Pin GPIO_PIN_5
#define DIG4_GPIO_Port GPIOB

#define SEG_MSK (0xffu | SEG_DP_Pin)
#define DIG_MSK (0xfu << 2)

#define TIME 500
#define	NDIGITS	4
uint16_t display[NDIGITS];

const uint8_t segments[] = {
		SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin,	// 0
		SEG_B_Pin | SEG_C_Pin,	// 1
		SEG_A_Pin | SEG_B_Pin | SEG_D_Pin | SEG_E_Pin | SEG_G_Pin,	// 2
		SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_G_Pin,	// 3
		SEG_B_Pin | SEG_C_Pin | SEG_F_Pin | SEG_G_Pin,	// 4
		SEG_A_Pin | SEG_C_Pin | SEG_D_Pin | SEG_F_Pin | SEG_G_Pin,	// 5
		SEG_A_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin | SEG_G_Pin,	// 6
		SEG_A_Pin | SEG_B_Pin | SEG_C_Pin,	// 7
		SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin | SEG_G_Pin,	// 8
		SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_F_Pin | SEG_G_Pin	// 9
	};

const uint8_t digits[] = {DIG4_Pin, DIG3_Pin, DIG2_Pin, DIG1_Pin};

/* USER CODE BEGIN 0 */



void dis7seg_init(void)
{
  // initialize GPIO
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIG1_Pin|DIG2_Pin|DIG3_Pin|DIG4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, SEG_A_Pin|SEG_B_Pin|SEG_C_Pin|SEG_D_Pin
                          |SEG_E_Pin|SEG_F_Pin|SEG_G_Pin|SEG_DP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIG1_Pin DIG2_Pin DIG3_Pin DIG4_Pin */
  GPIO_InitStruct.Pin = DIG1_Pin|DIG2_Pin|DIG3_Pin|DIG4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_A_Pin SEG_B_Pin SEG_C_Pin SEG_D_Pin
                           SEG_E_Pin SEG_F_Pin SEG_G_Pin SEG_DP_Pin */
  GPIO_InitStruct.Pin = SEG_A_Pin|SEG_B_Pin|SEG_C_Pin|SEG_D_Pin
                          |SEG_E_Pin|SEG_F_Pin|SEG_G_Pin|SEG_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}


uint8_t dis7seg_displayHHMM(uint16_t HH, uint16_t MM, uint16_t peak)
{
	//Display HH
	// 1 number
	display[2] = segments[HH - (HH/10) * 10];
	// 2 number
	display[3] = segments[HH / 10];

	display[2] |= SEG_DP_Pin;

	//Display MM
	// 1 number
	if (peak == 1)
	{
		display[0] = segments[MM - (MM/10) * 10] | SEG_DP_Pin;
	}
	if (peak == 0)
	{
		display[0] = segments[MM - (MM/10) * 10];
	}
	// 2 number
	display[1] = segments[MM / 10];

	return 0;
}

uint8_t dis7seg_displayMMSS(uint16_t MM, uint16_t SS)
{
	//Display MM
	// 1 number
	display[2] = segments[MM - (MM/10) * 10]  | SEG_DP_Pin;
	// 2 number
	display[3] = segments[MM / 10];

	//Display SS
	// 1 number
	display[0] = segments[SS - (SS/10) * 10];
	// 2 number
	display[1] = segments[SS / 10];

	return 0;
}


static inline void dis7seg_mpx(void)
{





	// Czyszczenie cyfry


}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

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
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
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
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
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
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
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
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
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
	dis7seg_init();
	/* USER CODE BEGIN SysTick_IRQn 1 */
	static uint16_t freq = 1;
	static uint16_t mode = 0;
	static uint16_t interrupt = 0;
	static uint16_t hh=23;
	static uint16_t mm=59;
	static uint16_t ss=30;
	static uint16_t peakSS = 0;
	static uint16_t x=0;
	static uint8_t dig = 0;
	static uint16_t podtrzymanie = 0;

	if(++interrupt == 1000)
	{
		interrupt = 0;
		if (JOY_OK_DOWN && mode == 1)
			{
				mode = 0;
			}
			else if (JOY_OK_DOWN && mode == 0)
			{
				mode = 1;
			}

		if (mode == 0)
		{
			dis7seg_displayMMSS(mm, ss);
		}
		if (mode == 1)
		{
			dis7seg_displayHHMM(hh, mm, peakSS);
		}

		ss++;
		peakSS = ss % 2;

		if (ss == 60)
		{
			ss = 0;
			mm++;
		}
		if (mm == 60)
		{
			mm = 0;
			hh++;
		}
		if (hh == 24)
		{
			hh = 0;
		}
	}
	else if(++x == 1)
	{
		if (freq == 1)
		{
			x=0;
			HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG_MSK, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_MSK, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEG_A_GPIO_Port, display[dig], GPIO_PIN_SET);
			HAL_GPIO_WritePin(DIG1_GPIO_Port, digits[dig], GPIO_PIN_SET);
			dig = (dig + 1) & 3;
		}
		else
		{
			x = 0;
			if (++podtrzymanie <= freq)
			{
				HAL_GPIO_WritePin(SEG_A_GPIO_Port, display[dig], GPIO_PIN_SET);
				HAL_GPIO_WritePin(DIG1_GPIO_Port, digits[dig], GPIO_PIN_SET);
			}
			else if (freq < ++podtrzymanie <= freq*2)
			{
				if (podtrzymanie == freq*2)
				{
					podtrzymanie = 0;
					dig = (dig + 1) & 3;
				}
				HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG_MSK, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_MSK, GPIO_PIN_RESET);
			}
		}
	}
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
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
