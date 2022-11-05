// Functions for easy use of joystick on Kamami KAMeLeon STM32L496 board.
// jk 11'2017
// The initialization code was generated in STM32CubeMX.

#include "kamami_l496_joy.h"
#include "stm32l4xx_hal.h"

#ifndef SYSTICK_FREQ
#define SYSTICK_FREQ	1000
#endif
#ifndef JOYTIM_FREQ
#define JOYTIM_FREQ		SYSTICK_FREQ	
#endif

#define JOY_RIGHT_Pin GPIO_PIN_0
#define JOY_RIGHT_GPIO_Port GPIOE
#define JOY_LEFT_Pin GPIO_PIN_1
#define JOY_LEFT_GPIO_Port GPIOE
#define JOY_DOWN_Pin GPIO_PIN_2
#define JOY_DOWN_GPIO_Port GPIOE
#define JOY_UP_Pin GPIO_PIN_3
#define JOY_UP_GPIO_Port GPIOE
#define JOY_OK_Pin GPIO_PIN_15
#define JOY_OK_GPIO_Port GPIOE

void joy_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pins : JOY_DOWN_Pin JOY_UP_Pin JOY_OK_Pin JOY_RIGHT_Pin 
                           JOY_LEFT_Pin */
  GPIO_InitStruct.Pin = JOY_DOWN_Pin|JOY_UP_Pin|JOY_OK_Pin|JOY_RIGHT_Pin 
                          |JOY_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

uint16_t joy_state = 0;

uint16_t joy_update(void)
{
	uint16_t static div = SYSTICK_FREQ / 33; // every 30 ms
	
	if(--div == 0)
	{
		div = SYSTICK_FREQ / 33; 
		joy_state = ~JOY_GPIO->IDR & JOY_ALL_MSK;
	}
	return joy_state;
}

uint16_t joy_get_state(void)
{
	return joy_state;
}

uint16_t joy_get_raw_state(void)
{
	return ~JOY_GPIO->IDR & JOY_ALL_MSK;
}
