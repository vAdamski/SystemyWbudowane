// Functions for easy use of joystick on Kamami KAMeLeon STM32L496 board.
// jk 11'2017

#ifndef	_KAMAMI_L496_JOY_H
#define	_KAMAMI_L496_JOY_H
#include <stdint.h>

// Bitmasks for joystick switches.
#define JOY_RIGHT 	GPIO_PIN_0
#define JOY_LEFT 	GPIO_PIN_1
#define JOY_DOWN 	GPIO_PIN_2
#define JOY_UP 		GPIO_PIN_3
#define JOY_OK 		GPIO_PIN_15
#define	JOY_ALL_MSK	(JOY_DOWN_Pin | JOY_UP_Pin | JOY_OK_Pin | JOY_RIGHT_Pin | JOY_LEFT_Pin)

#define	JOY_GPIO	GPIOE
/* The macros below can be used to check if the respective switch is pressed.
 * They have the value of 1 as long as the switch is down. */ 
#define JOY_RIGHT_DOWN	(~JOY_GPIO->IDR & JOY_RIGHT ? 1 : 0)
#define JOY_LEFT_DOWN 	(~JOY_GPIO->IDR & JOY_LEFT ? 1 : 0)
#define JOY_DOWN_DOWN 	(~JOY_GPIO->IDR & JOY_DOWN ? 1 : 0)
#define JOY_UP_DOWN 	(~JOY_GPIO->IDR & JOY_UP ? 1 : 0)	
#define JOY_OK_DOWN 	(~JOY_GPIO->IDR & JOY_OK ? 1 : 0)

/* Initialize joystick. 
 * The function should be called before using any other joystick function. 
 * It initializes GPIO lines. */
void joy_init(void);

/* This function should be called from SysTick_Handler or from the interrupt service routine 
 * of a timer running at constant frequency. If the SysTick is used and it's frequency is 
 * different from 1000 Hz, the symbol SYSTICK_FREQ should be defined accordingly. If another
 * timer is used and it's frequency is different from 1000 Hz, the symbol JOYTIM_FREQ should 
 * be defined accordingly.
 * The function takes care of debouncing. 
 * @retval The debounced state of the joystick switches. 
 */
uint16_t joy_update(void);

/* Get the debounced state of the joystick switches. 
 * The bitmasks defined above should be used to test the respective switches. 
 * If the switch is pressed, the respective bit is 1, oterwise it is 0. 
 * @retval The debounced state of the joystick switches. */
uint16_t joy_get_state(void);

/* Get the raw (not debounced) state of the joystick switches. 
 * The bitmasks defined above should be used to test the respective switches. 
 * If the switch is pressed, the respective bit is 1, oterwise it is 0. 
 * @retval The raw (not debounced) state of the joystick switches. */
uint16_t joy_get_raw_state(void);

#endif
