// Functions for easy use of RGB LED on Kamami KAMeLeon STM32L496 board.
// jk 11'2017

#ifndef	_KAMAMI_L496_LED_RGB_H
#define	_KAMAMI_L496_LED_RGB_H
#include <stdint.h>

/* Initialize RGB LED. 
 * The function should be called before using any other RGB LED function. 
 * It initializes GPIO lines and timer TIM4 used for PWM signal generation. */
void led_rgb_init(void);

/* Set intensity of individual (red, green and blue) LEDs making up RGB LED. 
 * 0 intensity means the LED is off, 255 intensity means the LED is fully lit.
 * @param r_intensity: intensity of red LED
 * @param g_intensity: intensity of green LED
 * @param b_intensity: intensity of blue LED */
void led_rgb_set_intensity(uint8_t r_intensity, uint8_t g_intensity, uint8_t b_intensity);

#endif
