// Functions for easy use of 4-digit 7-segment display on Kamami KAMeLeon STM32L496 board.
// jk 11'2017

#ifndef	_KAMAMI_L496_7SEG_H
#define	_KAMAMI_L496_7SEG_H
#include <stdint.h>

/* Initialize 7-segment display. 
 * The function should be called before using any other 7-segment display function. 
 * It initializes GPIO lines and timer used for multiplexing (TIM2 is used for 7-segment display). */
void dis7seg_init(void);

/* Display numeric value on 7-segment display. 
 * The value should not be greater than 9999 as the display has 4 digits. 
 * @param value: Value to be displayed. 
 * @retval 0 if everything went ok, 1 if incorrect parameters were given.  */ 
uint8_t dis7seg_display(uint16_t value);

/* Display numeric fixed-point value on 7-segment display. 
 * Allows specifying where the fraction point should be placed in the displayed value.
 * The value should not be greater than 9999 as the display has 4 digits. 
 * The number of fraction digits should not be grater than 3.
 * @param value: Value to be displayed specyfying all the digits including fraction digits. Should be in range 0-9999.
 * @param fraction_digits: Number of fraction digits (i.e. digits after decimal point). Should be in range 0-3.
 E.g. giving 456 as value and 2 as fraction_digits will result in "4.56" being displayed.
 * @retval 0 if everything went ok, 1 if incorrect parameters were given.  */ 
uint8_t dis7seg_display_fixed_point(uint16_t value, uint8_t fraction_digits);

#endif
