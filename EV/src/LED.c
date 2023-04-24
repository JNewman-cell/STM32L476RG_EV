/*
 * ECE 153B - Winter 2021
 *
 * Name(s):
 * Section:
 * Lab: 2A
 */

#include "LED.h"


void LED_Init(void) {
	// Enable GPIO Clocks
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	
	// Initialize Green LED
	GPIOA->MODER &= ~GPIO_MODER_MODE5_1;
}

void Green_LED_Off(void) {
	GPIOA->ODR &= ~GPIO_ODR_OD5;
}

void Green_LED_On(void) {
	GPIOA->ODR |= GPIO_ODR_OD5;
}

void Green_LED_Toggle(void) {
	if (GPIOA->ODR & GPIO_ODR_OD5){
		Green_LED_Off();
	} else {
		Green_LED_On();
	}
}
