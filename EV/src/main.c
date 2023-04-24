/*
 * ECE 153B - Winter 2023
 *
 * Name(s): Jackson Newman Michelle Ly
 * Section: W
 * Lab: 3B
 */
 
#include <stdio.h> 
#include <stdlib.h>
#include <string.h>
 
#include "stm32l476xx.h"
#include "LED.h"
#include "EXTI.h"
#include "SysClock.h"
#include "SysTimer.h"
#include "UART.h"
#include "SPI.h"
#include "Display.h"
#include "RTC.h"

char strTime[12] = {0};
char strDate[12] = {0};

uint32_t volatile currentValue = 0;
uint32_t volatile lastValue = 0;
uint32_t volatile overflowCount = 0;
uint32_t volatile timeInterval = 0;
uint32_t volatile count = 0;
uint32_t volatile distance = 0;
volatile char rxByte;
// Initializes USARTx
// USART2: UART Communication with Termite
// USART1: Bluetooth Communication with Phone
void Init_USARTx(int x) {
	if(x == 1) {
		UART1_Init();
		UART1_GPIO_Init();
		USART_Init(USART1);
	} else if(x == 2) {
		UART2_Init();
		UART2_GPIO_Init();
		USART_Init(USART2);
	} else {
		// Do nothing...
	}
}

void Motor1_GPIO_Init(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	
	GPIOA->MODER &= ~GPIO_MODER_MODE0;
	GPIOA->MODER &= ~GPIO_MODER_MODE1;
	GPIOA->MODER|= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED0;

	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL0;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL1;
	GPIOA->AFR[0]|=GPIO_AFRL_AFSEL0_0 | GPIO_AFRL_AFSEL1_0;
	
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0;
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD1;

}

void Motor2_GPIO_Init(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	
	GPIOA->MODER &= ~GPIO_MODER_MODE6;
	GPIOA->MODER &= ~GPIO_MODER_MODE7;
	GPIOA->MODER|= GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED0;

	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL6;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL7;
	GPIOA->AFR[0]|=GPIO_AFRL_AFSEL6_1 | GPIO_AFRL_AFSEL7_1;
	
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD6;
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD7;
}

void  Timer2_Init(void){
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; //enable clock access tto tim2
	TIM2->CR1 &= ~TIM_CR1_DIR;

	TIM2->PSC = 0; //set prescaller to 0 (no divider)
	TIM2->ARR = 255; //set the maximum count value
	
	TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;
	TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
	TIM2->CCMR1 =(TIM_CCMR1_OC1M_1)|(TIM_CCMR1_OC1M_2)|(TIM_CCMR1_OC2M_1)|(TIM_CCMR1_OC2M_2);
	//configure the pins as PWM
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
	
	TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; //enbale channel1 and channel2
	
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	
	TIM2->CR1 = TIM_CR1_CEN; //enable timer
}

void Timer3_Init(void){
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN; //enable clock access tto tim2
	TIM3->CR1 &= ~TIM_CR1_DIR;

	TIM3->PSC = 0; //set prescaller to 0 (no divider)
	TIM3->ARR = 255; //set the maximum count value
	
	TIM3->CCMR1 &= ~TIM_CCMR1_OC1M;
	TIM3->CCMR1 &= ~TIM_CCMR1_OC2M;
	TIM3->CCMR1 =(TIM_CCMR1_OC1M_1)|(TIM_CCMR1_OC1M_2)|(TIM_CCMR1_OC2M_1)|(TIM_CCMR1_OC2M_2);
	//configure the pins as PWM
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
	
	TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; //enbale channel1 and channel2
	
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	
	TIM3->CR1 = TIM_CR1_CEN; //enable timer
}

void Motor1_Backward(void){
	TIM2->CCR2=255;
	TIM2->CCR1=0;
}

void Motor1_Forward(void){
	TIM2->CCR2=0;
	TIM2->CCR1=255;
}

void Motor2_Backward(void){
	TIM3->CCR2=255;
	TIM3->CCR1=0;
	
}

void Motor2_Forward(void){
	TIM3->CCR2=0;
	TIM3->CCR1=255;
}


int main(void) {	
	//RTC initialization
	RTC_Init();
	
	LED_Init();
	//SysClock initialization
	System_Clock_Init();
	//1ms Timer initialization
	SysTick_Init();
	
	//Display GPIO initialization
	Display_GPIO_Init();
	//SPI1 initialization
	SPI1_Init();
	//Display initialization
	Display_init(SPI1);
	
	//Clear the LCD screen
	LCD_clrScr();
	
	//Set RTC Clock Time and set the display background to black
	LCD_setColor(50, 180, 255);
	LCD_setColorBg(0, 0, 0);
	
	LCD_print("Bluetooth:", 10, 220);

	LCD_print("Enabled", 10, 240);

	LCD_print("Motors:", 10, 270);

	LCD_print("Enabled", 10, 290);
	
	Get_RTC_Calendar(strTime, strDate);
	
	LCD_print("Date:", 5, 10);
	LCD_print(strDate, 75, 10);
	
	//Initialize GPIO for motors and their respective PWM timers
	Motor1_GPIO_Init();
	Timer2_Init();
	Motor2_GPIO_Init();
	Timer3_Init();
	
	Init_USARTx(2);
	char rxByte;
	printf("W = Forward, S = Backward, A = Left, D = Right, R = stop\n");
	
	while(1) {
		
		scanf("%c",&rxByte);

		if (rxByte=='w'){
			printf("Going forward\n");
			
			Motor1_Forward();
			Motor2_Forward();
		}

		else if(rxByte=='s'){
			printf("Going backward\n");

			Motor1_Backward();
			Motor2_Backward();
		}
		else if(rxByte=='a'){
			printf("Turning left\n");

			Motor1_Forward();
			Motor2_Backward();
		}
		else if(rxByte=='d'){
			printf("Turning right\n");

			Motor2_Forward();
			Motor1_Backward();
		}
		else if(rxByte=='r'){
			printf("Stopping\n");

			TIM3->CCR1 = 0;
			TIM3->CCR2 = 0;
			TIM2->CCR1 = 0;
			TIM2->CCR2 = 0;
		}
		else{
			printf("Input is invalid\n");
		}
	}
}
