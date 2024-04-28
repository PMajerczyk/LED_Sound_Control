#include <stdint.h>
#include "stm32f1xx.h"

#define LED_OFF GPIOC->BSRR |= GPIO_BSRR_BR13;    /* '0' on GPIOC13 */
#define LED_ON GPIOC->BSRR |= GPIO_BSRR_BS13;    /* '1' on GPIOC13 */
#define LED_TOGGLE GPIOC->ODR ^= GPIO_ODR_ODR13;    /* switch GPIOC13 */
#define DigitalSensor (GPIOA->IDR & (1<<5))    /* PA5 state */

#define PWM_TIMER 50    /* PWM delay */
#define LED_TIMER 500    /* LED delay */

void OnBoardLED_Configuration(void);
void ADC_Configuration(void);
void ADC_Conversion(void);
uint32_t ADC_ReadData(void);
void TIM3_Configration(void);
void PWM_PinConfiguration(void);
uint32_t GetSystemTick(void);

volatile uint32_t Tick;	   /* Tick for System Time */

int main(void)
{
	SysTick_Config(8000);    /* 1ms (8MHz/1000ms) */

	OnBoardLED_Configuration();
	ADC_Configuration();
	TIM3_Configration();
	PWM_PinConfiguration();

	uint32_t ADCSample;    /* Analog value variable */
	uint32_t Timer_PWM = GetSystemTick();    /* Software clock variable for PWM */
	uint32_t Timer_LED = GetSystemTick();    /* Software clock variable for PWM */
	while(1)
	{
		if((GetSystemTick() - Timer_PWM) > PWM_TIMER)
		{
			Timer_PWM = GetSystemTick();
			if(TIM3->CCR1 < (TIM3->ARR-1))    /* PWM control */
				TIM3->CCR1 += 1;
			else
				TIM3->CCR1 = 1;
		}

		if(DigitalSensor) /* if '1' on GPIOA5 */
		{
			// sensor detected sound
		}

		if((GetSystemTick() - Timer_LED) > LED_TIMER)
		{
			Timer_LED = GetSystemTick();
			LED_TOGGLE;     /* LED blink */
		}

		ADC_Conversion();
		ADCSample = ADC_ReadData();    /* Sensor Analog value */
	}
}



void OnBoardLED_Configuration(void)
{
	/* LED is connected to PC13 on the APB2 bus */

	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;    /* I/O port C clock enable */

	GPIOC->CRH |= GPIO_CRH_MODE13_0;    /* Output mode, max speed 10 MHz */
	GPIOC->CRH &= ~(GPIO_CRH_MODE13_1);

	GPIOC->CRH &= ~(GPIO_CRH_CNF13);    /* General purpose output push-pull */
}

void configureSensor(void)
{
	/* Sensor's digital output is connected to PA5 on the APB2 bus */

	//RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;    /* I/O port A clock enable */
	//The clock on port A has already been enabled in other function

	/* Input mode and Floating input are reset state */
}

void ADC_Configuration(void)
{
	/* Sensor's analog output is connected to PA0 on the APB2 bus */

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;    /* I/O port A clock enable */

	GPIOC->CRL &= ~(GPIO_CRL_MODE0);    /* Input mode */

	GPIOC->CRL &= ~(GPIO_CRL_CNF0);    /* Analog mode */
	//
	ADC1->CR2 |= ADC_CR2_ADON;    /* A/D converter ON */

	ADC1->SMPR1 |= ADC_SMPR1_SMP1_2;    /* sample time 41.5 cycles */

	ADC1->CR1 = ADC_CR2_CAL;    /* enable calibration */
	while(ADC1->CR1 & ADC_CR2_CAL);
}

void ADC_Conversion(void)
{
	ADC1->CR2 |= ADC_CR2_SWSTART;   /* Start conversion of regular channels */
	while(!(ADC1->SR & ADC_SR_EOC));    /* 1 - end of conversion  */
}

uint32_t ADC_ReadData(void)
{
	return ADC1->DR;    /* return data */
}

void TIM3_Configration(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;    /* TIM3 clock enable */
	TIM3->PSC = (8 - 1);    /* TIM prescaler value */
	TIM3->ARR = (100 - 1);    /* auto-reload value */

	TIM3->CNT = 0;    /* counter reset */
	TIM3->CR1 |= TIM_CR1_CEN;    /* enable timer */

	TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;    /* "110" - PWM mode 1 */
	TIM3->CCR1 = (50 - 1);    /* PWM signal fill (50%) */
	TIM3->CCER |= TIM_CCER_CC1E;    /* output 1 enable */
}

void PWM_PinConfiguration(void)
{
	/* PA6 as PWM T3C1 */
	//RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;    /* I/O port A clock enable */
	//The clock on port A has already been enabled in other function

	GPIOA->CRL |= GPIO_CRL_MODE6_0;    /* Output mode, max speed 10 MHz */
	GPIOA->CRL &= ~(GPIO_CRL_MODE6_1);

	GPIOA->CRL &= ~(GPIO_CRL_CNF6_0);    /* Alternate function output Push-pull */
	GPIOA->CRL |= GPIO_CRL_CNF6_1;
}

void SysTick_Handler(void)
{
	Tick++;    /* Increase system timer */
}

uint32_t GetSystemTick(void)
{
	return Tick;    /* Return current System Time */
}
