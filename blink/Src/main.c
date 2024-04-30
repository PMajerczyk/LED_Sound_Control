#include <stdint.h>
#include "stm32f1xx.h"

#define LED_OFF GPIOC->BSRR |= GPIO_BSRR_BR13;    /* '0' on GPIOC13 */
#define LED_ON GPIOC->BSRR |= GPIO_BSRR_BS13;    /* '1' on GPIOC13 */
#define LED_TOGGLE GPIOC->ODR ^= GPIO_ODR_ODR13;    /* switch GPIOC13 */
#define DigitalSensor !(GPIOA->IDR & (1<<5))    /* PA5 state */

#define PWM_TIMER 5    /* PWM delay */
#define ADC_TIMER 50    /* ADC delay */

void OnBoardLED_Configuration(void);
void ADC_Configuration(void);
void ADC_Conversion(void);
uint32_t ADC_ReadData(void);
void TIM3_Configration(void);
void PWM_PinConfiguration(void);
void set_RGB(uint32_t R, uint32_t G, uint32_t B);
uint32_t GetSystemTick(void);

volatile uint32_t Tick;	   /* Tick for System Time */

int main(void)
{
	SysTick_Config(8000);    /* 1ms (8MHz/1000ms) */

	OnBoardLED_Configuration();
	ADC_Configuration();
	TIM3_Configration();
	PWM_PinConfiguration();

	uint32_t counter = 0;
	uint32_t state = 0;
	uint32_t ADCSample;    /* Analog value variable */
	uint32_t Timer_PWM = GetSystemTick();    /* Software clock variable for PWM */
	uint32_t Timer_ADC = GetSystemTick();    /* Software clock variable for ADC */
	while(1)
	{
		if((GetSystemTick() - Timer_PWM) > PWM_TIMER)
		{
			Timer_PWM = GetSystemTick();
			if (state == 0) set_RGB(counter, 0, 0);    /* PWM control */
			if (state == 1) set_RGB(0, counter, 0);
			if (state == 2) set_RGB(0, 0, counter);
			counter++;
			if (counter == 100){
				counter = 0;
				state++;
			}
			if(state == 3) state = 0;
		}

		if(DigitalSensor)    /* sensor detected sound */
			LED_ON
		else
			LED_OFF

		if((GetSystemTick() - Timer_ADC) > ADC_TIMER)
		{
			Timer_ADC = GetSystemTick();
			ADC_Conversion();
			ADCSample = ADC_ReadData();    /* Sensor Analog value */
		}
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

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;    /* ADC1 clock enable */

	GPIOA->CRL &= ~(GPIO_CRL_MODE0);    /* Input mode */

	GPIOA->CRL &= ~(GPIO_CRL_CNF0);    /* Analog mode */

    ADC1->CR2 |= ADC_CR2_ADON;    /* wake up the ADC from Power Down mode. */

    ADC1->CR2 |= ADC_CR2_CAL;    /* Start calibration */
    while (ADC1->CR2 & ADC_CR2_CAL);    /* Wait for calibration to finish */

    ADC1->SMPR2 |= ADC_SMPR2_SMP0_2;    /* channel 0 sample time 41.5 cycles */
}

void ADC_Conversion(void)
{
	ADC1->CR2 |= ADC_CR2_ADON;    /* A/D converter ON */
    ADC1->CR2 |= ADC_CR2_SWSTART;    /* Start conversion of regular channels */
}

uint32_t ADC_ReadData(void)
{
    while (!(ADC1->SR & ADC_SR_EOC));    /* Wait for end of conversion */
    return ADC1->DR;    /* Read conversion result */
}


void TIM3_Configration(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;    /* TIM3 clock enable */
	TIM3->PSC = (8 - 1);    /* TIM prescaler value */
	TIM3->ARR = (100 - 1);    /* auto-reload value */

	TIM3->CNT = 0;    /* counter reset */
	TIM3->CR1 |= TIM_CR1_CEN;    /* enable timer */

	// Channel 1
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;    /* "110" - PWM mode 1 */
	TIM3->CCR1 = (50 - 1);    /* PWM signal fill (50%) */
	TIM3->CCER |= TIM_CCER_CC1E;    /* output 1 enable */

	// Channel 2
	TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;    /* "110" - PWM mode 1 */
	TIM3->CCR2 = (50 - 1);    /* PWM signal fill (50%) */
	TIM3->CCER |= TIM_CCER_CC2E;    /* output 2 enable */

	// Channel 3
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;    /* "110" - PWM mode 1 */
	TIM3->CCR3 = (50 - 1);    /* PWM signal fill (50%) */
	TIM3->CCER |= TIM_CCER_CC3E;    /* output 3 enable */
}

void PWM_PinConfiguration(void)
{
	/* PA6 as PWM T3C1,  PA7 as PWM T3C2, PB0 as PWM T3C3 */

	//RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;    /* I/O port A clock enable */
	//The clock on port A has already been enabled in other function
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;    /* I/O port B clock enable */

	GPIOA->CRL |= GPIO_CRL_MODE6_0;    /* Output mode, max speed 10 MHz */
	GPIOA->CRL &= ~(GPIO_CRL_MODE6_1);
	GPIOA->CRL |= GPIO_CRL_MODE7_0;
	GPIOA->CRL &= ~(GPIO_CRL_MODE7_1);
	GPIOB->CRL |= GPIO_CRL_MODE0_0;
	GPIOB->CRL &= ~(GPIO_CRL_MODE0_1);

	GPIOA->CRL &= ~(GPIO_CRL_CNF6_0);    /* Alternate function output Push-pull */
	GPIOA->CRL |= GPIO_CRL_CNF6_1;
	GPIOA->CRL &= ~(GPIO_CRL_CNF7_0);
	GPIOA->CRL |= GPIO_CRL_CNF7_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF0_0);
	GPIOB->CRL |= GPIO_CRL_CNF0_1;
}

void set_RGB(uint32_t R, uint32_t G, uint32_t B)
{
	/* Because we control by low potential we must invert the value */

	R = 100-R;
	G = 100-G;
	B = 100-B;
	if(R > 100) R = 100;
	if(G > 100) G = 100;
	if(B > 100) B = 100;
	if(R <= 0) R = 1;
	if(G <= 0) G = 1;
	if(B <= 0) B = 1;

	TIM3->CCR1 = R-1;    /* PWM signals fill */
	TIM3->CCR2 = G-1;
	TIM3->CCR3 = B-1;
}

void SysTick_Handler(void)
{
	Tick++;    /* Increase system timer */
}

uint32_t GetSystemTick(void)
{
	return Tick;    /* Return current System Time */
}
