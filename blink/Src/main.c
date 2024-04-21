#include <stdint.h>
#include "stm32f1xx.h"

#define DigitalSensor (GPIOA->IDR & (1<<6)) /* PA6 state */

void configureLED(void);
void delay_ms(uint32_t ms);
void configureSensor(void);

volatile uint32_t Tick;

int main(void)
{
	configureLED();
	configureSensor();

	SysTick_Config(8000); // 1ms (8MHz/1000ms)

	while(1)
	{
		if(DigitalSensor) /* if '1' on GPIOA6 */
		{
			GPIOC->BSRR |= GPIO_BSRR_BR13; /* '0' on GPIOC13 */
			delay_ms(100);

			GPIOC->BSRR |= GPIO_BSRR_BS13; /* '1' on GPIOC13 */
			delay_ms(100);
		}
	}
}



void configureLED(void)
{
	/* LED is connected to PC13 on the APB2 bus */

	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; /* I/O port C clock enable */

	GPIOC->CRH |= GPIO_CRH_MODE13_0; /* Output mode, max speed 10 MHz */
	GPIOC->CRH &= ~(GPIO_CRH_MODE13_1);

	GPIOC->CRH &= ~(GPIO_CRH_CNF13); /* General purpose output push-pull */
}

void configureSensor(void)
{
	/* Sensor's digital output is connected to PA6 on the APB2 bus */

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; /* I/O port A clock enable */

	/* Input mode and Floating input are reset state */
}

void SysTick_Handler(void)
{
	Tick++;
}

void delay_ms(uint32_t ms)
{
	uint32_t StartTime = Tick;

	while(Tick < (StartTime + ms))
	{
		//do nothing
	}
}
