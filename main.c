#include "inc/stm32f030x6.h"

#include "peripherals.h"

#include <stdint.h>
#include <stdbool.h>

#define LED_PIN 4

#define DATA_1_HIGH	38
#define DATA_1_LOW  22
#define DATA_1_PERIOD (DATA_1_HIGH + DATA_1_LOW)
#define DATA_0_HIGH 19
#define DATA_0_LOW  41
#define DATA_0_PERIOD (DATA_0_HIGH + DATA_0_LOW)

#define NUM_LEDS 3

void led_show(TIM_TypeDef *TIMx)
{
	TIMx->CCER |= TIM_CCER_CC4E;
	TIMx->CR1 |= TIM_CR1_CEN;
}

uint32_t bit_count = 0;
uint8_t led_data[NUM_LEDS * 3];
uint8_t led_bit_mask = 0B01000000;
uint16_t led_pos = 0;

uint8_t prev_bit = 1;

int main(void)
{
	/* Setup flash wait cycles */
	FLASH->ACR &= ~(0x00000017);
	FLASH->ACR |= (FLASH_ACR_LATENCY | FLASH_ACR_PRFTBE);

	/* Initialise system clock */
	clock_setup(false, true, PLL_MULT_X12);

	/* Enable the clock for timer 3 */
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	NVIC_SetPriority(TIM3_IRQn, 0x01);
	NVIC_EnableIRQ(TIM3_IRQn);

	/* Enable the clock for DMA channel 3 */
//	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
//	/* Enable the DMA transfer on timer 3 */
//	TIM3->DIER |= TIM_DIER_CC4DE;
//	/* Configure the peripheral data register address */
//	/* Configure the memory address */
//	/* Configure the number of memory transfers to occur */
//	/* Configure the increment, size and interrupts */
//	/* Enable DMA channel 3 */
//	DMA1_Channel3->CCR |= DMA_CCR_EN;

	/* Enable Port A GPIO clock */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	/* Enable Port B GPIO clock */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	/* Set PA4 to output and set high */
	gpio_init(PORTA, LED_PIN, GPIO_OUTPUT, 0);
	/* Finally set the pin high */
	GPIOA->ODR &= (1 << LED_PIN);

	/* Enable PB1 with alternate functionality */
	gpio_init(PORTB, PIN_1, GPIO_ALT_MODE, GPIO_AF1);

	/* Initialise LED data array */
	for(uint32_t i = 0; i < NUM_LEDS; i++)
	{
		led_data[3 * i] = 255;
		led_data[(3 * i) + 1] = 0;
		led_data[(3 * i) + 2] = 128;
	}

	/* Initialise the timer */
	init_timer(TIM3);
	//start_timer(TIM3, 48000, 1000);
	setup_timer_capture_compare(TIM3, TIM_CHAN_4, DATA_1_PERIOD, DATA_1_HIGH, 0);
	led_show(TIM3);

	/* Test stuff */
//	gpio_init(PORTB, PIN_1, GPIO_OUTPUT, 0);
//	GPIOB->OSPEEDR &= GPIO_OSPEEDR_OSPEEDR1_Msk;
//	GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR1_0 | GPIO_OSPEEDR_OSPEEDR1_1);
//	while (led_pos < (NUM_LEDS * 3))
//	{
//		uint8_t next_bit = led_data[led_pos] & led_bit_mask;
//
//		if (next_bit == 1)
//		{
//			GPIOB->BSRR = GPIO_BSRR_BS_1;
//			__asm__("NOP");
//			GPIOB->BSRR = GPIO_BSRR_BR_1;
//			__asm__("NOP");
//		}
//		else
//		{
//			GPIOB->BSRR = GPIO_BSRR_BS_1;
//			__asm__("NOP");
//			GPIOB->BSRR = GPIO_BSRR_BR_1;
//			__asm__("NOP");
//		}
//
//		led_bit_mask = led_bit_mask >> 1;
//		if (led_bit_mask == 1)
//		{
//			led_pos += 1;
//			led_bit_mask = 0B10000000;
//		}
//	}
	/* End of test stuff */
	
	/* Loop forever */
	while(1)
	{
//		GPIOB->BSRR = GPIO_BSRR_BS_1;
//		GPIOB->BSRR = GPIO_BSRR_BR_1;
	};
}

void TIM3_IRQHandler(void)
{

	/* Check the cause of the interrupt */
	if (TIM3->SR & TIM_SR_UIF)
	{
		/* Clear the interrupt flag */
		TIM3->SR &= ~(TIM_SR_UIF);
	}

	//uint8_t next_bit = led_data[led_pos] & led_bit_mask;
	if (prev_bit == 1)
	{
		TIM3->ARR = DATA_0_PERIOD;
		TIM3->CCR4 = DATA_0_HIGH;
		prev_bit = 0;
	}
	else
	{
		TIM3->ARR = DATA_1_PERIOD;
		TIM3->CCR4 = DATA_1_HIGH;
		prev_bit = 1;
	}

	//bit_count += bit_count;	
	//if (bit_count > (NUM_LEDS * 8 * 3))
	//{
	//	TIM3->CR1 &= ~TIM_CR1_CEN;
	//	bit_count = 0;
	//}
}
