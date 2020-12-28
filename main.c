#include "inc/stm32f030x6.h"

#include "peripherals.h"

#include <stdint.h>
#include <stdbool.h>

#define LED_PIN 0

#define DATA_1_HIGH	38
#define DATA_1_LOW  22
#define DATA_1_PERIOD (DATA_1_HIGH + DATA_1_LOW)
#define DATA_0_HIGH 19
#define DATA_0_LOW  41
#define DATA_0_PERIOD (DATA_0_HIGH + DATA_0_LOW)

#define NUM_LEDS 12

/* Set the number of leds of data you want to buffer */
/* NB: This should always be an even number! */
#define DMA_LED_BUFF 6
#define DMA_BUFF_SIZE (DMA_LED_BUFF * 8 * 3)
#define DMA_HALF_SIZE (DMA_BUFF_SIZE/2)
#define DMA_LOWER_HALF_OFFSET 0
#define DMA_UPPER_HALF_OFFSET DMA_HALF_SIZE
#define DMA_PADDING 10 

/* This buffer is used to store capture/compare values for the timer */
uint16_t CCR_buffer[DMA_BUFF_SIZE];


uint8_t led_data[NUM_LEDS * 3];
uint8_t led_bit_mask = 0B10000000;
uint16_t led_pos = 0;

void led_fill_dma_buffer(uint16_t offset, uint16_t length)
{
	for (uint16_t i = offset; i < length; i++)
	{
		/* Add padding if the array position exceeds the number of LEDs */
		if (led_pos >= (NUM_LEDS * 3))
		{
			CCR_buffer[i] = 0;
		}
		else
		{
			if (led_data[led_pos] & led_bit_mask)
			{
				CCR_buffer[i] = DATA_1_HIGH;
			}
			else
			{
				CCR_buffer[i] = DATA_0_HIGH;
			}
		}
		if (led_bit_mask == 1)
		{
			led_bit_mask = 0B10000000;
			led_pos = led_pos + 1;
			/* TODO: If LED pos reaches end + 1 here, trigger shutdown event */
			if (led_pos >= ((NUM_LEDS * 3) + DMA_PADDING))
			{
				/* Disable timer & DMA */
				TIM3->CR1 &= ~(TIM_CR1_CEN);
//				DMA1_Channel3->CCR &= ~(DMA_CCR_EN);
//				TIM3->CCER &= ~(TIM_CCER_CC4E);
			}
		}
		else
		{
			led_bit_mask = led_bit_mask >> 1;
		}
	}
}


void led_show(TIM_TypeDef *TIMx)
{
	/* Setup the timer for capture/compare mode */
	setup_timer_capture_compare(TIM3, TIM_CHAN_4, DATA_1_PERIOD, 0, 0, false,true);

//	/* Set the capture/compare register with the value for the first bit */
//	if ((led_data[led_pos] & led_bit_mask) == 1)
//	{
//		TIMx->CCR4 = DATA_1_HIGH;
//	}
//	else
//	{
//		TIMx->CCR4 = DATA_0_HIGH;
//	}
//
//	led_bit_mask = led_bit_mask >> 1;

	/* Fill full buffer before starting */
	led_fill_dma_buffer(DMA_LOWER_HALF_OFFSET, DMA_BUFF_SIZE);

	TIMx->CCER |= TIM_CCER_CC4E;
	TIMx->CR1 |= TIM_CR1_CEN;
}

int main(void)
{
	/* Setup flash wait cycles */
	FLASH->ACR &= ~(0x00000017);
	FLASH->ACR |= (FLASH_ACR_LATENCY | FLASH_ACR_PRFTBE);

	/* Initialise system clock */
	clock_setup(false, true, PLL_MULT_X12);

	/* Enable the clock for timer 3 */
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	//NVIC_SetPriority(TIM3_IRQn, 0x01);
	//NVIC_EnableIRQ(TIM3_IRQn);

	/* Initialise the timer */
	init_timer(TIM3);
	//start_timer(TIM3, 48000, 1000);

	/* Enable the clock for DMA channel 3 */
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	/* First make sure DMA is disabled */
	DMA1_Channel3->CCR &= ~(DMA_CCR_EN);
	/* Enable the DMA transfer on timer 3 */
	TIM3->DIER |= TIM_DIER_CC4DE;
	/* Configure the peripheral data register address */
	DMA1_Channel3->CPAR = (uint32_t)&TIM3->CCR4;
	/* Configure the memory address */
	DMA1_Channel3->CMAR = (uint32_t)&CCR_buffer[0];
	/* Setup the DMA configuration register */
	/* Set the memory and peripheral size to 16 bit (01) in respective bits */
	/* Set priority to very high (11) in the PL bits */
	/* Set the memory increment to true, whilst the peripheral increment is false */
	/* Set the DMA to circular mode */
	/* Set the direction to transfer from memory to peripheral */
	/* Enable the half transfer and transfer complete interrupts */
	DMA1_Channel3->CCR = (uint32_t)0;
	DMA1_Channel3->CCR |= (DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 |
							DMA_CCR_PL_0 | DMA_CCR_PL_1 | DMA_CCR_MINC |
							DMA_CCR_CIRC | DMA_CCR_DIR |
							DMA_CCR_HTIE | DMA_CCR_TCIE);
	/* Set the size of the DMA transfer to the buffer size */
	DMA1_Channel3->CNDTR = DMA_BUFF_SIZE;
	/* Enable DMA interrupts for channel 3 in the NVIC */
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	/* Enable DMA channel 3 */
	DMA1_Channel3->CCR |= DMA_CCR_EN;

	/* Enable Port A GPIO clock */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	/* Enable Port B GPIO clock */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	/* Set PA4 to output and set high */
	gpio_init(PORTA, LED_PIN, GPIO_OUTPUT, 0);
	/* Finally set the pin high */
	GPIOA->ODR |= (1 << LED_PIN);

	/* Enable PB1 with alternate functionality */
	gpio_init(PORTB, PIN_1, GPIO_ALT_MODE, GPIO_AF1);

	/* Initialise LED data array */
	for(uint32_t i = 0; i < NUM_LEDS; i++)
	{
//		led_data[3 * i]       = 0xaa;
//		led_data[(3 * i) + 1] = 0xaa;
//		led_data[(3 * i) + 2] = 0xaa;
		led_data[3 * i]       = 10;
		led_data[(3 * i) + 1] = 10;
		led_data[(3 * i) + 2] = 10;
//		led_data[3 * i]       = 255;
//		led_data[(3 * i) + 1] = 255;
//		led_data[(3 * i) + 2] = 255;
	}
	
	/* Call led setup function */
	led_show(TIM3);

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
}

void DMA1_Channel2_3_IRQHandler(void)
{
	/* Half way through buffer interrupt */
	if (DMA1->ISR | DMA_ISR_HTIF3)
	{
		/* Fill the lower half of the buffer */
		led_fill_dma_buffer(DMA_LOWER_HALF_OFFSET, DMA_HALF_SIZE);

		/* Clear the interrupt flag */
		DMA1->IFCR = DMA_IFCR_CHTIF3;
	}
	/* End of buffer interrupt */
	else if (DMA1->ISR | DMA_ISR_TCIF3)
	{
		/* Fill the upper half of the buffer */
		led_fill_dma_buffer(DMA_UPPER_HALF_OFFSET, DMA_HALF_SIZE);

		/* Clear the interrupt flag */
		DMA1->IFCR = DMA_IFCR_CTCIF3;
	}
}
