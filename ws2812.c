#include "inc/stm32f030x6.h"

#include "ws2812.h"
#include "peripherals.h"

#include <stdint.h>
#include <stdbool.h>

#define DATA_1_HIGH	38
#define DATA_1_LOW  22
#define DATA_1_PERIOD (DATA_1_HIGH + DATA_1_LOW)
#define DATA_0_HIGH 19
#define DATA_0_LOW  41
#define DATA_0_PERIOD (DATA_0_HIGH + DATA_0_LOW)

/* This buffer is used to store capture/compare values for the timer */
uint16_t CCR_buffer[DMA_BUFF_SIZE];

void led_fill_dma_buffer(rgb_led_t *leds, uint16_t offset, uint16_t length)
{
	for (uint16_t i = offset; i < (length + offset); i++)
	{
		/* Add padding if the array position exceeds the number of LEDs */
		if (leds->arr_pos >= (leds->num_leds * 3))
		{
			CCR_buffer[i] = 0;
		}
		else
		{
			if (leds->data[leds->arr_pos] & leds->bit_mask)
			{
				CCR_buffer[i] = DATA_1_HIGH;
			}
			else
			{
				CCR_buffer[i] = DATA_0_HIGH;
			}
		}
		if (leds->bit_mask == 1)
		{
			leds->bit_mask = 0B10000000;
			leds->arr_pos = leds->arr_pos + 1;
		}
		else
		{
			leds->bit_mask = leds->bit_mask >> 1;
		}
	}
}

void dma_setup(void)
{
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
	/* Set the priority to high for DMA */
	NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0);
	/* Enable DMA interrupts for channel 3 in the NVIC */
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	/* Enable DMA channel 3 */
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

void led_write_all(rgb_led_t *leds, uint8_t red, uint8_t green, uint8_t blue)
{
	for (uint16_t i = 0; i < leds->num_leds; i++)
	{
		leds->data[(3 * i)]     = green;
		leds->data[(3 * i) + 1] = red;
		leds->data[(3 * i) + 2] = blue;
	}
}

void led_init(void)
{
	/* Setup the timer for capture/compare mode */
	setup_timer_capture_compare(TIM3, TIM_CHAN_4, DATA_1_PERIOD, 0, 0, false,true);
}

void led_show(rgb_led_t *leds, TIM_TypeDef *TIMx)
{
	/* Reset the array position and bit mask before starting */
	leds->bit_mask = 0B10000000;
	leds->arr_pos = 0;

	/* Reset the DMA */
	DMA1_Channel3->CCR &= ~(DMA_CCR_EN);
	DMA1_Channel3->CCR |= DMA_CCR_EN;

	/* Fill full buffer before starting */
	led_fill_dma_buffer(leds, DMA_LOWER_HALF_OFFSET, DMA_BUFF_SIZE);

	dma_setup();

	TIMx->CCER |= TIM_CCER_CC4E;
	TIMx->CR1 |= TIM_CR1_CEN;
}
