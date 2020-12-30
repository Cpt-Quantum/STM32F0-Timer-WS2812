#ifndef WS2812_H
#define WS2812_H

#include "inc/stm32f030x6.h"

#include "peripherals.h"

#include <stdint.h>
#include <stdbool.h>

/* Set the number of leds of data you want to buffer */
/* NB: This should always be an even number! */
#define DMA_LED_BUFF 4
#define DMA_BUFF_SIZE (DMA_LED_BUFF * 8 * 3)
#define DMA_HALF_SIZE (DMA_BUFF_SIZE/2)
#define DMA_LOWER_HALF_OFFSET 0
#define DMA_UPPER_HALF_OFFSET DMA_HALF_SIZE

typedef enum
{
	LED_GRB = 0,
	LED_RGB = 1,
	LED_RGBW = 2,
} LED_FORMAT_E;

typedef struct
{
	uint8_t *data;
	uint8_t bit_mask;
	uint16_t arr_pos;
	const uint16_t num_leds;
	const LED_FORMAT_E data_format;
} rgb_led_t;

void led_fill_dma_buffer(rgb_led_t *leds, uint16_t offset, uint16_t length);

void dma_setup(void);

void led_init(void);

void led_write_all(rgb_led_t *leds, uint8_t red, uint8_t green, uint8_t blue);

void led_show(rgb_led_t *leds, TIM_TypeDef *TIMx);

#endif //WS2812_H
