#include "inc/stm32f030x6.h"

#include <stdint.h>
#include <stdbool.h>

#include "peripherals.h"

void gpio_init(GPIO_PORT_E gpio_port, const GPIO_PIN_E io_pin, GPIO_MODER_E gpio_mode,
				GPIO_ALT_MODE_E gpio_af)
{
	if (gpio_port == PORTA)
	{
		GPIOA->MODER &= ~(GPIO_MODER_MAX << (io_pin * 2));
		GPIOA->MODER |= (gpio_mode << (io_pin * 2));
		if (gpio_mode == GPIO_ALT_MODE)
		{
			/* First, clear the 4 bit nibble you want to set, then or the */
			/* register with the value you want to set it to */
			GPIOA->AFR[io_pin/8] &= ~(GPIO_AF_MAX << ((io_pin & 0x7) * 4));
			GPIOA->AFR[io_pin/8] |=  (gpio_af     << ((io_pin & 0x7) * 4));
		}
	}
	else if (gpio_port == PORTB)
	{
		GPIOB->MODER &= ~(GPIO_MODER_MAX << (io_pin * 2));
		GPIOB->MODER |= (gpio_mode << (io_pin * 2));
		if (gpio_mode == GPIO_ALT_MODE)
		{
			/* First, clear the 4 bit nibble you want to set, then or the */
			/* register with the value you want to set it to */
			GPIOB->AFR[io_pin/8] &= ~(GPIO_AF_MAX << ((io_pin & 0x7) * 4));
			GPIOB->AFR[io_pin/8] |=  (gpio_af     << ((io_pin & 0x7) * 4));
		}
	}
	else if (gpio_port == PORTC)
	{
		GPIOC->MODER &= ~(GPIO_MODER_MAX << (io_pin * 2));
		GPIOC->MODER |= (gpio_mode << (io_pin * 2));
		if (gpio_mode == GPIO_ALT_MODE)
		{
			/* First, clear the 4 bit nibble you want to set, then or the */
			/* register with the value you want to set it to */
			GPIOC->AFR[io_pin/8] &= ~(GPIO_AF_MAX << ((io_pin & 0x7) * 4));
			GPIOC->AFR[io_pin/8] |=  (gpio_af     << ((io_pin & 0x7) * 4));
		}
	}
	else if (gpio_port == PORTD)
	{
		GPIOD->MODER &= ~(GPIO_MODER_MAX << (io_pin * 2));
		GPIOD->MODER |= (gpio_mode << (io_pin * 2));
		if (gpio_mode == GPIO_ALT_MODE)
		{
			/* First, clear the 4 bit nibble you want to set, then or the */
			/* register with the value you want to set it to */
			GPIOD->AFR[io_pin/8] &= ~(GPIO_AF_MAX << ((io_pin & 0x7) * 4));
			GPIOD->AFR[io_pin/8] |=  (gpio_af     << ((io_pin & 0x7) * 4));
		}
	}
	/* NB: There is no port E on the STM32F030 */
	else if (gpio_port == PORTF)
	{
		GPIOF->MODER &= ~(GPIO_MODER_MAX << (io_pin * 2));
		GPIOF->MODER |= (gpio_mode << (io_pin * 2));
		if (gpio_mode == GPIO_ALT_MODE)
		{
			/* First, clear the 4 bit nibble you want to set, then or the */
			/* register with the value you want to set it to */
			GPIOF->AFR[io_pin/8] &= ~(GPIO_AF_MAX << ((io_pin & 0x7) * 4));
			GPIOF->AFR[io_pin/8] |=  (gpio_af     << ((io_pin & 0x7) * 4));
		}
	}
}

void init_timer(TIM_TypeDef *TIMx)
{
	/* Make sure the timer is disabled before starting initialisation */
	TIMx->CR1 &= ~(TIM_CR1_CEN);

	/* Reset and start the timer */
	if (TIMx == TIM1)
	{
		RCC->APB2RSTR |=   RCC_APB2RSTR_TIM1RST;
		RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM1RST);
	}
	else if (TIMx == TIM16)
	{
		RCC->APB2RSTR |=   RCC_APB2RSTR_TIM16RST;
		RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM16RST);
	}
	else if (TIMx == TIM17)
	{
		RCC->APB2RSTR |=   RCC_APB2RSTR_TIM17RST;
		RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM17RST);
	}
	else if (TIMx == TIM3)
	{
		RCC->APB1RSTR |=   RCC_APB1RSTR_TIM3RST;
		RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM3RST);
	}
	else if (TIMx == TIM14)
	{
		RCC->APB1RSTR |=   RCC_APB1RSTR_TIM14RST;
		RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM14RST);
	}
}

void start_timer(TIM_TypeDef *TIMx, uint16_t prescale, uint16_t count)
{
	/* Set the prescaler */
	TIMx->PSC = prescale;
	/* Set the value to count to */
	TIMx->ARR = count;
	/* Set the update event to generate an update of its registers and reset it */
	TIMx->EGR |= TIM_EGR_UG;
	/* Setup timer to trigger a hardware interrupt upon reaching the count */
	TIMx->DIER |= TIM_DIER_UIE;
	/* Now enable the timer */
	TIMx->CR1 |= TIM_CR1_CEN;
}

void setup_timer_capture_compare(TIM_TypeDef *TIMx, const TIMER_CHANNEL_E channel,
		uint16_t ARR, uint16_t CCR, uint16_t prescale)
{
	/* Set prescaler */
	TIMx->PSC = prescale;
	/* Set the overall count */
	TIMx->ARR = ARR;

	switch(channel)
	{
		case TIM_CHAN_1:
			/* Set the first count in the capture compare register */
			TIMx->CCR1 = CCR;
			/* Set the output to go high when the CNT register reaches the value in */
			/* CCR1. */
			TIMx->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk);
			TIMx->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
			/* Disable the capture/compare 1 output for now*/
			/* NB: Can also set the polarity in this register */
			TIMx->CCER |= ~TIM_CCER_CC1E;
			break;
		case TIM_CHAN_2:
			/* Set the first count in the capture compare register */
			TIMx->CCR2 = CCR;
			/* Set the output to go high when the CNT register reaches the value in */
			/* CCR1. */
			TIMx->CCMR1 &= ~(TIM_CCMR1_OC2M_Msk);
			TIMx->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);
			/* Disable the capture/compare 1 output for now*/
			/* NB: Can also set the polarity in this register */
			TIMx->CCER |= ~TIM_CCER_CC2E;
			break;
		case TIM_CHAN_3:
			/* Set the first count in the capture compare register */
			TIMx->CCR3 = CCR;
			/* Set the output to go high when the CNT register reaches the value in */
			/* CCR1. */
			TIMx->CCMR2 &= ~(TIM_CCMR2_OC3M_Msk);
			TIMx->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2);
			/* Disable the capture/compare 1 output for now*/
			/* NB: Can also set the polarity in this register */
			TIMx->CCER |= ~TIM_CCER_CC3E;
			break;
		case TIM_CHAN_4:
			/* Set the first count in the capture compare register */
			TIMx->CCR4 = CCR;
			/* Set the output to go high when the CNT register reaches the value in */
			/* CCR1. */
			TIMx->CCMR2 &= ~(TIM_CCMR2_OC4M_Msk);
			TIMx->CCMR2 |= (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2);
			/* Disable the capture/compare 1 output for now*/
			/* NB: Can also set the polarity in this register */
			TIMx->CCER |= ~TIM_CCER_CC4E;
			break;
	}

	/* Set the update event to generate an update of its registers and reset it */
	TIMx->EGR |= TIM_EGR_UG;
	/* Set the ARPE bit to allow changes to registers to take immediate effect */
	TIMx->CR1 |= TIM_CR1_ARPE;
	/* Setup timer to trigger a hardware interrupt upon reaching the count */
	TIMx->DIER |= TIM_DIER_UIE;
	/* Disable the timer for now*/
	TIMx->CR1 &= ~TIM_CR1_CEN;
}

void clock_setup(bool external_clk, bool use_pll, PLL_MULT_E pll_mult)
{
	/* Enable the selected clock and wait for it to be ready */
	if (external_clk == true)
	{
		RCC->CR |= RCC_CR_HSEON;
		while(!(RCC->CR & RCC_CR_HSERDY)){};
	}
	else
	{
		RCC->CR |= RCC_CR_HSION;
		while(!(RCC->CR & RCC_CR_HSIRDY)){};
	}

	/* Now enable the PLL if needed */
	if (use_pll == true)
	{
		if (external_clk == true)
		{
			RCC->CFGR &= ~(RCC_CFGR_PLLMUL |
						   RCC_CFGR_PLLSRC);
			RCC->CFGR |= (RCC_CFGR_PLLSRC_HSE_PREDIV |
						  pll_mult);
		}
		else
		{
			RCC->CFGR &= ~(RCC_CFGR_PLLMUL |
						   RCC_CFGR_PLLSRC);
			RCC->CFGR |= (RCC_CFGR_PLLSRC_HSI_DIV2 |
						  pll_mult);
		}

		/* Turn on the PLL and wait for the hardware to set the ready flag */
		RCC->CR |= RCC_CR_PLLON;
		while(!(RCC->CR & RCC_CR_PLLRDY)){};
	}

	/* Now set the clock source of the system clock */
	if (use_pll == true)
	{
		RCC->CFGR &= ~(RCC_CFGR_SW);
		RCC->CFGR |= RCC_CFGR_SW_PLL;
		while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)){};
	}
	else if (external_clk == true)
	{
		RCC->CFGR &= ~(RCC_CFGR_SW);
		RCC->CFGR |= RCC_CFGR_SW_HSE;
		while(!(RCC->CFGR & RCC_CFGR_SWS_HSE)){};
	}
	else if (external_clk == false)
	{
		RCC->CFGR &= ~(RCC_CFGR_SW);
		RCC->CFGR |= RCC_CFGR_SW_HSI;
		while(!(RCC->CFGR & RCC_CFGR_SWS_HSI)){};
	}
}
