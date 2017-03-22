/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>,
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <unicore-mx/stm32/rcc.h>
#include <unicore-mx/stm32/gpio.h>
#include <unicore-mx/stm32/timer.h>

/* Set STM32 to 48 MHz, based on the 12MHz crystal */
static void clock_setup(void)
{
    rcc_osc_off(RCC_PLL);
    rcc_set_pll_multiplication_factor(4);
    rcc_osc_on(RCC_PLL);
    rcc_wait_for_osc_ready(RCC_PLL);
    rcc_set_sysclk_source(RCC_HSI);
}

int main(void)
{
    int i;

    clock_setup();

    // Enable peripheral timers
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_TIM3);

    // Set GPIO outputs
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO6);

    // Connect PA6 to its alternate function, TIM_CH1
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO6);   // Set AF1(TIM3_CH1) of PA1

    // Set up TIM3
    timer_reset(TIM3);
    timer_set_mode(TIM3,            // Operate on TIM3
            TIM_CR1_CKD_CK_INT,     // No prescaling(Internal clock)
            TIM_CR1_CMS_EDGE,       // Edge-aligned
            TIM_CR1_DIR_UP);        // Up-counting

    // Set up TIM3 output modes
    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM2);
    timer_enable_oc_output(TIM3, TIM_OC1);
    timer_enable_break_main_output(TIM3);

    // Configure channel 1(PA6)
    timer_set_oc_value(TIM3, TIM_OC1, (192 / 4) - 1);

    // Set TIM3's period
    timer_set_period(TIM3, 192 - 1);

    // Enable the timer
    timer_enable_counter(TIM3);

    // Blink the LED (PA1)
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);

    while (1) {
        gpio_toggle(GPIOA, GPIO1);

        for (i = 0; i < 3000000; i++) {
            __asm__("nop");
        }
    }

    return 0;
}
