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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>

#define DMA_SIZE    64

uint16_t waveform[DMA_SIZE];

/* Set STM32 to 48 MHz, based on the 12MHz crystal */
static void clock_setup_pll_48mhz(void) {
    // If we're the PLL is selected as the system clock,
    // we need to switch to HSI before configuring
    if(rcc_system_clock_source() == RCC_PLL) {
        //TODO
    }

    // Turn off PLL
    //rcc_osc_off(RCC_PLL);     << this isn't implemented yet
    RCC_CR &= ~RCC_CR_PLLON;

    // Wait for PLL to turn off
    while ((RCC_CR & RCC_CR_PLLRDY) != 0);

    // Configure the predivider and multiplier
    // (12MHz / 2) * 8 = 48MHz
    rcc_set_prediv(RCC_CFGR2_PREDIV_DIV2);
    rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_MUL8);

    // Set the PLL source to the HSE
    RCC_CFGR |= RCC_CFGR_PLLSRC;

    // Set the AHB and APB prescalers to 1
    rcc_set_ppre(RCC_CFGR_PPRE_NODIV);
    rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
    rcc_apb1_frequency = 48000000;
    rcc_ahb_frequency = 48000000;

    // Update the flash memory speed before increasing the clock speed
    flash_set_ws(FLASH_ACR_LATENCY_024_048MHZ);

    // Turn on the source oscillator
    rcc_osc_on(RCC_HSE);
    rcc_wait_for_osc_ready(RCC_HSE);

    // Turn on PLL
    rcc_osc_on(RCC_PLL);
    rcc_wait_for_osc_ready(RCC_PLL);
    rcc_set_sysclk_source(RCC_PLL);
}

static void gpio_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO6);
}

static void pwm_setup(void) {
    // Enable peripheral timers
    rcc_periph_clock_enable(RCC_TIM3);

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
    // Will be low for 1us, high for the remaining 3us
    timer_set_oc_value(TIM3, TIM_OC1, 48 - 1);

    // Set TIM3's period to 4us at 48MHz
    timer_set_period(TIM3, 192 - 1);

    // Enable interrupts on overflow
    timer_enable_irq(TIM3, TIM_DIER_CC1IE);
    nvic_enable_irq(NVIC_TIM3_IRQ);

    // Enable the timer
    timer_enable_counter(TIM3);
}

// TIM3, Channel 1 DMA events are sent to DMA Channel 4
static void dma_setup(void) {
    rcc_periph_clock_enable(RCC_DMA);

    dma_channel_reset(DMA1, DMA_CHANNEL4);
    dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_HIGH);

    dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_16BIT);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
    dma_enable_circular_mode(DMA1, DMA_CHANNEL4);

    dma_set_read_from_memory(DMA1, DMA_CHANNEL4);

    dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t) &TIM3_CCR1);
    dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t) &waveform[0]);

    dma_set_number_of_data(DMA1, DMA_CHANNEL4, DMA_SIZE);

    nvic_enable_irq(NVIC_DMA1_CHANNEL4_5_IRQ);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);

    // Enable DMA interrupt source
    timer_enable_irq(TIM3, TIM_DIER_CC1DE);
    //timer_set_dma_on_compare_event(TIM3);

    dma_enable_channel(DMA1, DMA_CHANNEL4);
}

void tim3_isr(void) {
    timer_clear_flag(TIM3, TIM_SR_CC1IF);
    /* Toggle PA3 just to keep aware of activity and frequency. */
    gpio_toggle(GPIOA, GPIO3);
}

void dma1_channel4_5_isr(void){
    dma_clear_interrupt_flags(DMA1, DMA_CHANNEL4, DMA_TCIF);
    /* Toggle PA1 just to keep aware of activity and frequency. */
    gpio_toggle(GPIOA, GPIO1);
}

int main(void) {
    uint16_t i;
    uint32_t x;

    // Set up DMA waveform
    for(i = 0; i < DMA_SIZE; i++) {
        waveform[i] = (30+i);
    }

    clock_setup_pll_48mhz();
    gpio_setup();
    pwm_setup();
    dma_setup();

    while (1) {
        for (x = 0; x < 300000; x++) {		/* Wait a bit. */
            __asm__("nop");
        }
    }

    return 0;
}
