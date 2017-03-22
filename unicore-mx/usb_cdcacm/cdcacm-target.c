/*
 * This file is part of the unicore-mx project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2014 Kuldeep Singh Dhaka <kuldeepdhaka9@gmail.com>
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

#include <stdlib.h>
#include <unicore-mx/stm32/rcc.h>
#include <unicore-mx/stm32/flash.h>
#include <unicore-mx/stm32/gpio.h>
#include <unicore-mx/stm32/crs.h>
#include <unicore-mx/stm32/syscfg.h>
#include <unicore-mx/usbd/usbd.h>
#include <unicore-mx/cm3/scb.h>
#include "cdcacm-target.h"

/* Set STM32 to 48 MHz, based on the 12MHz crystal */
static void clock_setup(void)
{
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

void cdcacm_target_init(void)
{
    clock_setup();

	rcc_set_usbclk_source(RCC_PLL);

	rcc_periph_clock_enable(RCC_SYSCFG_COMP);

	SYSCFG_CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;
}

const usbd_backend *cdcacm_target_usb_driver(void)
{
	return USBD_STM32_FSDEV_V2;
}
