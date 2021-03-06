/*
 * This file is part of the unicore-mx project.
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

/* Set STM32 to 48 MHz, based on the 12MHz crystal */
static void clock_setup(void)
{
    rcc_osc_off(RCC_PLL);
    rcc_set_pll_multiplication_factor(4);
    rcc_osc_on(RCC_PLL);
    rcc_wait_for_osc_ready(RCC_PLL);
    rcc_set_sysclk_source(RCC_HSI);
}

static void gpio_setup(void)
{
    /* Enable GPIOA clock. */
    rcc_periph_clock_enable(RCC_GPIOA);

    /* Set GPIO1 in PORT1 to 'output push-pull'. */
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
}

static void button_setup(void)
{
    /* Enable GPIOA clock. */
    rcc_periph_clock_enable(RCC_GPIOA);

    /* Set GPIO0 (in GPIO port A) to 'input open-drain'. */
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
}

int main(void)
{
    int i;

    clock_setup();
    button_setup();
    gpio_setup();

    /* Blink the LED (PD12) on the board. */
    while (1) {
        gpio_toggle(GPIOA, GPIO1);

        /* Upon button press, blink more slowly. */
        if (gpio_get(GPIOA, GPIO0)) {
            for (i = 0; i < 3000000; i++) {	/* Wait a bit. */
                __asm__("nop");
            }
        }

        for (i = 0; i < 300000; i++) {		/* Wait a bit. */
            __asm__("nop");
        }
    }

    return 0;
}
