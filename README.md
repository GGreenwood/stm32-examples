# stm32-examples
Short example projects for the STM32F070F6 using libopencm3. All projects are intended to be uploaded to a bare breadboarded chip with a 12MHz oscillator.

Example breadboard setup is shown [here](https://twitter.com/shinewavegcn/status/842257523717603328).

To set up, pull the libopencm3 submodule and build it. Then go into an example project direction and build with `make` and flash over DFU with `make dfu-flash`.

## blinky
Blinks PA1 using a wait loop.

## pwm
Configures TIM3 to output a 25% 250kHz pulse to PA6.
