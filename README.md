# stm32-examples
Short example projects for the STM32F070F6 using unicore-mx and libopencm3. The projects for each library are located in `unicore-mx`, and `libopencm3`, respectively.

All projects are intended to be uploaded to a bare breadboarded chip with a 12MHz oscillator.

Example breadboard setup is shown [here](https://twitter.com/shinewavegcn/status/842257523717603328).

To set up, pull the unicore-mx or libopencm3 submodule and build it. Then go into an example project direction and build with `make` and flash over DFU with `make dfu-flash`.
