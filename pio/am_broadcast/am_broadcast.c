/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "am_broadcast.pio.h"

void am_broadcast_forever(PIO pio, uint sm, uint offset, uint pin, uint carrier_freq);

int main() {
    setup_default_uart();
    set_sys_clock_khz(1600*10*2, true);

    uint offset = pio_add_program(pio0, &am_broadcast_program);
    printf("Loaded program at %d\n", offset);
    uint carrier_freq = 1400000;
    am_broadcast_forever(pio0, 0, offset, 8, carrier_freq);

    int i = 0;
    while (true) {
        //printf("loop %d\n", i);
        //CHECK //SHAME It seems bad to do direct fifo access?
        if (i % 2) {
            pio0->txf[0] = 0xFFFFFFFF;
        } else {
            pio0->txf[0] = 0x00000000;
        }
        sleep_us(1000*(sin(time_us_64()/1000000.0)+1));
        i++;
    }
}

void am_broadcast_forever(PIO pio, uint sm, uint offset, uint pin, uint carrier_freq) {
    am_broadcast_program_init(pio, sm, offset, pin, carrier_freq);
    pio_sm_set_enabled(pio, sm, true);

    printf("Broadcasting AM on pin %d at %d Hz\n", pin, carrier_freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    //pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}
