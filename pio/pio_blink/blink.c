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
#include "blink.pio.h"
#include "blinkmax.pio.h"
#include "am_broadcast.pio.h"

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);
void blinkmax_pin_forever(PIO pio, uint sm, uint offset, uint pin);
void am_broadcast_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);

int main() {
    setup_default_uart();
    set_sys_clock_khz(1600*10*2, true);

    // todo get free sm
    uint offset0 = pio_add_program(pio0, &blink_program);
    printf("Loaded program at %d\n", offset0);

    blink_pin_forever(pio0, 0, offset0, 5, 3);
    blink_pin_forever(pio0, 1, offset0, 6, 4);
    blink_pin_forever(pio0, 2, offset0, 7, 1000);

    // uint offset1 = pio_add_program(pio1, &blinkmax_program);
    // printf("Loaded program at %d\n", offset1);
    // blinkmax_pin_forever(pio1, 0, offset1, 8);

    uint offset1 = pio_add_program(pio1, &am_broadcast_program);
    printf("Loaded program at %d\n", offset1);
    am_broadcast_forever(pio1, 0, offset1, 8, 1400000);

    int i = 0;
    while (true) {
        //printf("loop %d\n", i);
        //CHECK //SHAME It seems bad to do direct fifo access?
        if (i % 2) {
            pio1->txf[0] = 0xFFFFFFFF;
        } else {
            pio1->txf[0] = 0x00000000;
        }
        sleep_us(1000*(sin(time_us_64()/1000000.0)+1));
        i++;
    }}

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}

void blinkmax_pin_forever(PIO pio, uint sm, uint offset, uint pin) {
    blinkmax_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinkmaxing pin %d\n", pin);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    //pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}

void am_broadcast_forever(PIO pio, uint sm, uint offset, uint pin, uint carrier_freq) {
    am_broadcast_program_init(pio, sm, offset, pin, carrier_freq);
    pio_sm_set_enabled(pio, sm, true);

    printf("Broadcasting AM on pin %d at %d Hz\n", pin, carrier_freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    //pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}
