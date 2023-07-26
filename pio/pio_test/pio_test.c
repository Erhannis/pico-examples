/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "read_epc.pio.h"
#include "write_fdic.pio.h"

void read_epc_loop(PIO pio, uint sm, uint offset, uint pin, uint freq);
void write_fdic_loop(PIO pio, uint sm, uint offset, uint pin, uint freq);

int main() {
    setup_default_uart();

    // todo get free sm
    uint offset0 = pio_add_program(pio0, &read_epc_program);
    printf("Loaded program at %d\n", offset0);

    uint offset1 = pio_add_program(pio0, &write_fdic_program);
    printf("Loaded program at %d\n", offset1);

    read_epc_loop(pio0, 0, offset0, 0, 3);
    write_fdic_loop(pio1, 0, offset1, 0, 3);
}

//DUMMY
void read_epc_loop(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    read_epc_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}

void write_fdic_loop(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    write_fdic_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}
