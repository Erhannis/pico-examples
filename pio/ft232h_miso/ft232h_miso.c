/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>
#include <hardware/vreg.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "am_broadcast.pio.h"
#include "freq_gen.pio.h"
#include "count2ftdi.pio.h"
#include "write2ftdi.pio.h"

void setFreq(PIO pio, uint sm, float freq);
void write2ftdi_forever(PIO pio, uint sm, uint offset, uint data_pin, uint txe_then_clk_pin, uint wr_pin);

int comms_pin = 9;
int data_pins_8 = 15;
int txe_then_clk_pin2_2 = 26;
int wr_pin = 28;

int main() {
    setup_default_uart();

    uint vco, postdiv1, postdiv2;
    int maxf = 0;
    
    //int target = 240000;
    int target = 275000;
    //int target = 280000;

    //int target = 290000;
    //int target = 300000;
    //int target = 360000;
    
    int nearest = 0;
    for (int f = 0; f < 1000000; f += 1000) {
        if (check_sys_clock_khz(f, &vco, &postdiv1, &postdiv2)) {
            printf("%d\n", f);
            maxf = f;
            if (abs(target-f) < abs(target-nearest)) {
                nearest = f;
            }
        }
    }
    printf("max: %d\n", maxf);
    printf("nearest: %d\n", nearest);

    //vreg_set_voltage(VREG_VOLTAGE_1_20);
    set_sys_clock_khz(nearest, true);

    uint cf_offset = pio_add_program(pio1, &write2ftdi_program);
    printf("Loaded program at %d\n", cf_offset);
    write2ftdi_forever(pio1, 0, cf_offset, data_pins_8, txe_then_clk_pin2_2, wr_pin);

    int cycles_per_bit = 2;
    int sys_clock = clock_get_hz(clk_sys);
    float div = sys_clock / (1000000 * cycles_per_bit);
    //pio_sm_set_clkdiv(pio1, 0, div);

    uint32_t count = 0;
    while (true) {
        pio_sm_put_blocking(pio1, 0, count); // 3-4 counts

        //pio1->txf[0] = count; // 1-2 counts , but I prob...MAYBE need the blocking.

        count++;
    }
}

void setFreq(PIO pio, uint sm, float freq) {
    if (freq < 7) {
        freq = 7; //DUMMY Turn off?
        int cycles_per_bit = 32*2;
        int sys_clock = clock_get_hz(clk_sys);
        float div = sys_clock / (freq * cycles_per_bit);
        pio_sm_set_clkdiv(pio, sm, div);
    } else {
        int cycles_per_bit = 32*2;
        int sys_clock = clock_get_hz(clk_sys);
        float div = sys_clock / (freq * cycles_per_bit);
        pio_sm_set_clkdiv(pio, sm, div);
    }
}

void write2ftdi_forever(PIO pio, uint sm, uint offset, uint data_pin, uint txe_then_clk_pin, uint wr_pin) {
    write2ftdi_program_init(pio, sm, offset, data_pin, txe_then_clk_pin, wr_pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Writing to FTDI on pins %d %d %d\n", data_pin, txe_then_clk_pin, wr_pin);
}
