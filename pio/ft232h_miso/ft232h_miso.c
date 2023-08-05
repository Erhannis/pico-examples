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
void freq_gen_forever(PIO pio, uint sm, uint offset, uint comms_pin);
void write2ftdi_forever(PIO pio, uint sm, uint offset, uint data_pins_8, uint cs_a0_wr_pin);

int comms_pin = 9;
int data_pins_8 = 15;
int cs_a0_wr_pin = 26;

int main() {
    setup_default_uart();

    uint vco, postdiv1, postdiv2;
    int maxf = 0;
    
    // int target = 25000;
    // int target = 100000;
    // int target = 120000;
    // int target = 140000;
    // int target = 200000;
    int target = 240000;
    // int target = 260000;
    // int target = 275000;
    // int target = 280000;

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

    // //BCRASH This is bad, according to the docs, something something metastable
    // hw_set_bits(&pio1->input_sync_bypass, 1u << (txe_then_clk_pin2_2+1));

    // gpio_init(txe_then_clk_pin2_2+1);
    // gpio_set_dir(txe_then_clk_pin2_2+1, GPIO_IN);
    // uint32_t events = GPIO_EVT_EDGE_RISE;
    // gpio_set_irq_enabled(PIN, GPIO_EVT_EDGE_RISE, true, &gpio_callback);

    uint fg_offset = pio_add_program(pio0, &freq_gen_program);
    printf("Loaded program at %d\n", fg_offset);
    freq_gen_forever(pio0, 0, fg_offset, comms_pin);

    uint cf_offset = pio_add_program(pio1, &write2ftdi_program);
    printf("Loaded program at %d\n", cf_offset);
    write2ftdi_forever(pio1, 0, cf_offset, data_pins_8, cs_a0_wr_pin);

    // pio1->sm[0].shiftctrl =
    //     (1u << PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_LSB) |
    //     (1u << PIO_SM0_SHIFTCTRL_AUTOPULL_LSB) |
    //     (7u << PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB); //CHECK
    pio1->sm[0].shiftctrl = 
        (1u << PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_LSB) |
        (1u << PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_LSB) |
        0;



    int cycles_per_bit = 2;
    int sys_clock = clock_get_hz(clk_sys);
    float div = sys_clock / (1000000 * cycles_per_bit);
    //pio_sm_set_clkdiv(pio1, 0, div);

    uint32_t count = 0;
    while (true) {
        pio_sm_put_blocking(pio1, 0, count);
        // pio_sm_put_blocking(pio1, 0, count & 0b10000001);

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

void freq_gen_forever(PIO pio, uint sm, uint offset, uint comms_pin) {
    freq_gen_program_init(pio, sm, offset, comms_pin);
    pio_sm_set_enabled(pio, sm, true);
}

void write2ftdi_forever(PIO pio, uint sm, uint offset, uint data_pins_8, uint cs_a0_wr_pin) {
    write2ftdi_program_init(pio, sm, offset, data_pins_8, cs_a0_wr_pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Writing to FTDI on pins %d %d %d\n", data_pins_8, cs_a0_wr_pin);
}
