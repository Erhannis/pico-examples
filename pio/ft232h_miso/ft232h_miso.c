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
#include "freq_gen.pio.h"
#include "count2ftdi.pio.h"
#include "write2ftdi.pio.h"

void setFreq(PIO pio, uint sm, float freq);
void setNote(PIO pio, uint sm, int note);
void playSong(PIO pio, uint sm, int delay_ms, int transpose, int* note, int count);
void playSongRhythm(PIO pio, uint sm, int delay_ms, int transpose, int* note, int* rhythm, int count);
void count2ftdi_forever(PIO pio, uint sm, uint offset, uint tx_pin, uint freq);
void write2ftdi_forever(PIO pio, uint sm, uint offset, uint tx_pin);
void freq_gen_forever(PIO pio, uint sm, uint offset, uint pin, uint carrier_freq);

int main() {
    setup_default_uart();

    uint vco, postdiv1, postdiv2;
    int maxf = 0;
    
    int target = 240000;
    //int target = 275000;

    //int target = 300000;
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

    set_sys_clock_khz(nearest, true);

    int tx_pin = 15;
    int comms_pin = 9;

    uint fg_offset = pio_add_program(pio0, &freq_gen_program);
    printf("Loaded program at %d\n", fg_offset);
    freq_gen_forever(pio0, 0, fg_offset, comms_pin, 1);

    uint cf_offset = pio_add_program(pio1, &write2ftdi_program);
    printf("Loaded program at %d\n", cf_offset);
    write2ftdi_forever(pio1, 0, cf_offset, tx_pin);

    int cycles_per_bit = 2;
    int sys_clock = clock_get_hz(clk_sys);
    float div = sys_clock / (60000000 * cycles_per_bit);
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

float noteBase = pow(2, 1.0/12);
void setNote(PIO pio, uint sm, int note) {
    setFreq(pio, sm, pow(noteBase, note-49)*440);
}

void playSong(PIO pio, uint sm, int delay_ms, int transpose, int* note, int count) {
    playSongRhythm(pio, sm, delay_ms, transpose, note, NULL, count);
}

void playSongRhythm(PIO pio, uint sm, int delay_ms, int transpose, int* note, int* rhythm, int count) {
    for (int i = 0; i < count; i++) {
        setNote(pio, sm, note[i]+transpose);
        if (rhythm != NULL) {
            sleep_ms(delay_ms * rhythm[i]);
        } else {
            sleep_ms(delay_ms);
        }
    }
    setNote(pio, sm, 0);
}

void count2ftdi_forever(PIO pio, uint sm, uint offset, uint tx_pin, uint freq) {
    count2ftdi_program_init(pio, sm, offset, tx_pin, freq);
    pio_sm_set_enabled(pio, sm, true);

    printf("Counting on pins %d at %d KHz\n", tx_pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    //pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}

void write2ftdi_forever(PIO pio, uint sm, uint offset, uint tx_pin) {
    write2ftdi_program_init(pio, sm, offset, tx_pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Writing to FTDI on pins %d\n", tx_pin);
}

void freq_gen_forever(PIO pio, uint sm, uint offset, uint comms_pin, uint freq) {
    freq_gen_program_init(pio, sm, offset, comms_pin, freq);
    pio_sm_set_enabled(pio, sm, true);

    printf("Generating frequency on pin %d\n", comms_pin);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    //pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}
