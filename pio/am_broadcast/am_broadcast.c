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

void setFreq(PIO pio, uint sm, float freq);
void setNote(PIO pio, uint sm, int note);
void playSong(PIO pio, uint sm, int delay_ms, int transpose, int* note, int count);
void am_broadcast_forever(PIO pio, uint sm, uint offset, uint tx_pin, uint comms_pin, uint carrier_freq);
void freq_gen_forever(PIO pio, uint sm, uint offset, uint pin, uint carrier_freq);

int song[] = {40, 42, 44, 42, 40, 47, 40, 40};
// According to chatgpt, hahaha...
int never_gonna_give_you_up_chorus_indices[] = {
    34, 34, 36, 39, 41, 36, 41, 36, 34, 41, 41, 39, 36, 34, 34,
    36, 39, 41, 36, 41, 36, 34, 41, 41, 39, 36, 34, 32, 36, 39,
    41, 39, 36, 34, 29, 31, 32, 32, 29, 29, 31, 34, 32, 29, 31,
    29, 32, 32, 29, 29, 31, 34, 36, 36, 36, 34, 29, 29, 34, 32,
    36, 34, 29, 29, 34, 32, 36, 34, 29, 31, 36, 34, 29, 29, 34,
    32, 36, 34, 29, 31, 29, 36, 32, 32, 32, 31, 29, 32, 34, 36,
    36, 36, 36, 32, 32, 36, 34, 29, 29, 34, 32, 36, 34, 29, 31,
    36, 34, 29, 29, 34, 32, 36, 34, 29, 31, 29, 36, 32, 32, 32,
    29, 31, 32, 34, 36, 36, 36, 32, 32, 36, 34, 29, 29, 34, 32,
    36, 34, 29, 31, 36, 34, 29, 29, 34, 32, 36, 34, 29, 31, 29,
    36, 32, 32, 32, 29, 31, 32, 34, 36, 36, 36, 32, 32, 36, 34,
    29, 29, 34, 32, 36, 34, 29, 31, 36, 34, 29, 29, 34, 32, 36,
    34, 29, 31, 29, 36, 32, 32, 32, 29, 31, 32, 34, 36, 36, 36,
    32, 32, 36, 34, 29, 29, 34, 32, 36, 34, 29, 31, 36, 34, 29,
    29, 34, 32, 36, 34, 29, 31, 29, 36, 32, 32, 32, 29, 31, 32,
    34, 36, 36, 36, 32, 32, 36, 34, 29, 29, 34, 32, 36, 34, 29,
    31, 36, 34, 29, 29, 34, 32, 36, 34, 29, 31, 29, 36, 32, 32,
    32, 29, 31, 32, 34, 36, 36, 36, 32, 32, 36, 34, 29, 29, 34,
    32, 36, 34, 29, 31, 36, 34, 29, 29, 34, 32, 36, 34, 29, 31,
    29, 36, 32, 32, 32, 29, 31, 32, 34, 36, 36, 36, 32, 32, 36,
    34, 29, 29, 34, 32, 36, 34, 29, 31, 36, 34, 29, 29, 34, 32,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int main() {
    setup_default_uart();
    set_sys_clock_khz(1600*10*2, true);

    int tx_pin = 8;
    int comms_pin = 9;

    uint ab_offset = pio_add_program(pio0, &am_broadcast_program);
    printf("Loaded program at %d\n", ab_offset);
    uint carrier_freq = 1400000;
    am_broadcast_forever(pio0, 0, ab_offset, tx_pin, comms_pin, carrier_freq);

    uint fg_offset = pio_add_program(pio1, &freq_gen_program);
    printf("Loaded program at %d\n", fg_offset);
    freq_gen_forever(pio1, 0, fg_offset, comms_pin, 1);

    // int i = 0;
    // while (true) {
    //     setFreq(pio1, 0, pow(1.1,i));
    //     i++;
    //     sleep_ms(100);
    //     i %= 1000;
    // }

    // int i = 20;
    // while (true) {
    //     setFreq(pio1, 0, i);
    //     i--;
    //     sleep_ms(100);
    //     if (i <= 0) {
    //         i = 20;
    //     }
    // }

    // int i = 0;
    // while (true) {
    //     setNote(pio1, 0, i);
    //     i++;
    //     sleep_ms(250);
    //     i %= 100;
    // }

    int count;
    while (true) {
        count = sizeof(song) / sizeof(song[0]);
        playSong(pio1, 0, 250, 1, song, count);

        count = sizeof(never_gonna_give_you_up_chorus_indices) / sizeof(never_gonna_give_you_up_chorus_indices[0]);
        playSong(pio1, 0, 100, 1, never_gonna_give_you_up_chorus_indices, count);
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
    for (int i = 0; i < count; i++) {
        setNote(pio, sm, note[i]+transpose);
        sleep_ms(delay_ms);
    }
    setNote(pio, sm, 0);
}

void am_broadcast_forever(PIO pio, uint sm, uint offset, uint tx_pin, uint comms_pin, uint carrier_freq) {
    am_broadcast_program_init(pio, sm, offset, tx_pin, comms_pin, carrier_freq);
    pio_sm_set_enabled(pio, sm, true);

    printf("Broadcasting AM on pin %d at %d Hz with comms pin %d\n", tx_pin, carrier_freq, comms_pin);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    //pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}

void freq_gen_forever(PIO pio, uint sm, uint offset, uint comms_pin, uint freq) {
    freq_gen_program_init(pio, sm, offset, comms_pin, freq);
    pio_sm_set_enabled(pio, sm, true);

    printf("Generating frequency on pin %d\n", comms_pin);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    //pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}
