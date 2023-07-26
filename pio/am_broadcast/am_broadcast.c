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
void playSongRhythm(PIO pio, uint sm, int delay_ms, int transpose, int* note, int* rhythm, int count);
void am_broadcast_forever(PIO pio, uint sm, uint offset, uint tx_pin, uint comms_pin, uint carrier_freq);
void freq_gen_forever(PIO pio, uint sm, uint offset, uint pin, uint carrier_freq);

// Song tweaked from https://projecthub.arduino.cc/rowan07
#define  a3f    36     // 208 Hz
#define  a3     37     //
#define  b3f    38     // 233 Hz
#define  b3     39     // 247 Hz
#define  c4     40     // 261 Hz MIDDLE C
#define  c4s    41     // 277 Hz
#define  d4f    41     // 277 Hz
#define  d4     42     // 
#define  e4f    43     // 311 Hz    
#define  e4     44     //    
#define  f4     45     // 349 Hz 
#define  f4s    46     //
#define  g4     47     //
#define  a4f    48     // 415 Hz  
#define  a4     49     //
#define  b4f    50     // 466 Hz 
#define  b4     51     //  493 Hz 
#define  c5     52     // 523 Hz 
#define  c5s    53     // 554  Hz
#define  d5f    53     // 554  Hz
#define  d5     54     //
#define  e5f    55     // 622 Hz  
#define  e5     56     //
#define  f5     57     // 698 Hz 
#define  f5s    58     // 740 Hz
#define  g5     59     //
#define  a5f    60     // 831 Hz 

int song1_chorus_tempo = (int)(1000/(453/60.0));

int song1_chorus_melody[] =
{ a4f, b4f, d5f, b4f,
  f5, f5, e5f, a4f, b4f, d5f, b4f, e5f, e5f, c5s, c5, b4f,
  a4f, b4f, d5f, b4f,
  c5s, e5f, c5, b4f, a4f, a4f, a4f, e5f, c5s,
  a4f, b4f, d5f, b4f,
  f5,  f5, e5f, a4f, b4f, d5f, b4f, a5f, c5, c5s, c5, b4f,
  a4f, b4f, d5f, b4f,
  c5s, e5f, c5, b4f, a4f, 0, a4f, e5f, c5s, 0
};

int song1_chorus_rhythm[]  =
{ 1, 1, 1, 1,
  3, 3, 6, 1, 1, 1, 1, 3, 3, 3, 1, 2,
  1, 1, 1, 1,
  3, 3, 3, 1, 2, 2, 2, 4, 8,
  1, 1, 1, 1,
  3, 3, 6, 1, 1, 1, 1, 3, 3, 3,  1, 2,
  1, 1, 1, 1,
  3, 3, 3, 1, 2, 2, 2, 4, 8, 4
};

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
        count = sizeof(song1_chorus_melody) / sizeof(song1_chorus_melody[0]);
        playSongRhythm(pio1, 0, song1_chorus_tempo, 0, song1_chorus_melody, song1_chorus_rhythm, count);
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
