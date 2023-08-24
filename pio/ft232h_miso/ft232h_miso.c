/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>
#include <hardware/vreg.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "pico/flash.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "am_broadcast.pio.h"
#include "freq_gen.pio.h"
#include "count2ftdi.pio.h"
#include "write2ftdi.pio.h"

void setFreq(PIO pio, uint sm, float freq);
void freq_gen_forever(PIO pio, uint sm, uint offset, uint comms_pin);
void write2ftdi_forever(PIO pio, uint sm, uint offset, uint data_pins_8, uint rd_wr_a0_cs_pin);

int comms_pin = 9;
int data_pins_8 = 22;
int wr_rd_a0_cs_pin = 18;

void core1_entry() {
    // pico_get_unique_id;
    // int initResult = flash_init();
    // printf("init: %d\n", initResult);
    
    // uint32_t flash_data;
    // flash_read(0, &flash_data, sizeof(flash_data));
    
    // printf("read data from flash: %u\n", flash_data);

    gpio_init(0);
    gpio_set_dir(0, GPIO_OUT);
    while (true) {
        gpio_put(0, true);
        sleep_ms(1);
        gpio_put(0, false);
        sleep_ms(1);
    }
}

#define FLASH_TARGET_OFFSET (256 * 1024)

const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

/*
Ok, I could swear to you, this program was not running on normal boot.
WAIT, NO, IT'S AGAIN NOT WORKING, AS EXPECTED.
...
Ok, so.  It's behaving VERY strangely - it exhibits a bug, I apply patch, the bug goes away,
I remove patch, the bug STAYS GONE, I try it on new board, IT STAYS GONE,
I try on another new board, the bug is back, and so forth.

So.  If you should find that the code runs from the debugger, but does not start on its own
when you plug in the usb, add to your rootmost CMakeLists.txt

set(PICO_BOARD pico_pair)

referencing pico-sdk/src/boards/include/boards/pico_pair.h
and if you don't have that file, just copy pico-sdk/src/boards/include/boards/pico.h
but add these three lines near the top, after the name definition (maybe rename the name lines appropriately)

#ifndef PICO_XOSC_STARTUP_DELAY_MULTIPLIER
#define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 64
#endif

See, for instance, https://forum.micropython.org/viewtopic.php?t=12695
*/
// int delayMul = PICO_XOSC_STARTUP_DELAY_MULTIPLIER;

int main() {
    stdio_init_all();
    // setup_default_uart();

    // printf("mul %d\n", delayMul);

    // uint8_t id_out[8];
    // id_out[0] = 0x17;
    // flash_get_unique_id(id_out);
    // printf("flash id: %02X%02X%02X%02X%02X%02X%02X%02X\n", id_out[0], id_out[1], id_out[2], id_out[3], id_out[4], id_out[5], id_out[6], id_out[7]);

    // uint8_t random_data[FLASH_PAGE_SIZE];
    // for (int i = 0; i < FLASH_PAGE_SIZE; ++i)
    //     random_data[i] = rand() >> 16;
    // random_data[0] = 0x00u;
    // random_data[1] = 0xFFu;
    // random_data[2] = 0x10u;
    // random_data[3] = 0x20u;
    // random_data[4] = 0xFFu;
    // random_data[5] = 0x00u;

    // flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    // flash_range_program(FLASH_TARGET_OFFSET, random_data, FLASH_PAGE_SIZE);

    uint vco, postdiv1, postdiv2;
    int maxf = 0;
    
    // int target = 25000;
    // int target = 100000;
    // int target = 120000;
    // int target = 140000;
    // int target = 200000;
    
    // int target = 180000;
    // int target = 240000;
    // int target = 250000;

    // int target = 260000;
    // int target = 275000;
    int target = 280000;

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
    write2ftdi_forever(pio1, 0, cf_offset, data_pins_8, wr_rd_a0_cs_pin);

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

    multicore_launch_core1(core1_entry);

    uint32_t min = 0x10000000; // Flash start
    uint32_t max = min + 16*1024*1024 - 1;
    uint32_t count = 0x10000000;
    while (true) {
        pio_sm_put_blocking(pio1, 0, *(uint8_t*)count);
        // pio_sm_put_blocking(pio1, 0, count & 0b10000001);

        //pio1->txf[0] = count; // 1-2 counts , but I prob...MAYBE need the blocking.

        count++;
        if (count > max) {
            count = min;
            pio_sm_put_blocking(pio1, 0, 0x00);
            pio_sm_put_blocking(pio1, 0, 0xFF);
            pio_sm_put_blocking(pio1, 0, 0x40);
            pio_sm_put_blocking(pio1, 0, 0x60);
            pio_sm_put_blocking(pio1, 0, 0xFF);
            pio_sm_put_blocking(pio1, 0, 0x00);
        }
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

void write2ftdi_forever(PIO pio, uint sm, uint offset, uint data_pins_8, uint wr_rd_a0_cs_pin) {
    write2ftdi_program_init(pio, sm, offset, data_pins_8, wr_rd_a0_cs_pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Writing to FTDI on pins %d %d %d\n", data_pins_8, wr_rd_a0_cs_pin);
}
