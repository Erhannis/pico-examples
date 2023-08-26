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
#include "hardware/spi.h"
#include "write2ftdi.pio.h"

void write2ftdi_forever(PIO pio, uint sm, uint offset, uint data_pins_8, uint rd_wr_a0_cs_pin);

int comms_pin = 9;
int data_pins_8 = 22;
int wr_rd_pin = 18;

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

    uint vco, postdiv1, postdiv2;
    int maxf = 0;

    int target = 242000;
    
    int i1 = 0;
    int d1 = abs(target-i1);
    int i2 = 0;
    int d2 = abs(target-i2);
    int i3 = 0;
    int d3 = abs(target-i3);
    for (int f = 0; f < 1000000; f += 1000) {
        if (check_sys_clock_khz(f, &vco, &postdiv1, &postdiv2)) {
            printf("%d\n", f);
            maxf = f;
            int d = abs(target-f);
            //SHAME Probably not optimal
            if (d < d3) {
                i3 = f;
                d3 = d;
            }
            if (d < d2) {
                i3 = i2;
                d3 = d2;
                i2 = f;
                d2 = d;
            }
            if (d < d1) {
                i3 = i2;
                d3 = d2;
                i2 = i1;
                d2 = d1;
                i1 = f;
                d1 = d;
            }
        }
    }
    printf("max: %d\n", maxf);
    printf("nearest 3: %d %d %d\n", i1, i2, i3);

    //vreg_set_voltage(VREG_VOLTAGE_1_20);
    set_sys_clock_khz(i1, true);

    uint cf_offset = pio_add_program(pio1, &write2ftdi_program);
    printf("Loaded program at %d\n", cf_offset);
    write2ftdi_forever(pio1, 0, cf_offset, data_pins_8, wr_rd_pin);

    pio1->sm[0].shiftctrl = 
        (1u << PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_LSB) |
        (1u << PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_LSB) |
        0;

    uint32_t count = 0;
    while (true) {
        pio_sm_put_blocking(pio1, 0, 512);
        pio_sm_put_blocking(pio1, 0, 256);
    
        // for (int i = 0; i < 512; i++) {
        //     pio_sm_put_blocking(pio1, 0, count);
        //     //pio1->txf[0] = count; // 1-2 counts , but I prob...MAYBE need the blocking.

        //     count++;
        // }
    }
}

void write2ftdi_forever(PIO pio, uint sm, uint offset, uint data_pins_8, uint wr_rd_pin) {
    write2ftdi_program_init(pio, sm, offset, data_pins_8, wr_rd_pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Writing to FTDI on pins %d %d %d\n", data_pins_8, wr_rd_pin);
}
