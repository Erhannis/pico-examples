#include <stdio.h>
#include <math.h>
#include <hardware/vreg.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/spi.h"
#include "write2ftdi.pio.h"

void write2ftdi_forever(PIO pio, uint sm, uint offset, uint data_pins_8, uint rd_wr_a0_cs_pin);

int comms_pin = 9;
int data_pins_8 = 22;
int wr_rd_a0_cs_pin = 18;

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

#define DATA_RDY 27

// #define SPI_MODE SPI_MODE0
#define AUTO_INIT 1
#define AUTO_SHUTTER 1
#define BATCH_MODE 1

const uint16_t BS_STARTUP[] = {
  //DUMMY Load sequencer
};

const uint16_t BS_SETTNGS_1[] = {
  0x8100, // Page select 1
  0x5A00, // Adjust 1
  0x8500, // Page select 5
  0x4B00, // Adjust 2
  0x0000  // NOP //THINK Skip nop?
};

// Only for WAFER ID < 13
const uint16_t BS_SETTNGS_2[] = {
  0x8400, // Page select 4
  0x481F, // Adjust 3
  0x8500, // Page select 5
  0x4E01, // Adjust 4
  0x8600, // Page select 6
  0x5162, // Adjust 5
  0x0000  // NOP //DITTO
};

const uint16_t BS_SET_MODE_GIM[] = {
  0x8400, // Page select 4
  0x52C0, // Set modulation selection
  0x5523, // Set 8x8 greyscale imager mode
  0x0000  // NOP //DITTO  
};

const uint16_t BS_SET_MODULATION_FREQUENCY[] = {
  0x8400, // Page select 4
  0x4501, // Set mod. freq. to 10MHz , Is also integration time base
  0x0000  // NOP //DITTO  
};

const uint16_t BS_SET_INTEGRATION_TIME[] = {
  0x8500, // Page select 5
  // Set int. time 1.6384ms
  0x4000, // Integration time multiplier, high byte
  0x4101, // Integration time multiplier, low byte (lowest number = 1)
  0x4204, // Integration length, high byte //RAINY Permit control
  0x43FF, // Integration length, low byte
  0x0000  // NOP //DITTO  
};

const uint16_t BS_START_MEASUREMENT[] = {
  0x8200, // Page select 2
  0x5801, // Set TRIGGER, start measurement
  0x0000  // NOP //DITTO  
};

const uint16_t BS_READ_WAFER_ID[] = {
  0x8700,
  0x3600,
  0x3700,
  0x3800,
  0x3900,
  0x0000
};

const uint16_t BS_READ_2ROW[] = {
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x2C00,
  0x0000
};

static const int spiClk = 16000000;

static inline void cs_select() {
    asm volatile("nop \n nop \n nop"); //DUMMY Why are these here???
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);
    asm volatile("nop \n nop \n nop"); // FIXME
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop"); // FIXME
}

/*
solder oscillators together
  redo pcbs
test max speeds of both modes
*/

uint16_t exchange(uint16_t tx) {
  cs_select();
  uint16_t rx = 0;
  spi_write16_read16_blocking(spi, &tx, &rx, 1);
  cs_deselect();
  return rx;
}

uint16_t print_exchange(uint16_t tx) {
  uint16_t rx = exchange(tx);
  printf("tx/rx %04X/%04X\n", tx, rx);
  return rx;
}

uint16_t exchangeBuffer[256];
void print_exchange_buffer(const uint16_t tx[], int offset, int count) {
  for (int i = 0; i < count; i++) {
    cs_select();
    exchangeBuffer[i] = hspi->transfer16(tx[offset+i]);
    cs_deselect(); // Apparently I have to toggle nss between words
  }
  for (int i = 0; i < count; i++) {
    printf("tx/rx %04X/%04X\n", tx[offset+i], exchangeBuffer[i]);
  }
  println();
}
void exchange_buffer(const uint16_t tx[], int tx_offset, int count, uint16_t rx[], int rx_offset) {
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE));
  for (int i = 0; i < count; i++) {
    cs_select();
    rx[rx_offset+i] = hspi->transfer16(tx[tx_offset+i]);
    cs_deselect(); // Apparently I have to toggle nss between words
  }
}

void wait_ready() {
  println("delaying for ready....");
  for (int i = 0; i < 30; i++) {
    uint16_t rx = print_exchange(0x0000);
    if (rx == 0x0000) {
      println("Ready!");  
      println();
      return;
    }
    sleep_ms(100);
  }
  println("Not ready... :(");  
  println();
}

void initEpc611() {
    printf("Size of frame: %d\n", 8*8*2);

#if !defined(spi_default) || !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || !defined(PICO_DEFAULT_SPI_RX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)
#warning spi example requires SPI pins
    puts("Default SPI pins were not defined");
#endif

    spi_init(spi_default, spiClk);

    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "SPI CS"));

    gpio_init(DATA_RDY);
    gpio_set_dir(DATA_RDY, GPIO_IN);
    // while (true) {
    //     //gpio_get
    //     // gpio_put(0, true);
    // }

    wait_ready();

    if (AUTO_INIT) {
        processCommand("init");
    }
}

int main() {
    stdio_init_all();

    uint vco, postdiv1, postdiv2;
    int maxf = 0;

    int target = 280000;
    
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
    write2ftdi_forever(pio1, 0, cf_offset, data_pins_8, wr_rd_a0_cs_pin);

    pio1->sm[0].shiftctrl = 
        (1u << PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_LSB) |
        (1u << PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_LSB) |
        0;

    uint32_t count = 0;
    while (true) {
        pio_sm_put_blocking(pio1, 0, count);

        //pio1->txf[0] = count; // 1-2 counts , but I prob...MAYBE need the blocking.

        count++;
    }
}

void write2ftdi_forever(PIO pio, uint sm, uint offset, uint data_pins_8, uint wr_rd_a0_cs_pin) {
    write2ftdi_program_init(pio, sm, offset, data_pins_8, wr_rd_a0_cs_pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Writing to FTDI on pins %d %d %d\n", data_pins_8, wr_rd_a0_cs_pin);
}
