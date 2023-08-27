#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

#define DATA_RDY 6

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

uint16_t __time_critical_func(exchange)(uint16_t tx) {
  cs_select();
  uint16_t rx = 0;
  spi_write16_read16_blocking(spi_default, &tx, &rx, 1); //LEAK Could maybe be faster to excerpt code from INSIDE this function
  cs_deselect();
  return rx;
}

// Not time critical, I guess, given printfs...
uint16_t print_exchange(uint16_t tx) {
  uint16_t rx = exchange(tx);
  printf("tx/rx %04X/%04X\n", tx, rx);
  return rx;
}

uint16_t exchangeBuffer[256];
// Also not time critical
void print_exchange_buffer(const uint16_t tx[], int offset, int count) {
  for (int i = 0; i < count; i++) {
    cs_select();
    spi_write16_read16_blocking(spi_default, &tx[offset+i], &exchangeBuffer[i], 1); //LEAK Could maybe be faster to excerpt code from INSIDE this function
    cs_deselect(); // Apparently I have to toggle nss between words
  }
  for (int i = 0; i < count; i++) {
    printf("tx/rx %04X/%04X\n", tx[offset+i], exchangeBuffer[i]);
  }
  printf("\n");
}
void __time_critical_func(exchange_buffer)(const uint16_t tx[], int tx_offset, int count, uint16_t rx[], int rx_offset) {
  for (int i = 0; i < count; i++) {
    cs_select();
    spi_write16_read16_blocking(spi_default, &tx[tx_offset+i], &rx[rx_offset+i], 1); //LEAK Could maybe be faster to excerpt code from INSIDE this function
    cs_deselect(); // Apparently I have to toggle nss between words
  }
}

void wait_ready() {
  printf("delaying for ready....\n");
  for (int i = 0; i < 30; i++) {
    uint16_t rx = print_exchange(0x0000);
    if (rx == 0x0000) {
      printf("Ready!\n");
      printf("\n");
      return;
    }
    sleep_ms(100);
  }
  printf("Not ready... :(\n");  
  printf("\n");
}

void read_wafer_id() {
  printf("Reading wafer id...\n");
  print_exchange_buffer(BS_READ_WAFER_ID, 0, sizeof(BS_READ_WAFER_ID) / sizeof(BS_READ_WAFER_ID[0]));
  printf("\n");
}

void processCommand(char* command) {
  // Compare the received command with predefined commands
  if (!strcmp(command, "h") || !strcmp(command, "help")) {
    printf("h/help, id, t####, init, dr, c/s/shutter\n"); //PERIODIC Keep up to date
  } else if (!strcmp(command, "id")) {
    read_wafer_id();
  } else if (strlen(command) == 5 && command[0] == 't') {
    char *endptr;
    uint16_t c = strtol(&command[1], &endptr, 16);
    print_exchange(c);
  } else if (!strcmp(command, "init")) {
    //DUMMY Probably do this automatically
    //DUMMY Load sequencer
    print_exchange_buffer(BS_SETTNGS_1, 0, sizeof(BS_SETTNGS_1) / sizeof(BS_SETTNGS_1[0]));
    // skip settings 2, at least for my chips //DUMMY will we ever need to deal with wafer < 13?
    print_exchange_buffer(BS_SET_MODE_GIM, 0, sizeof(BS_SET_MODE_GIM) / sizeof(BS_SET_MODE_GIM[0]));
    print_exchange_buffer(BS_SET_MODULATION_FREQUENCY, 0, sizeof(BS_SET_MODULATION_FREQUENCY) / sizeof(BS_SET_MODULATION_FREQUENCY[0]));
    print_exchange_buffer(BS_SET_INTEGRATION_TIME, 0, sizeof(BS_SET_INTEGRATION_TIME) / sizeof(BS_SET_INTEGRATION_TIME[0]));
  } else if (!strcmp(command, "c") || !strcmp(command, "s") || !strcmp(command, "shutter")) {
    if (!BATCH_MODE) {
      print_exchange(0x8200);
      print_exchange(0x5801); // Set trigger
    } else {
      exchange(0x8200);
      exchange(0x5801); // Set trigger
    }
    
    int16_t frame[8*8];
    uint16_t row_buf[24+1];
    uint8_t row2[24];

    // Read frame
    pio_sm_put_blocking(pio1, 0, 'F');
    pio_sm_put_blocking(pio1, 0, 'R');
    pio_sm_put_blocking(pio1, 0, '\n');
    unsigned long delay = 0;
    absolute_time_t start = get_absolute_time();
    for (int i = 3; i >= 0; i--) {
      // Wait for data ready
      absolute_time_t ta = get_absolute_time();
      while (!gpio_get(DATA_RDY));
      absolute_time_t tb = get_absolute_time();
      delay += absolute_time_diff_us(ta, tb);

      exchange_buffer(BS_READ_2ROW, 0, sizeof(BS_READ_2ROW) / sizeof(BS_READ_2ROW[0]), row_buf, 0);
      //DUMMY I'm dropping the least significant 4 bits
      for (int j = 0; j < 4; j++) {
        int k = j*3;
        uint8_t b1 = (uint8_t)(row_buf[k+1] & 0x00FF);
        uint8_t b2 = (uint8_t)(row_buf[k+2] & 0x00FF);
        uint8_t b3 = (uint8_t)(row_buf[k+3] & 0x00FF);
        uint16_t combinedInt1 = ((uint16_t)b1 << 4) | (b2 >> 4);
        uint16_t combinedInt2 = (b3 << 4) | (b2 & 0x0F);
        frame[((  i)*8)+(j*2  )] = (int16_t)(combinedInt1 << 4) >> 4;
        frame[((  i)*8)+(j*2+1)] = (int16_t)(combinedInt2 << 4) >> 4;
      }
      for (int j = 0; j < 4; j++) {
        int k = (j+4)*3;
        uint8_t b1 = (uint8_t)(row_buf[k+1] & 0x00FF);
        uint8_t b2 = (uint8_t)(row_buf[k+2] & 0x00FF);
        uint8_t b3 = (uint8_t)(row_buf[k+3] & 0x00FF);
        uint16_t combinedInt1 = ((uint16_t)b1 << 4) | (b2 >> 4);
        uint16_t combinedInt2 = (b3 << 4) | (b2 & 0x0F);
        frame[((7-i)*8)+(j*2  )] = (int16_t)(combinedInt1 << 4) >> 4;
        frame[((7-i)*8)+(j*2+1)] = (int16_t)(combinedInt2 << 4) >> 4;
      }
    }
    absolute_time_t stop = get_absolute_time();
    if (!BATCH_MODE) {
      printf("micros elapsed: %ld\n", absolute_time_diff_us(start, stop));
      printf("micros delay: %ld\n", delay);
      printf("\n");
      // printFrameScaled(frame);
      printf("\n");
    } else {
      //stdio_uart_out_chars((char*)frame, sizeof(frame));
      int sf = sizeof(frame);
      //printf("%d", sf);
      for (int i = 0; i < sf; i++) {
          pio_sm_put_blocking(pio1, 0, ((uint8_t*)frame)[i]);
      }
      // uart_write_blocking(uart_default, (uint8_t*)frame, sizeof(frame));
      // Serial.write((char*)frame, sizeof(frame));
    }
  } else if (!strcmp(command, "dr")) {
    int dataRdy = gpio_get(DATA_RDY);
    printf("dataRdy: %d\n", dataRdy);
  } else {
    printf("Invalid command\n"); // Command not recognized
  }
}

void initEpc611() {
    printf("Size of frame: %d\n", 8*8*2);

#if !defined(spi_default) || !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || !defined(PICO_DEFAULT_SPI_RX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)
#warning spi example requires SPI pins
    puts("Default SPI pins were not defined");
#endif

    // If the params are wrong, see spi_set_format
    spi_init(spi_default, spiClk);
    spi_set_format(spi_default, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_LSB_FIRST);

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
    // stdio_init_all();

    uint vco, postdiv1, postdiv2;
    int maxf = 0;

    int target = 280000;
    
    int nearest = 0;
    for (int f = 0; f < 1000000; f += 1000) {
        if (check_sys_clock_khz(f, &vco, &postdiv1, &postdiv2)) {
            // printf("%d\n", f);
            maxf = f;
            if (abs(target-f) < abs(target-nearest)) {
                nearest = f;
            }
        }
    }
    // printf("max: %d\n", maxf);
    // printf("nearest: %d\n", nearest);
    // printf("nearest: %d\n", nearest);
    // printf("nearest: %d\n", nearest);
    // printf("nearest: %d\n", nearest);

    //vreg_set_voltage(VREG_VOLTAGE_1_20);
    set_sys_clock_khz(nearest, true);

    stdio_init_all();

    printf("\n\nStartup!\n\n");

    printf("max: %d\n", maxf);
    printf("nearest: %d\n", nearest);

    initEpc611();
    
    uint cf_offset = pio_add_program(pio1, &write2ftdi_program);
    printf("Loaded program at %d\n", cf_offset);
    write2ftdi_forever(pio1, 0, cf_offset, data_pins_8, wr_rd_a0_cs_pin);

    pio1->sm[0].shiftctrl = 
        (1u << PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_LSB) |
        (1u << PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_LSB) |
        0;

    // if (AUTO_SHUTTER) {
    //     // long t = micros();
    //     // long diff = t - timer;
    //     // Serial.printf("out: %ld", diff);
    //     // timer = t;

    //     while (true) {
    //         processCommand("s");
    //     }

    //     // t = micros();
    //     // diff = t - timer;
    //     // Serial.printf("in: %ld", diff);
    //     // timer = t;
    // } else {
    //     readSerialCommand();
    // }
    //delay(10);

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
