#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
 
// ---- User-configurable ---------------------------------------------------
#define I2C_PORT       i2c0
#define I2C_SDA_PIN    8
#define I2C_SCL_PIN    9
#define I2C_BAUDRATE   4 * 1000      // 100 kHz (standard mode)
// --------------------------------------------------------------------------
 
// Reserved 7-bit I2C addresses that we skip:
//   0x00-0x07 : reserved (general call, start byte, CBUS, etc.)
//   0x78-0x7F : reserved (10-bit addressing, future use)
static bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}
 
static void do_scan(void) {
    printf("\nI2C Bus Scan (7-bit addresses)\n");
    printf("   ");
    for (int i = 0; i < 16; i++) printf("%3x", i);
    printf("\n");
 
    int found = 0;
 
    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }
 
        int ret;
        uint8_t rxdata;
 
        if (reserved_addr(addr)) {
            // Reserved addresses are never probed
            ret = -1;
        } else {
            // Try a 1-byte read; if device ACKs its address, we count it.
            // Using a short timeout avoids hanging on a stuck bus.
            ret = i2c_read_blocking(I2C_PORT, addr, &rxdata, 1, false);
        }
 
        if (ret < 0) {
            printf(" . ");          // no device / reserved
        } else {
            printf("@%02x", addr);   // device ACKed
            found++;
        }
 
        if (addr % 16 == 15) printf("\n");
    }
 
    printf("\nScan complete. %d device%s found.\n",
           found, found == 1 ? "" : "s");
}
 
int main() {
    stdio_init_all();
 
    // Give USB CDC a moment to enumerate so we don't miss early output.
    sleep_ms(2000);
 
    printf("\n=== Pico I2C Scanner ===\n");
    printf("Port      : I2C%d\n", i2c_hw_index(I2C_PORT));
    printf("SDA pin   : GP%d\n", I2C_SDA_PIN);
    printf("SCL pin   : GP%d\n", I2C_SCL_PIN);
    printf("Baudrate  : %d Hz\n", I2C_BAUDRATE);
 
    // Initialize I2C peripheral
    i2c_init(I2C_PORT, I2C_BAUDRATE);
 
    // Configure the GPIOs for I2C function and enable internal pull-ups
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
 
    // Declare binary info so picotool shows the pin assignment
    bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));
 
    // Re-scan every 5 seconds so devices can be hot-plugged for testing.
    while (true) {
        do_scan();
        printf("\nRescanning in 5 seconds...\n");
        sleep_ms(5000);
    }
 
    return 0;
}