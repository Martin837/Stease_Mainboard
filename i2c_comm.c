/**
 * I2C Communication Handler
 * -----------------------
 * Manages I2C bus communication with peripheral devices
 */

#include "i2c_comm.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "RP2040.h"
#include <hardware/i2c.h"

void i2c_comm(uint8_t command, uint8_t *address, uint8_t message[], uint8_t buffer[]) {
    /* Handle I2C communication operations
     * Parameters:
     *   command: 'W' for write, 'R' for read
     *   address: Target device register address
     *   message: Data to write (for write operations)
     *   buffer: Buffer to store read data (for read operations)
     */
    if(command == 'W') {
        // Write operation - send 6 bytes to device 0x17
        i2c_write_blocking(i2c1, 0x17, message, 6, false);
    }
    if(command == 'R') {
        // Read operation:
        // 1. Write register address
        // 2. Read 9 bytes from device
        i2c_write_blocking(i2c1, 0x17, address, 1, true);
        i2c_read_blocking(i2c1, 0x17, buffer, 9, true);
    }
}