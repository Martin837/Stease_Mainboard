#include "i2c_comm.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "RP2040.h"
#include <hardware/i2c.h>
#include <pico/stdlib.h>
#include "hardware/i2c.h"


void i2c_comm(uint8_t command, uint8_t *address, uint8_t message[], uint8_t buffer[]){
    if(command == 'W'){
        i2c_write_blocking(i2c1, 0x17, message, 6, false);
    }
    if(command == 'R'){
        i2c_write_blocking(i2c1, 0x17, address, 1, true);
        i2c_read_blocking(i2c1, 0x17, buffer, 9, true);
    }
}