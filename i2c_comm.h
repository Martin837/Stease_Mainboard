#include <stdio.h>
#include "pico/stdlib.h"
#include "RP2040.h"
#include <hardware/i2c.h>
#include <pico/stdlib.h>
#include "hardware/i2c.h"
#include "string.h"

//message = mem adress + data(byte)
//data = angle angle angle led torque
void i2c_comm(uint8_t command, uint8_t *address, uint8_t message[], uint8_t buffer[]);