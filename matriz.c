#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "hardware/adc.h"
#include "martinlib.c"
#include "pins.h"
#include "matriz.h"
#include "pico/stdlib.h"
#include "RP2040.h"

uint32_t trigger_buttons = 0, trigger_enc = 0, done = 0;
uint8_t last_states[24] = {0}, wait = 0, stage = 0;
uint32_t debounce = 10e3, pressed = 0;
uint8_t sfc1 = 0, sfc2 = 0, lfc1 = 0, lfc2 = 0;
uint32_t press1 = 0, press2 = 0;


uint32_t readMatrix(uint8_t buttons[], uint8_t edges[], uint8_t menu){
    switch (stage){
        case 0:
            writePin(COLUMN1,0,1);
            writePin(COLUMN2,0,0);
            writePin(COLUMN3,0,0);
            writePin(COLUMN4,0,0);
            stage = 1;
        break;

        case 1:
            if(wait == 4){
                stage = 2;
                wait = 0;
            }
            else wait++;
        break;
        
        case 2:
            buttons[1] = readPin(ROW1, 0);
            buttons[5] = readPin(ROW2, 0) & !menu;
            buttons[9] = readPin(ROW3, 0);         
                
            writePin(COLUMN1,0,0);
            writePin(COLUMN2,0,1);
            writePin(COLUMN3,0,0);
            writePin(COLUMN4,0,0);
            stage = 3;
        break;

        case 3:
            if(wait == 4){
                stage = 4;
                wait = 0;
            }
            else wait++;
        break;
        
        case 4:          
            buttons[2] = readPin(ROW1, 0) & !menu;
            buttons[6] = readPin(ROW2, 0);
            buttons[10] = readPin(ROW3, 0) & !menu;
        
            writePin(COLUMN1,0,0);
            writePin(COLUMN2,0,0);
            writePin(COLUMN3,0,1);
            writePin(COLUMN4,0,0);
            stage = 5;
        break;

        case 5:
            if(wait == 4){
                stage = 6;
                wait = 0;
            }
            else wait++;
        break;

        case 6:  
            buttons[3] = readPin(ROW1, 0) & !menu;
            buttons[7] = readPin(ROW2, 0);
            buttons[11] = readPin(ROW3, 0) & !menu;
                
            writePin(COLUMN1,0,0);
            writePin(COLUMN2,0,0);
            writePin(COLUMN3,0,0);
            writePin(COLUMN4,0,1);
            stage = 7;
        break;

        case 7:
            if(wait == 4){
                stage = 8;
                wait = 0;
            }
            else wait++;
        break;
        
        case 8:
            buttons[4] = readPin(ROW1, 0) & !menu;
            buttons[8] = readPin(ROW2, 0) & !menu;
            buttons[12] = readPin(ROW3, 0) & !menu;
                
            stage = 0;
            done = 1;
        break;

    }

    if(done){

        if(buttons[1]) pressed |= 1 << 1;
        else pressed &= ~(1<<1);

        if(buttons[2]) pressed |= 1 << 2;
        else pressed &= ~(1<<2);

        if(buttons[3]) pressed |= 1 << 3;
        else pressed &= ~(1<<3);

        if(buttons[4]) pressed |= 1 << 4;
        else pressed &= ~(1<<4);

        if(buttons[5]) pressed |= 1 << 5;
        else pressed &= ~(1<<5);

        if(buttons[6]) pressed |= 1 << 6;
        else pressed &= ~(1<<6);

        if(buttons[7]) pressed |= 1 << 7;
        else pressed &= ~(1<<7);

        if(buttons[8]) pressed |= 1 << 8;
        else pressed &= ~(1<<8);

        if(buttons[9]) pressed |= 1 << 9;
        else pressed &= ~(1<<9);

        if(buttons[10]) pressed |= 1 << 10;
        else pressed &= ~(1<<10);

        if(buttons[11]) pressed |= 1 << 11;
        else pressed &= ~(1<<11);

        if(buttons[12]) pressed |= 1 << 12;
        else pressed &= ~(1<<12);

        done = 0;
    }

    sfc1 = readPin(fc1, 0);
    sfc2 = readPin(fc2, 0);

    if((sfc1 != lfc1) && ((press1 + 10e3) < timer_hw->timelr)){
        press1 = timer_hw->timelr;
        lfc1 = sfc1;
        if(sfc1){
            pressed |= 1 << 13;
        }
        else{
            pressed &= ~(1<<13);
        }
    }
    
    if((sfc2 != lfc2) && ((press2 + 10e3) < timer_hw->timelr)){
        press2 = timer_hw->timelr;
        lfc2 = sfc2;
        if(sfc2){
            pressed |= 1 << 14;
        }
        else{
            pressed &= ~(1<<14);
        }
    }

    return pressed;
}