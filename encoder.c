#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "RP2040.h"
#include "martinlib.h"
#include "pins.h"
#include "encoder.h"

extern uint8_t encoder1, encoder2, encoder3, encoder4, encoder5;

// Interrupt handler for the encoder A pin
/*
void read_encoders(uint8_t buttons[24]) {
    uint8_t encoder1a = readPin(RE_LEFT_A, 0);
    uint8_t encoder1b = readPin(RE_LEFT_B, 0);

    if(timer_hw->timelr > (trigger_encoder + 10e3)){

        trigger_encoder = timer_hw->timelr;
        if(encoder1a != last_encoder1a){
            // Determine the direction based on B pin
            if(encoder1b != encoder1a){
                if(encoder1 < 100){
                    encoder1++;  // Clockwise rotation
                    buttons[15] = 1;
                }
            } 
            else{
                if(encoder1 > 0){
                    encoder1--;  // Counter-clockwise rotation
                    buttons[16] = 1;
                }
            }
        }
        last_encoder1a = encoder1a;
    }

    if(timer_hw->timelr > (trigger_encoder + 10e3)){
        trigger_encoder = timer_hw->timelr;
        if(encoder1b != last_encoder1b) {
            // Determine the direction based on A pin
            if(encoder1b != encoder1a){
                if(encoder1 > 0){
                    encoder1--;  // Clockwise rotation
                    buttons[16] = 1;
                }
            } 
            else{
                if(encoder1 < 100){
                    encoder1++;  // Counter-clockwise rotation
                    buttons[15] = 1;
                }
            }
        }
        
        last_encoder1b = encoder1b;
    }

    uint8_t encoder2a = readPin(RE_C_LEFT_A, 0);
    uint8_t encoder2b = readPin(RE_C_LEFT_B, 0);
    
    
    if(timer_hw->timelr > (trigger_encoder + 10e3)){
        trigger_encoder = timer_hw->timelr;

        if(encoder2a != last_encoder2a){
            // Determine the direction based on B pin
            if(encoder2b != encoder2a){
                if(encoder2 < 100){
                    encoder2++;  // Clockwise rotation
                    buttons[17] = 1;
                }
            } 
            else{
                if(encoder2 > 0){
                    encoder2--;  // Counter-clockwise rotation
                    buttons[18] = 1;
                }
            }
        }
        
        last_encoder2a = encoder2a;
    }

    if(timer_hw->timelr > (trigger_encoder + 10e3)){
        trigger_encoder = timer_hw->timelr;
        if(encoder2b != last_encoder2b){
            // Determine the direction based on A pin
            if(encoder2b != encoder2a){
                if(encoder2 > 0){
                    encoder2--;  // Clockwise rotation
                    buttons[18] = 1;
                }
            } 
            else{
                if(encoder2 < 100){
                    encoder2++;  // Counter-clockwise rotation
                    buttons[17] = 1;
                }
            }
        }
        
        last_encoder2b = encoder2b;
    }

    uint8_t encoder3a = readPin(RE_MID_A, 0);
    uint8_t encoder3b = readPin(RE_MID_B, 0);
    
    if(timer_hw->timelr > (trigger_encoder + 10e3)){
        trigger_encoder = timer_hw->timelr;
        if(encoder3a != last_encoder3a){
            // Determine the direction based on B pin
            if(encoder3b != encoder3a){
                if(encoder3 < 100){
                    encoder3++;  // Clockwise rotation
                    buttons[19] = 1;
                }
            } 
            else{
                if(encoder3 > 0){
                    encoder3--;  // Counter-clockwise rotation
                    buttons[20] = 0;
                }
            }
        }
        
        last_encoder3a = encoder3a;
    }

    if(timer_hw->timelr > (trigger_encoder + 10e3)){
        trigger_encoder = timer_hw->timelr;
        if(encoder3b != last_encoder3b){
            // Determine the direction based on A pin
            if(encoder3b != encoder3a){
                if(encoder3 > 0){
                    encoder3--;  // Clockwise rotation
                    buttons[20] = 1;
                }
            } 
            else{
                if(encoder3 < 100){
                    encoder3++;  // Counter-clockwise rotation
                    buttons[19] = 1;
                }
            }
        }
        
        last_encoder3b = encoder3b;
    }

    uint8_t encoder4a = readPin(RE_C_RIGHT_A, 0);
    uint8_t encoder4b = readPin(RE_C_RIGHT_B, 0);
    
    if(timer_hw->timelr > (trigger_encoder + 10e3)){
        trigger_encoder = timer_hw->timelr;
        if(encoder4a != last_encoder4a){
            // Determine the direction based on B pin
            if(encoder4b != encoder4a){
                if(encoder4 < 100){
                    encoder4++;  // Clockwise rotation
                    buttons[21] = 1;
                }
            } 
            else{
                if(encoder4 > 0){
                    encoder4--;  // Counter-clockwise rotation
                    buttons[22] = 1;
                }
            }
        }
        
        last_encoder4a = encoder4a;
    }

    if(timer_hw->timelr > (trigger_encoder + 10e3)){
        trigger_encoder = timer_hw->timelr;
        if(encoder4b != last_encoder4b){
            // Determine the direction based on A pin
            if(encoder4b != encoder4a){
                if(encoder4 > 0){
                    encoder4--;  // Clockwise rotation
                    buttons[22];
                }
            } else {
                if(encoder4 < 100){
                    encoder4++;  // Counter-clockwise rotation
                    buttons[21] = 1;
                }
            }
        }
        
        last_encoder4b = encoder4b;
    }
    uint8_t encoder5a = readPin(RE_RIGHT_A, 0);
    uint8_t encoder5b = readPin(RE_RIGHT_B, 0);
    
    if(timer_hw->timelr > (trigger_encoder + 10e3)){
        trigger_encoder = timer_hw->timelr;
        if(encoder5a != last_encoder5a){
            // Determine the direction based on B pin
            if(encoder5b != encoder5a){
                if(encoder5 < 100){
                    encoder5++;  // Clockwise rotation
                    buttons[23] = 1;
                }
            } 
            else{
                if(encoder5 > 0){
                    encoder5--;  // Counter-clockwise rotation
                    buttons[24] = 1;
                }
            }
        }
        
        last_encoder5a = encoder5a;
    }

    if(timer_hw->timelr > (trigger_encoder + 10e3)){
        if(encoder5b != last_encoder5b){
            trigger_encoder = timer_hw->timelr;
            // Determine the direction based on A pin
            if(encoder5b != encoder5a){
                if(encoder5 > 0){
                    encoder5--;  // Clockwise rotation
                    buttons[24] = 1;
                }
            } 
            else{
                if(encoder5 < 100){
                    encoder5++;  // Counter-clockwise rotation
                    buttons[23] = 1;
                }
            }
        }
        
        last_encoder5b = encoder5b;
    }
}
*/

void read_encoders(uint8_t buttons[]){
    static uint8_t state1 = 0, state2 = 0, state3 = 0, state4 = 0, state5 = 0;

    uint8_t A1_state = readPin(RE_LEFT_A, 0);
    uint8_t B1_state = readPin(RE_LEFT_B, 0);

    uint8_t A2_state = readPin(RE_C_LEFT_A, 0);
    uint8_t B2_state = readPin(RE_C_LEFT_B, 0);

    uint8_t A3_state = readPin(RE_MID_A, 0);
    uint8_t B3_state = readPin(RE_MID_B, 0);

    uint8_t A4_state = readPin(RE_C_RIGHT_A, 0);
    uint8_t B4_state = readPin(RE_C_RIGHT_B, 0);

    uint8_t A5_state = readPin(RE_RIGHT_A, 0);
    uint8_t B5_state = readPin(RE_RIGHT_B, 0);

    switch (state1) {
        case 0:                         // Idle state, encoder not turning
            if (!A1_state){             // Turn clockwise and CLK goes low first
                state1 = 1;
            } else if (!B1_state) {      // Turn anticlockwise and DT goes low first
                state1 = 4;
            }
        break;
        // Clockwise rotation
        case 1:                     
            if (!B1_state) {             // Continue clockwise and DT will go low after CLK
                state1 = 2;
            } 
        break;
        case 2:
            if (A1_state) {             // Turn further and CLK will go high first
                state1 = 3;
            }
        break;
        case 3:
            if (A1_state && B1_state) {  // Both CLK and DT now high as the encoder completes one step clockwise
                state1 = 0;
                if(encoder1 < 100)
                    encoder1++;

                buttons[15] = 1;
            }
        break;
        // Anticlockwise rotation
        case 4:                         // As for clockwise but with CLK and DT reversed
            if (!A1_state) {
                state1 = 5;
            }
        break;
        case 5:
            if (B1_state) {
                state1 = 6;
            }
        break;
        case 6:
            if (A1_state && B1_state) {
                state1 = 0;

                if(encoder1 > 0)
                    encoder1--;
                
                buttons[16] = 1;
            }
        break; 
    }

    switch (state2) {
        case 0:                         // Idle state, encoder not turning
            if (!A2_state){             // Turn clockwise and CLK goes low first
                state2 = 1;
            } else if (!B2_state) {      // Turn anticlockwise and DT goes low first
                state2 = 4;
            }
        break;
        // Clockwise rotation
        case 1:                     
            if (!B2_state) {             // Continue clockwise and DT will go low after CLK
                state2 = 2;
            } 
        break;
        case 2:
            if (A2_state) {             // Turn further and CLK will go high first
                state2 = 3;
            }
        break;
        case 3:
            if (A2_state && B2_state) {  // Both CLK and DT now high as the encoder completes one step clockwise
                state2 = 0;
                if(encoder2 < 100)
                    encoder2++;

                buttons[17] = 1;
            }
        break;
        // Anticlockwise rotation
        case 4:                         // As for clockwise but with CLK and DT reversed
            if (!A2_state) {
                state2 = 5;
            }
        break;
        case 5:
            if (B2_state) {
                state2 = 6;
            }
        break;
        case 6:
            if (A2_state && B2_state) {
                state2 = 0;
                if(encoder2 > 0)
                encoder2--;
                buttons[18] = 1;
            }
        break; 
    }

    switch (state3) {
        case 0:                         // Idle state, encoder not turning
            if (!A3_state){             // Turn clockwise and CLK goes low first
                state3 = 1;
            } else if (!B3_state) {      // Turn anticlockwise and DT goes low first
                state3 = 4;
            }
        break;
        // Clockwise rotation
        case 1:                     
            if (!B3_state) {             // Continue clockwise and DT will go low after CLK
                state3 = 2;
            } 
        break;
        case 2:
            if (A3_state) {             // Turn further and CLK will go high first
                state3 = 3;
            }
        break;
        case 3:
            if (A3_state && B3_state) {  // Both CLK and DT now high as the encoder completes one step clockwise
                state3 = 0;

                if(encoder3 < 100)
                    encoder3++;
                
            }
        break;
        // Anticlockwise rotation
        case 4:                         // As for clockwise but with CLK and DT reversed
            if (!A3_state) {
                state3 = 5;
            }
        break;
        case 5:
            if (B3_state) {
                state3 = 6;
            }
        break;
        case 6:
            if (A3_state && B3_state) {
                state3 = 0;
                if(encoder3 > 0)
                    encoder3--;


            }
        break; 
    }

    switch (state4) {
        case 0:                         // Idle state, encoder not turning
            if (!A4_state){             // Turn clockwise and CLK goes low first
                state4 = 1;
            } else if (!B4_state) {      // Turn anticlockwise and DT goes low first
                state4 = 4;
            }
        break;
        // Clockwise rotation
        case 1:                     
            if (!B4_state) {             // Continue clockwise and DT will go low after CLK
                state4 = 2;
            } 
        break;
        case 2:
            if (A4_state) {             // Turn further and CLK will go high first
                state4 = 3;
            }
        break;
        case 3:
            if (A4_state && B4_state) {  // Both CLK and DT now high as the encoder completes one step clockwise
                state4 = 0;
                if(encoder4 < 100)
                    encoder4++;
                
                buttons[21] = 1;
            }
        break;
        // Anticlockwise rotation
        case 4:                         // As for clockwise but with CLK and DT reversed
            if (!A4_state) {
                state4 = 5;
            }
        break;
        case 5:
            if (B4_state) {
                state4 = 6;
            }
        break;
        case 6:
            if (A4_state && B4_state) {
                state4 = 0;
                if(encoder4 > 0)
                    encoder4--;
                
                buttons[22] = 1;
            }
        break; 
    }

    switch (state5) {
        case 0:                         // Idle state, encoder not turning
            if (!A5_state){             // Turn clockwise and CLK goes low first
                state5 = 1;
            } else if (!B5_state) {      // Turn anticlockwise and DT goes low first
                state5 = 4;
            }
        break;
        // Clockwise rotation
        case 1:                     
            if (!B5_state) {             // Continue clockwise and DT will go low after CLK
                state5 = 2;
            } 
        break;
        case 2:
            if (A5_state) {             // Turn further and CLK will go high first
                state5 = 3;
            }
        break;
        case 3:
            if (A5_state && B5_state) {  // Both CLK and DT now high as the encoder completes one step clockwise
                state5 = 0;
                if(encoder5 < 100)
                    encoder5++;

                buttons[23] = 1;
            }
        break;
        // Anticlockwise rotation
        case 4:                         // As for clockwise but with CLK and DT reversed
            if (!A5_state) {
                state5 = 5;
            }
        break;
        case 5:
            if (B5_state) {
                state5 = 6;
            }
        break;
        case 6:
            if (A5_state && B5_state) {
                state5 = 0;
                if(encoder5 > 0)
                encoder5--;
                buttons[23] = 1;
            }
        break; 
    }


}