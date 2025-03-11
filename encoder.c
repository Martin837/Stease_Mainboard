/**
 * Rotary Encoder Handler
 * ---------------------
 * Manages multiple rotary encoders for input control
 */

#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "RP2040.h"
#include "martinlib.h"
#include "pins.h"
#include "encoder.h"

/* External Encoder Position Variables */
extern uint8_t encoder1;        // Left encoder position
extern uint8_t encoder2;        // Center-left encoder position
extern uint8_t encoder3;        // Middle encoder position
extern uint8_t encoder4;        // Center-right encoder position
extern uint8_t encoder5;        // Right encoder position

void read_encoders(uint8_t buttons[]) {
    /* Read all encoder states and update positions
     * Parameters:
     *   buttons[]: Array containing button states
     * 
     * Uses quadrature decoding to determine rotation direction:
     * - Clockwise: A leads B
     * - Counter-clockwise: B leads A
     */
    
    // Read encoder 1 (Left)
    uint8_t encoder1a = readPin(RE_LEFT_A, 0);
    uint8_t encoder1b = readPin(RE_LEFT_B, 0);
    
    // Debounce check
    if(timer_hw->timelr > (trigger_encoder + 10e3)) {
        // Update position based on quadrature state
        // ...existing code...
    }
    
    // Read encoder 2 (Center-left)
    uint8_t encoder2a = readPin(RE_C_LEFT_A, 0);
    uint8_t encoder2b = readPin(RE_C_LEFT_B, 0);
    // ...existing code...

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