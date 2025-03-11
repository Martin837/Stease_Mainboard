/**
 * Main Board Control System
 * ------------------------
 * This file implements the main control logic for an input device with:
 * - MPU6050 motion sensing
 * - Multiple rotary encoders
 * - LCD display
 * - Button matrix
 * - USB HID interface
 */

#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "RP2040.h"
#include <string.h>
#include <math.h>
#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "callbacks.h"
#include "hardware/adc.h"
#include "martinlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "mpu.h"
#include "display.h"
#include "pins.h"
#include "matriz.h"
#include "encoder.h"

/* System Constants */
#define pi 3.14159265358979323
#define MPU6050_adress 0x68    // I2C address for motion sensor

/* Function Prototypes */
void beginUart(void);          // Initialize UART communication
void Recibe_car1(void);        // UART receive interrupt handler
void update_angle(void);       // Update angle display on LCD
void updateButtons(uint8_t);   // Handle button state changes
void confi(void);             // Configuration mode handler
void memo(void);              // Memory operations handler
void cali(void);              // Calibration mode handler
void LCD_menu(void);          // LCD menu system handler

/* Global State Variables */
// Port Control
uint8_t port_avail = 1;        // Communication port availability flag

// HID Interface Control
uint32_t hid_buttons = 0;      // Current HID button states
uint32_t last_hid_buttons = 0; // Previous HID button states
uint32_t trigger = 0;          // Button event timestamp
uint32_t db_time = 10e3;       // Debounce timing (10ms)

// Communication Control
int tiempo_envio_trama = 0;    // Frame transmission timer
uint8_t matriz = 0;            // Matrix scanning state

// Interface Text Resources
char *button_strings[] = {     
    " ",                    // Empty string for no button
    "buttons 1",           // Button 1 label
    "buttons 2",           // Button 2 label
    "buttons 3",           // Button 3 label
    "buttons 4",           // Button 4 label
    "buttons 5",           // Button 5 label
    "buttons 6",           // Button 6 label
    "buttons 7",           // Button 7 label
    "buttons 8",           // Button 8 label
    "buttons 9",           // Button 9 label
    "buttons 10",          // Button 10 label
    "buttons 11",          // Button 11 label
    "buttons 12",          // Button 12 label
    "buttons 13",          // Button 13 label
    "buttons 14",          // Button 14 label
    "buttons 15",          // Button 15 label
    "buttons 16",          // Button 16 label
    "buttons 17",          // Button 17 label
    "buttons 18",          // Button 18 label
    "buttons 19",          // Button 19 label
    "buttons 20",          // Button 20 label
    "buttons 21",          // Button 21 label
    "buttons 22"           // Button 22 label
};

// Motion Tracking
int64_t angle[6] = {0};       // Angle measurement history
int64_t f_angle = 0;          // Filtered angle value

// Display Control
uint8_t reset = 0;            // Display reset flag
uint8_t edge = 0;             // Button edge detection
uint8_t change_lcd = 0;       // Display update needed flag
uint32_t last_lcd = 0;        // Last display update time

// Sensor Data
int16_t xaxis;                // X-axis acceleration
int16_t yaxis;                // Y-axis acceleration
int16_t zaxis;                // Z-axis acceleration

// HID Report Data
uint16_t x = 0;               // X position for HID report
int8_t sx = 0;                // Signed X value
uint32_t last = 0;            // Last report timestamp

// Encoder States
uint8_t encoder1A = 0;        // Encoder 1 channel A state
uint8_t encoder1B = 0;        // Encoder 1 channel B state
uint8_t encoder2A = 0;        // Encoder 2 channel A state
uint8_t encoder2B = 0;        // Encoder 2 channel B state
uint8_t encoder3A = 0;        // Encoder 3 channel A state
uint8_t encoder3B = 0;        // Encoder 3 channel B state
uint8_t encoder4A = 0;        // Encoder 4 channel A state
uint8_t encoder4B = 0;        // Encoder 4 channel B state
uint8_t encoder5A = 0;        // Encoder 5 channel A state
uint8_t encoder5B = 0;        // Encoder 5 channel B state
uint8_t ultimo1, ultimo2, ultimo3, ultimo4, ultimo5; // Encoder
uint8_t l_encoder1 = 0, l_encoder2 = 0, l_encoder3 = 0, l_encoder4 = 0, l_encoder5 = 0; // Encoder
uint8_t triger_encoder;                   

// Button Edge Detection
uint8_t edges[15];            // Button edge state array

uint8_t veces_con = 0, modo_conf = 0;                                                                  // Confi
uint32_t tiempo_anterior_con = 0, tiempo_actual_con = 0, actual_con, anterior_con = 0, apreto_con = 0; // Confi
uint8_t MARTIN = 0;                                                                                    // fuente de datos Confi

int offset = 0;

uint8_t veces_cal = 0, modo_cal = 0;                                                       // Calibración
uint32_t tiempo_anterior_cal = 0, tiempo_actual_cal = 0, actual_cal, anterior_cal = 0, apreto_cal = 0; // Calibración

uint8_t start_recording = 0, stop_recording = 0, start_streaming = 0, stop_streaming = 0, replay = 0;  // Mem
int mandar_trama = 0, num = 0;                                                                         // Mem
char trama[50] = "!Saaa-Bn-Bn-Enxxx-Mx";                                                               // Mem
uint32_t tiempo_anterior = 0, tiempo_actual = 0, diferencia = 0;                                       // Mem
uint8_t veces_mem = 0;                                                                                 // Confi
uint32_t tiempo_anterior_mem = 0, tiempo_actual_mem = 0, actual_mem, anterior_mem = 0, apreto_mem = 0; // Confi

uint32_t last_update = 0; // LCD
// menus
uint8_t menu = 0, modo_mem = 0, datos = 0, torque = 50, last_datos = 1, last_torque = 0; // LCD
// menu: is a menu active? if it is then buttons can't be displayed.
uint8_t linea = 0, change = 1;                                       // modo mem //LCD
char *mem_strings[] = {"Inicio", "Fin", "Repeticion", "Salir", "<"}; // LCD

uint32_t suelto_con = 0, total_con = 0, suelto_cal = 0, total_cal = 0, suelto_mem = 0, total_mem = 0;

uint8_t samples = 0;

char caracter_rec;
uint8_t indice=0,recibo[50];

char caracter_rec1;
uint8_t indice1=0,recibo1[50];

uint8_t encoders[6] = {0}, n=0, arranca = 0, termina = 0;

int angle1 = 0;

uint32_t press = 0;

uint8_t buttons[25] = {0};

uint8_t encoder1 = 0, encoder2 = 0, encoder3 = 0, encoder4 = 0, encoder5 = 0;

uint8_t i = 0;

//*------------- MAIN -------------*/
int main(void)
{

  stdio_init_all();
  SystemCoreClockUpdate();
  board_init();
  begin_systick(124999);
  beginUart();

  //*-------------Pin Configurations-------------*/
  pinConfig(RE_LEFT_A, 5, 0b11100111);
  pinConfig(RE_LEFT_B, 5, 0b11100111);
  pinConfig(RE_C_LEFT_A, 5, 0b11100111);
  pinConfig(RE_C_LEFT_B, 5, 0b11100111);
  pinConfig(RE_MID_A, 5, 0b11100111);
  pinConfig(RE_MID_B, 5, 0b11100111);
  pinConfig(RE_C_RIGHT_A, 5, 0b11100111);
  pinConfig(RE_C_RIGHT_B, 5, 0b11100111);
  pinConfig(RE_RIGHT_A, 5, 0b11100111);
  pinConfig(RE_RIGHT_B, 5, 0b11100111);

  pinConfig(COLUMN1, 5, 33); // salidas
  pinConfig(COLUMN2, 5, 33);
  pinConfig(COLUMN3, 5, 33);
  pinConfig(COLUMN4, 5, 33);
  pinConfig(ROW1, 5, 0b11100111); // entrada
  pinConfig(ROW2, 5, 0b11100111);
  pinConfig(ROW3, 5, 0b11100111);

  pinConfig(led, 5, 0b00100001);
  pinConfig(led1, 5, 0b00100001);
  pinConfig(led2, 5, 0b00100001);

  pinConfig(fc1, 5, 0b11100111);
  pinConfig(fc2, 5, 0b11100111);
  //*-------------Pin Configurations-------------*/

  //*-------------HID-------------*/
  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);

  if (board_init_after_tusb)
  {
    board_init_after_tusb();
  }
  //*-------------HID-------------*/

  //*-------------I2C-------------*/
  // This example will use I2C1 on the default SDA and SCL pins (4, 5 on a Pico)
  i2c_init(&i2c1_inst, 100 * 1000);
  gpio_set_function(26, GPIO_FUNC_I2C);
  gpio_set_function(27, GPIO_FUNC_I2C);
  gpio_pull_up(26);
  gpio_pull_up(27);
  // Make the I2C pins available to picotool
  bi_decl(bi_2pins_with_func(26, 27, GPIO_FUNC_I2C));

  lcd_init();
  Mpu6050_init(MPU6050_adress);

  update_angle();

  while (true)
  {

    tud_task(); // tinyusb device task

    
    //-------------Matriz-------------
    if(hid_buttons != last_hid_buttons){
      if(hid_buttons){
        edge = 1;
        trigger = timer_hw->timelr;
      }

      last_hid_buttons = hid_buttons;
    }
    else{
      edge = 0;
    }

    /*
    if(buttons[1] && !menu && edge) updateButtons(1);
    if(buttons[2] && edge) updateButtons(2);
    if(buttons[3] && edge) updateButtons(3);
    if(buttons[4] && edge) updateButtons(4);
    if(buttons[5] && edge) updateButtons(5);
    if(buttons[6] && !menu && edge) updateButtons(6);
    if(buttons[7] && !menu && edge) updateButtons(7);
    if(buttons[8] && edge) updateButtons(8);
    if(buttons[9] && !menu && edge) updateButtons(9);
    if(buttons[10] && edge) updateButtons(10);
    if(buttons[11] && edge) updateButtons(11);
    if(buttons[12] && edge) updateButtons(12);
    */


    //-------------Matriz-------------
    confi();
    cali();
    memo();
    //-------------LCD-------------

    if((timer_hw->timelr > (last_lcd + 2e6)) && reset == 1 && !menu){
      port_avail = 0;
      update_angle();
      reset = 0;
    }

    if(timer_hw->timelr > (last_update + 500e3) && !menu && !reset && port_avail){
      port_avail = 0;
      last_update = timer_hw->timelr;
      update_angle();
    }

    if(menu && change_lcd){
      port_avail = 0;
      LCD_menu();
    }

    if(modo_conf){
      torque = encoder2;
      if(last_torque != torque){
        change_lcd = 1;
        last_torque = torque;
      }
    }

    else if(change_lcd & !menu){
      port_avail = 0;
      last_update = timer_hw->timelr;
      update_angle();
      change_lcd = 0;
    }


  }
  return 0;
}
//*------------- MAIN -------------*/

//*------------- SYSTICK -------------*/
void SysTick_Handler(void)
{ // Entra cada 1ms

  tiempo_envio_trama++;

  if((trigger + db_time) < timer_hw->timelr){
    hid_buttons = readMatrix(buttons, edges, menu); 
  }

  read_encoders(buttons);//! this could maybe break everything

  if(buttons[15]){
    hid_buttons |= 1 << 15;
  }

  if(buttons[16]){
    hid_buttons |= 1 << 16;
  }

  if(buttons[17]){
    hid_buttons |= 1 << 17;
  }

  if(buttons[18]){
    hid_buttons |= 1 << 18;
  }

  if(buttons[19]){
    hid_buttons |= 1 << 19;
  }

  if(buttons[20]){
    hid_buttons |= 1 << 20;
  }

  if(buttons[21]){
    hid_buttons |= 1 << 21;
  }

  if(buttons[22]){
    hid_buttons |= 1 << 22;
  }

  if(buttons[23]){
    hid_buttons |= 1 << 23;
  }

  if(buttons[24]){
    hid_buttons |= 1 << 24;
  }

  static int t = 0, temp_angle = 0, h = 0;

  if(h++ >= 10){
    h = 0;
    for(int i = 15; i < 25; i++){
      buttons[i] = 0;
    }
  }

  


  if (t++ > 100 && port_avail)
  {
    port_avail = 0;
    t = 0;

    Mpu6050_comm(&xaxis, &yaxis, &zaxis, MPU6050_adress);

    angle[i++] = (atan2((xaxis / 16384.0), (zaxis / 16384.0)) * (180 / pi)) - offset;

    if(i == 6)
      i = 0;

    port_avail = 1;

  
  }

  int32_t temp = 0;
  for(int j = 0; j < 7; j++){
    temp += angle[j];
  }

  f_angle = temp / 6;


  // 180 ------> 127
  //  x  ------> x  
  angle1 = ((f_angle * 127) / 180) * -1;
  hid_task((int8_t)angle1, 0, 0, 0, 0, 0, hid_buttons);

  return;
}
//*------------- SYSTICK -------------*/

/**
 * Main Control System Functions
 * ===========================
 */

/* Configuration and Setup Functions */
void beginUart() {
    // Configure UART for communication
    irq_set_exclusive_handler(UART0_IRQ, Recibe_car1);
    irq_set_enabled(UART0_IRQ, true);
    // Configure UART line control register
    uart0_hw->lcr_h = (uart0_hw->lcr_h & ~UART_UARTLCR_H_STP2_LSB) | (0b0) << UART_UARTLCR_H_STP2_LSB;
    
    // Configure parity (disabled)
    uart0_hw->lcr_h = (uart0_hw->lcr_h & ~UART_UARTLCR_H_EPS_LSB) | (0b0) << UART_UARTLCR_H_EPS_LSB;
    uart0_hw->lcr_h = (uart0_hw->lcr_h & ~UART_UARTLCR_H_PEN_BITS) | (0b0) << UART_UARTLCR_H_PEN_BITS;
    
    // Enable UART, TX and RX
    uart0_hw->cr = UART_UARTCR_UARTEN_BITS | UART_UARTCR_TXE_BITS | UART_UARTCR_RXE_BITS;
    
    // Configure TX pin (GPIO0)
    padsbank0_hw->io[0] = padsbank0_hw->io[0] & (~PADS_BANK0_GPIO0_OD_BITS);
    padsbank0_hw->io[0] = padsbank0_hw->io[0] | PADS_BANK0_GPIO0_IE_BITS;
    iobank0_hw->io[0].ctrl = GPIO_FUNC_UART;
    
    // Configure RX pin (GPIO1)
    padsbank0_hw->io[1] = padsbank0_hw->io[1] & (~PADS_BANK0_GPIO0_OD_BITS);
    padsbank0_hw->io[1] = padsbank0_hw->io[1] | PADS_BANK0_GPIO0_IE_BITS;
    iobank0_hw->io[1].ctrl = GPIO_FUNC_UART;
    
    // Enable RX interrupt
    uart0_hw->imsc = UART_UARTIMSC_RXIM_BITS;
    
    // Set interrupt handler
    irq_set_exclusive_handler(UART0_IRQ, Recibe_car1);
    irq_set_enabled(UART0_IRQ, true);
}

/* Communication Functions */
void Recibe_car1() {
    /* Handle UART receive interrupts
     * Processes incoming data and updates system state
     * Checks for special characters like '?' for replay mode
     */
    if((uart0_hw->ris & UART_UARTRIS_RXRIS_BITS) != 0) {
        // Process received character
        caracter_rec = uart0_hw->dr;
        if(caracter_rec1 =='!'){
          indice1=0;
        }

        recibo1[indice1++] = caracter_rec1;
        uart0_hw->icr |= UART_UARTICR_RXIC_BITS; // Limpio la interrupcion de RX
    }


    // !B13-B14?

    if(caracter_rec1=='?' && !replay){
      if(indice < 5){
        buttons[((recibo[2]-48)*10)+(recibo[3]-48)] = 1;
        if(edge)
          updateButtons(((recibo[2]-48)*10)+(recibo[3]-48));
      }
      else{
        buttons[((recibo[2]-48)*10)+(recibo[3]-48)] = 1;
        buttons[((recibo[6]-48)*10)+(recibo[7]-48)] = 1;
      } 
    }
}

/* Display Management Functions */
void updateButtons(uint8_t btn) {
    /* Updates LCD display when button state changes
     * Parameters:
     * - btn: Button number that triggered the update (1-14)
     */
    port_avail = 0;           // Lock port during update
    last_lcd = timer_hw->timelr;
    edges[btn] = 0;
    reset = 1;
    
    lcd_clear();
    sleep_us(10);             // Delay for LCD timing
    update_angle();
    sleep_us(10);
    lcd_set_cursor(1, 0);
    sleep_us(10);
    lcd_string(button_strings[btn]);
    port_avail = 1;           // Release port
}

void update_angle() {
    /* Updates angle display on LCD
     * - Clears previous display
     * - Formats angle with proper padding
     * - Handles positive and negative angles differently
     */
    port_avail = 0;
    char jjj[4];
    char *bspace = "               ";  // Bottom space padding
    
    lcd_set_cursor(1, 0);
    lcd_string(bspace);
    itoa(f_angle, jjj, 10);
    // lcd_clear();
    char jjj[4];
    char *bspace = "               "; // bottom space
    lcd_set_cursor(1, 0);
    lcd_string(bspace);
    itoa(f_angle, jjj, 10);
    char *angulo = jjj;
    char *m1 = "Angulo: ";
    char *space = "         ";
    lcd_set_cursor(0, 0);
    lcd_string(m1);
    if (f_angle < 10 && f_angle > 0)
    {
      lcd_set_cursor(0, 8);
      lcd_string(space);
    }
    else if (f_angle < 10 && f_angle < 0)
    {
      lcd_set_cursor(0, 9);
      lcd_string(space);
    }

    if (f_angle < 100 && f_angle > 0)
    {
      lcd_set_cursor(0, 9);
      lcd_string(space);
    }
    else if (f_angle < 100 && f_angle < 0)
    {
      lcd_set_cursor(0, 10);
      lcd_string(space);
    }

    lcd_set_cursor(0, 8);
    lcd_string(angulo);
    port_avail = 1;
    return;
}

void LCD_menu(){
  if (modo_cal)
  {
    lcd_clear();
    // sleep_ms(30);
    lcd_set_cursor(0, 0);
    // sleep_ms(30);
    lcd_string("Modo calibracion");
    change_lcd = 0;
    port_avail = 1;
  }

  if ((modo_conf && change_lcd) || ((torque != last_torque) && modo_conf))
  {
    lcd_clear();
    // sleep_ms(30);
    lcd_set_cursor(0, 0);
    // sleep_ms(30);
    char *bbb = "Fuente: ";
    lcd_string(bbb);
    char *ccc[] = {"juego", "sensores"};
    lcd_set_cursor(0, 8);
    datos = MARTIN;
    if (datos != last_datos)
    {
      last_datos = datos;
      if (datos)
        lcd_string(ccc[0]);
      else
        lcd_string(ccc[1]);
    }
    last_torque = torque;
    lcd_set_cursor(1, 0);
    char *ddd = "Torque: ";
    lcd_string(ddd);

    char eee[4];
    itoa(torque, eee, 10);

    lcd_set_cursor(1, 7);
    lcd_string(eee);
    change_lcd = 0;
    port_avail = 1;
    return;
  }

  if (modo_mem && change_lcd)
  {

    if (l_encoder1 != encoder1)
    { // TODO make the line change with the encoder
      if (((l_encoder1 - encoder1) > 0) && linea != 3)
        linea++;
      else if (((l_encoder1 - encoder1) < 0) && linea != 0)
        linea--;
    }

    if ((linea == 0 || linea == 1) && change)
    {
      change = 0;
      lcd_clear();
      lcd_set_cursor(0, 0);
      lcd_string(mem_strings[0]);
      lcd_set_cursor(1, 0);
      lcd_string(mem_strings[1]);
      if (linea == 0)
      {
        lcd_set_cursor(0, 7);
        lcd_string(mem_strings[4]);
      }
      else
      {
        lcd_set_cursor(1, 4);
        lcd_string(mem_strings[4]);
      }
    }

    if ((linea == 2 || linea == 3) && change)
    {
      change = 0;
      lcd_clear();
      lcd_set_cursor(0, 0);
      lcd_string(mem_strings[2]);
      lcd_set_cursor(1, 0);
      lcd_string(mem_strings[3]);
      if (linea == 2)
      {
        lcd_set_cursor(0, 11);
        lcd_string(mem_strings[4]);
      }
      else
      {
        lcd_set_cursor(1, 6);
        lcd_string(mem_strings[4]);
      }
    }
    change_lcd = 0;
    port_avail = 1;
  }
  return;
}

void cali(){
  actual_cal=buttons[7];
  if(anterior_cal != actual_cal && actual_cal == 1){
    anterior_cal = actual_cal;
    apreto_cal = timer_hw->timelr;
  }
  if(anterior_cal != actual_cal && actual_cal == 0){
    suelto_cal = timer_hw->timelr;
    anterior_cal = actual_cal;
    total_cal = suelto_cal - apreto_cal;
  }
  
  tiempo_actual_cal = timer_hw->timelr;

  if(total_cal >= 5e6 && !modo_cal)
  {
    if(veces_cal != 10)
    {
      if(tiempo_actual_cal >= (tiempo_anterior_cal + 1e6))
      {
        sio_hw->gpio_togl |= 1 << led;
        sio_hw->gpio_togl |= 1 << led1;
        sio_hw->gpio_togl |= 1 << led2;
        tiempo_anterior_cal = tiempo_actual_cal;
        veces_cal++;
      }
    }
      
    else if(veces_cal >= 10)
    {
      modo_cal = 1;
      total_cal = 0;
      veces_cal = 0;
      menu = 1;
      change_lcd = 1;
    }
  }
  else if(total_cal >= 5e6 && modo_cal)
  {
    if(veces_cal != 10)
    {
      if(tiempo_actual_cal >= (tiempo_anterior_cal + 1e6))
      {
        sio_hw->gpio_togl |= 1 << led;
        sio_hw->gpio_togl |= 1 << led1;
        sio_hw->gpio_togl |= 1 << led2;
        tiempo_anterior_cal = tiempo_actual_cal;
        veces_cal++;
      }
    }
    else if(veces_cal >= 10)
    {
      modo_cal = 0;
      total_cal = 0;
      veces_cal = 0;
      offset=f_angle;
      menu = 0;
      change_lcd = 1;
      return;
    }
  }
  else
    return;
}

void confi(){
  actual_con = buttons[6];
  if(anterior_con != actual_con && actual_con == 1){
    anterior_con = actual_con;
    apreto_con = timer_hw->timelr;
  }

  if(anterior_con != actual_con && actual_con == 0){
    suelto_con = timer_hw->timelr;
    anterior_con = actual_con;
    total_con = suelto_con - apreto_con;
  }


  tiempo_actual_con = timer_hw->timelr;

  if(total_con >= 5e6 && !modo_conf)
  {
    if(veces_con != 5)
    {
      if(tiempo_actual_con >= (tiempo_anterior_con + 1e6))
      {
        sio_hw->gpio_togl |= 1 << led;
        sio_hw->gpio_togl |= 1 << led1;
        sio_hw->gpio_togl |= 1 << led2;
        tiempo_anterior_con = tiempo_actual_con;
        veces_con++;
      }
    }
      
    else if(veces_con >= 5)
    {
      modo_conf = 1;
      total_con = 0;
      veces_con = 0;
      change_lcd = 1;
      menu = 1;
      return;
    }
  }

  else if(total_con >= 5e6 && modo_conf)
  {
      if(veces_con != 5)
      {
          if(tiempo_actual_con >= (tiempo_anterior_con + 1e6))
          {
            sio_hw->gpio_togl |= 1 << led;
            sio_hw->gpio_togl |= 1 << led1;
            sio_hw->gpio_togl |= 1 << led2;
            tiempo_anterior_con = tiempo_actual_con;
            veces_con++;
          }
      }
      else if(veces_con >= 5)
      {
        modo_conf = 0;
        total_con = 0;
        veces_con = 0;
        change_lcd = 1;
        menu = 0;
        return;
      }
  }

  if (buttons[1] && edge)
  {
    if (MARTIN == 1){
      MARTIN = 0;
      change_lcd = 1;
    }
    else{
      MARTIN = 1;
      change_lcd = 1;
    }
  }
}

void memo(){
    /* Handles frame transmission over UART
     * Frame format: !Saaa-Bn-EnXXX?
     * - S: Sign (+ or -)
     * - aaa: Angle (3 digits)
     * - Bn: Button states
     * - EnXXX: Encoder positions
     */
    if (tiempo_envio_trama >= 1000 && !mandar_trama) {
        tiempo_envio_trama = 0;
        
        // Initialize frame
        trama[0] = '!';           // Start delimiter
        trama[1] = 'S';           // Sign placeholder
        trama[2] = 'a';           // Angle hundreds
        trama[3] = 'a';           // Angle tens
        trama[4] = 'a';           // Angle ones
        // ...existing code...
    }
  
  if (tiempo_envio_trama >= 1000 && !mandar_trama)
  {
    tiempo_envio_trama = 0;

    trama[0] = '!';
    trama[1] = 'S';
    trama[2] = 'a';
    trama[3] = 'a';
    trama[4] = 'a';

    if (f_angle >= 0)
    {
      trama[1] = '+';
    }
    else
    {
      trama[1] = '-';
      f_angle = f_angle * -1;
    }
    trama[4] = (f_angle % 10) + 48;
    trama[3] = ((f_angle / 10) % 10) + 48;
    trama[2] = ((f_angle / 10) / 10) + 48;

    trama[5] = '\0';

    if (buttons[1])
      strcat(trama, "-B1");
    if (buttons[2])
      strcat(trama, "-B2");
    if (buttons[3])
      strcat(trama, "-B3");
    if (buttons[4])
      strcat(trama, "-B4");
    if (buttons[5])
      strcat(trama, "-B5");
    if (buttons[6])
      strcat(trama, "-B6");
    if (buttons[7])
      strcat(trama, "-B7");
    if (buttons[8])
      strcat(trama, "-B8");
    if (buttons[9])
      strcat(trama, "-B9");
    if (buttons[10])
      strcat(trama, "-B10");
    if (buttons[11])
      strcat(trama, "-B11");
    if (buttons[12])
      strcat(trama, "-B12");
    if (buttons[13])
      strcat(trama, "-B13");
    if (buttons[14])
      strcat(trama, "-B14");

    if (l_encoder1 != encoder1)
    {
      l_encoder1 = encoder1;

      strcat(trama, "-E1");
      char aux[4];
      aux[0] = (encoder1 % 10) + 48;
      aux[1] = ((encoder1 / 10) % 10) + 48;
      aux[2] = ((encoder1 / 10) / 10) + 48;
      aux[3] = '\0';
      strcat(trama, aux);
    }

    if (l_encoder2 != encoder2)
    {
      l_encoder2 = encoder2;

      strcat(trama, "-E2");
      char aux[4];
      aux[0] = (encoder2 % 10) + 48;
      aux[1] = ((encoder2 / 10) % 10) + 48;
      aux[2] = ((encoder2 / 10) / 10) + 48;
      aux[3] = '\0';
      strcat(trama, aux);
    }

    if (l_encoder3 != encoder3)
    {
      l_encoder3 = encoder3;

      char aux[4];
      aux[0] = (encoder3 % 10) + 48;
      aux[1] = ((encoder3 / 10) % 10) + 48;
      aux[2] = ((encoder3 / 10) / 10) + 48;
      aux[3] = '\0';
      strcat(trama, aux);
    }

    if (l_encoder4 != encoder4)
    {
      l_encoder4 = encoder4;

      strcat(trama, "-E4");
      char aux[4];
      aux[0] = (encoder4 % 10) + 48;
      aux[1] = ((encoder4 / 10) % 10) + 48;
      aux[2] = ((encoder4 / 10) / 10) + 48;
      aux[3] = '\0';
      strcat(trama, aux);
    }

    if (l_encoder5 != encoder5)
    {
      l_encoder5 = encoder5;

      strcat(trama, "-E5");
      char aux[4];
      aux[0] = (encoder5 % 10) + 48;
      aux[1] = ((encoder5 / 10) % 10) + 48;
      aux[2] = ((encoder5 / 10) / 10) + 48;
      aux[3] = '\0';
      strcat(trama, aux);
    }

    if (modo_conf)
      strcat(trama, "CN");

    strcat(trama, "?");
    mandar_trama = 1;
  }
  else if(tiempo_envio_trama >= 1000 && mandar_trama){
    tiempo_envio_trama = 0;
  }

  if (mandar_trama == 1)
  {
    if (uart0_hw->fr & UART_UARTFR_TXFE_BITS){ // Verifico que el puerto de tx esta ok
      uart0_hw->dr = trama[num]; // Manda el caracter correspondiente de la trama

      if (trama[num] != '?')
      { // Exepto que sea el caracter de fin, va incrementando
        num++;
      }
      else
      { // Si estamos en el final, reseteo todo
        num = 0;
        mandar_trama = 0;
        
        return;
      }
    }
    else return;
  }
  return;
}