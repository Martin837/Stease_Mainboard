#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "callbacks.h"
#include "hardware/adc.h"
#include "martinlib.c"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

#define pi 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679

//*-------------Display-------------*/
// commands
const int LCD_CLEARDISPLAY = 0x01;
const int LCD_RETURNHOME = 0x02;
const int LCD_ENTRYMODESET = 0x04;
const int LCD_DISPLAYCONTROL = 0x08;
const int LCD_CURSORSHIFT = 0x10;
const int LCD_FUNCTIONSET = 0x20;
const int LCD_SETCGRAMADDR = 0x40;
const int LCD_SETDDRAMADDR = 0x80;

// flags for display entry mode
const int LCD_ENTRYSHIFTINCREMENT = 0x01;
const int LCD_ENTRYLEFT = 0x02;

// flags for display and cursor control
const int LCD_BLINKON = 0x01;
const int LCD_CURSORON = 0x02;
const int LCD_DISPLAYON = 0x04;

// flags for display and cursor shift
const int LCD_MOVERIGHT = 0x04;
const int LCD_DISPLAYMOVE = 0x08;

// flags for function set
const int LCD_5x10DOTS = 0x04;
const int LCD_2LINE = 0x08;
const int LCD_8BITMODE = 0x10;

// flag for backlight control
const int LCD_BACKLIGHT = 0x08;

const int LCD_ENABLE_BIT = 0x04;

// OUR DISPLAY IS ON ADDRESS 0x3F
static int addr = 0x3F;

// Modes for lcd_send_byte
#define LCD_CHARACTER  1
#define LCD_COMMAND    0

#define MAX_LINES      2
#define MAX_CHARS      16

/* Quick helper function for single byte transfers */
void i2c_write_byte(uint8_t val) {
  i2c_write_blocking(&i2c1_inst, addr, &val, 1, false);
}

void lcd_toggle_enable(uint8_t val) {
  // Toggle enable pin on LCD display
  // We cannot do this too quickly or things don't work
  #define DELAY_US 600
  sleep_us(DELAY_US);
  i2c_write_byte(val | LCD_ENABLE_BIT);
  sleep_us(DELAY_US);
  i2c_write_byte(val & ~LCD_ENABLE_BIT);
  sleep_us(DELAY_US);
}

// The display is sent a byte as two separate nibble transfers
void lcd_send_byte(uint8_t val, int mode) {
  uint8_t high = mode | (val & 0xF0) | LCD_BACKLIGHT;
  uint8_t low = mode | ((val << 4) & 0xF0) | LCD_BACKLIGHT;

  i2c_write_byte(high);
  lcd_toggle_enable(high);
  i2c_write_byte(low);
  lcd_toggle_enable(low);
}

void lcd_clear(void) {
  lcd_send_byte(LCD_CLEARDISPLAY, LCD_COMMAND);
}

// go to location on LCD
void lcd_set_cursor(int line, int position) {
  int val = (line == 0) ? 0x80 + position : 0xC0 + position;  
  lcd_send_byte(val, LCD_COMMAND);
}

static inline void lcd_char(char val) {
  lcd_send_byte(val, LCD_CHARACTER);
}

void lcd_string(const char *s) {
  while (*s) {
    lcd_char(*s++);
  }
}

void lcd_init() {
  lcd_send_byte(0x03, LCD_COMMAND);
  lcd_send_byte(0x03, LCD_COMMAND);
  lcd_send_byte(0x03, LCD_COMMAND);
  lcd_send_byte(0x02, LCD_COMMAND);

  lcd_send_byte(LCD_ENTRYMODESET | LCD_ENTRYLEFT, LCD_COMMAND);
  lcd_send_byte(LCD_FUNCTIONSET | LCD_2LINE, LCD_COMMAND);
  lcd_send_byte(LCD_DISPLAYCONTROL | LCD_DISPLAYON, LCD_COMMAND);
  lcd_clear();
}
//*-------------Display-------------*/


//*-------------Prototypes-------------*/
void beginUart();

void update_angle(); //LCD

void updateButtons(uint8_t btn); //LCD

void Mpu6050_init(int16_t address);

void Mpu6050_comm(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z, int16_t address);
//*-------------Prototypes-------------*/

//*-------------Pin Definitions-------------*/
#define RE_LEFT_A 11
#define RE_LEFT_B 10
#define RE_C_LEFT_A 9
#define RE_C_LEFT_B 8
#define RE_MID_A 7
#define RE_MID_B 6
#define RE_C_RIGHT_A 5
#define RE_C_RIGHT_B 4
#define RE_RIGHT_A 3
#define RE_RIGHT_B 2

#define COLUMN1 15
#define COLUMN2 22
#define COLUMN3 18
#define COLUMN4 19
#define ROW1 14
#define ROW2 13
#define ROW3 12

#define led 20
#define led1 21
#define led2 28
//*-------------Pin Definitions-------------*/

//*-------------Global Variables-------------*/
#define MPU6050_adress 0x68

uint32_t buttons = 0; //HID

int tiempo_envio_trama = 0; //Mem

uint8_t  matriz = 0;

char *button_strings[] = {" ", "Boton 1", "Boton 2", "Boton 3", "Boton 4", "Boton 5", "Boton 6", "Boton 7", "Boton 8", "Boton 9", "Boton 10", "Boton 11", "Boton 12", "Boton 13", "Boton 14", "Boton 15", "Boton 16", "Boton 17", "Boton 18", "Boton 19", "Boton 20", "Boton 21", "Boton 22"};

int8_t angle = 0;

uint8_t reset = 0; //LCD

int grado2 = 0;
int16_t xaxis, yaxis, zaxis;
//*-------------Global Variables-------------*/

//*------------- MAIN -------------*/
int main(void){
  board_init();
  stdio_init_all();
  SystemCoreClockUpdate();

  begin_systick(124999);
  beginUart();

  //*-------------Pin Configurations-------------*/
  pinConfig(RE_LEFT_A,5,0b11100111);
  pinConfig(RE_LEFT_B,5,0b11100111);
  pinConfig(RE_C_LEFT_A,5,0b11100111);
  pinConfig(RE_C_LEFT_B,5,0b11100111);
  pinConfig(RE_MID_A,5,0b11100111);
  pinConfig(RE_MID_B,5,0b11100111);
  pinConfig(RE_C_RIGHT_A,5,0b11100111);
  pinConfig(RE_C_RIGHT_B,5,0b11100111);
  pinConfig(RE_RIGHT_A,5,0b11100111);
  pinConfig(RE_RIGHT_B,5,0b11100111);

  pinConfig(COLUMN1, 5, 33); //salidas
  pinConfig(COLUMN2, 5, 33);
  pinConfig(COLUMN3, 5, 33);
  pinConfig(COLUMN4, 5, 33);
  pinConfig(ROW1, 5, 0b11100111);//entrada
  pinConfig(ROW2, 5, 0b11100111);
  pinConfig(ROW3, 5, 0b11100111);

  pinConfig(led,5,0b00100001);
  pinConfig(led1,5,0b00100001);
  pinConfig(led2,5,0b00100001);
  //*-------------Pin Configurations-------------*/
  
  //*-------------Variables-------------*/
  uint16_t x = 0; //HID
  int8_t sx = 0; //HID
  uint32_t last = 0; //HID

  uint8_t encoder1A=0, encoder1B=0,encoder2A=0, encoder2B=0,encoder3A=0, encoder3B=0,encoder4A=0, encoder4B=0,encoder5A=0, encoder5B=0; //Encoder
  uint8_t ultimo1,ultimo2,ultimo3,ultimo4,ultimo5; //Encoder
  uint8_t encoder1,encoder2=0,encoder3=0,encoder4=0,encoder5=0; //Encoder
  uint8_t l_encoder1 = 0, l_encoder2 = 0, l_encoder3 = 0, l_encoder4 = 0, l_encoder5 = 0; //Encoder

  uint8_t stateSW1 = 0, stateSW2 = 0, stateSW3 = 0, stateSW4 = 0, stateSW5 = 0, stateSW6 = 0, stateSW7 = 0, stateSW8 = 0; //Matriz
  uint8_t stateSW9=0, stateSW10 = 0, stateSW11 = 0, stateSW12 = 0; //Matriz

  uint8_t veces_con = 0, modo_conf = 0; //Confi
  uint32_t tiempo_anterior_con = 0, tiempo_actual_con = 0, actual_con, anterior_con = 0, apreto_con = 0, suelto_con = 0, total_con = 0; //Confi
  uint8_t MARTIN=0; //fuente de datos Confi

  uint8_t veces_cal = 0,modo_cal=0, offset=0; //Calibración
  uint32_t tiempo_anterior_cal = 0, tiempo_actual_cal = 0, actual_cal, anterior_cal = 0, apreto_cal = 0, suelto_cal = 0, total_cal = 0; //Calibración

  uint8_t start_recording = 0, stop_recording = 0, start_streaming = 0, stop_streaming = 0, replay = 0; //Mem
  int mandar_trama = 0, num = 0; //Mem
  char trama[50]="!Saaa-Bn-Bn-Enxxx-Mx"; //Mem
  uint32_t tiempo_anterior = 0, tiempo_actual = 0, diferencia = 0; //Mem
  uint8_t veces_mem = 0; //Confi
  uint32_t tiempo_anterior_mem = 0, tiempo_actual_mem = 0, actual_mem, anterior_mem = 0, apreto_mem = 0, suelto_mem = 0, total_mem = 0; //Confi

  uint8_t stateSW13 = 0, stateSW14 = 0; //Gears 

  //*-------------Variables-------------*/


  //*-------------HID-------------*/
  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);

  if (board_init_after_tusb) {
    board_init_after_tusb();
  }
  //*-------------HID-------------*/

  //*-------------LCD-------------*/
  // This example will use I2C1 on the default SDA and SCL pins (4, 5 on a Pico)
  i2c_init(&i2c1_inst, 100 * 1000);
  gpio_set_function(26, GPIO_FUNC_I2C);
  gpio_set_function(27, GPIO_FUNC_I2C);
  gpio_pull_up(26);
  gpio_pull_up(27);
  // Make the I2C pins available to picotool
  bi_decl(bi_2pins_with_func(26, 27, GPIO_FUNC_I2C));

  lcd_init();
  uint32_t last_lcd = 0;
  uint8_t last_state = 0;

  //menus
  uint8_t menu = 0, modo_mem = 0, datos = 0, torque = 50, last_datos = 1, last_torque = 0;
  //menu: is a menu active? if it is then buttons can't be displayed.
  uint8_t linea = 2, change = 1; //modo mem
  char *mem_strings[] = {"Inicio", "Fin", "Repeticion", "Salir", "<"};
  
  update_angle();
  //*-------------LCD-------------*/


  //! TODO Display
  //! TODO IMU
  //TODO Replay



  while (true){
    tud_task(); // tinyusb device task


    if(stateSW1) buttons |= 1 << 1;
    else buttons &= ~(1<<1);
    if(stateSW2) buttons |= 1 << 2;
    else buttons &= ~(1<<2);
    if(stateSW3) buttons |= 1 << 3;
    else buttons &= ~(1<<3);
    if(stateSW4) buttons |= 1 << 4;
    else buttons &= ~(1<<4);
    if(stateSW5) buttons |= 1 << 5;
    else buttons &= ~(1<<5);
    if(stateSW6) buttons |= 1 << 6;
    else buttons &= ~(1<<6);
    if(stateSW7) buttons |= 1 << 7;
    else buttons &= ~(1<<7);
    if(stateSW8) buttons |= 1 << 8;
    else buttons &= ~(1<<8);
    if(stateSW9) buttons |= 1 << 9;
    else buttons &= ~(1<<9);
    if(stateSW10) buttons |= 1 << 10;
    else buttons &= ~(1<<10);
    if(stateSW11) buttons |= 1 << 11;
    else buttons &= ~(1<<11);
    if(stateSW12) buttons |= 1 << 12;
    else buttons &= ~(1<<12);


    //*-------------Encoders-------------*/
    encoder1A = readPin(RE_LEFT_A, 1);
    encoder1B = readPin(RE_LEFT_B, 1);
    encoder2A = readPin(RE_C_LEFT_A, 1);
    encoder2B = readPin(RE_C_LEFT_B, 1);
    encoder3A = readPin(RE_MID_A, 1);
    encoder3B = readPin(RE_MID_B, 1);
    encoder4A = readPin(RE_C_RIGHT_A, 1);
    encoder4B = readPin(RE_C_RIGHT_B, 1);
    encoder5A = readPin(RE_RIGHT_A, 1);
    encoder5B = readPin(RE_RIGHT_B, 1);

    // si el A es diferente al ultimo estado, that means a Pulse has occured
    if(encoder1A != ultimo1){     
      // si el B es diferente al A, that means the encoder is rotating clockwise
      if(encoder1B != encoder1A) { 
        encoder1 ++;
        buttons |= 1 << 13;
        updateButtons(13);
      } else {
        encoder1 --;
        buttons |= 1 << 14;
        updateButtons(14);
      }
    } 
    ultimo1 = encoder1A; // poner el ultimo estado q esta y no entre al if

    // si el A es diferente al ultimo estado, that means a Pulse has occured
    if(encoder2A != ultimo2){     
      // si el B es diferente al A, that means the encoder is rotating clockwise
      if(encoder2B != encoder2A) { 
        encoder2 ++;
        buttons |= 1 << 15;
        updateButtons(15);
      } else {
        encoder2 --;
        buttons |= 1 << 16;
        updateButtons(16);
      }
    } 
    ultimo2 = encoder2A; // poner el ultimo estado q esta y no entre al if

    // si el A es diferente al ultimo estado, that means a Pulse has occured
    if(encoder3A != ultimo3){     
      // si el B es diferente al A, that means the encoder is rotating clockwise
      if(encoder3B != encoder3A) { 
        encoder3 ++;
        buttons |= 1 << 17;
        updateButtons(17);
      } else {
        encoder3 --;
        buttons |= 1 << 18;
        updateButtons(18);
      }
    } 
    ultimo3 = encoder3A; // poner el ultimo estado q esta y no entre al if

    // si el A es diferente al ultimo estado, that means a Pulse has occured
    if(encoder4A != ultimo4){     
      // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
      if(encoder4B != encoder4A) { 
        encoder4 ++;
        buttons |= 1 << 19;
        updateButtons(19);
      } else {
        encoder4 --;
        buttons |= 1 << 20;
        updateButtons(20);
      }
    } 
    ultimo4 = encoder4A; // poner el ultimo estado q esta y no entre al if

    // si el A es diferente al ultimo estado, that means a Pulse has occured
    if(encoder5A != ultimo5){     
      // si el B es diferente al A, that means the encoder is rotating clockwise
      if(encoder5B != encoder5A) { 
        encoder5 ++;
        buttons |= 1 << 21;
        updateButtons(21);
      } else {
        encoder5 --;
        buttons |= 1 << 22;
        updateButtons(22);
      }
    } 
    ultimo5 = encoder5A; // poner el ultimo estado q esta y no entre al if
    //*-------------Encoders-------------*/

    //*-------------Matriz-------------*/
    writePin(COLUMN4,0,0);
    writePin(COLUMN1,0,1);
    stateSW1 = readPin(ROW1, 1) & !stateSW4;
    stateSW5 = readPin(ROW2, 1) & !stateSW8;
    stateSW9 = readPin(ROW3, 1) & !stateSW12;
    writePin(COLUMN1,0,0);
    writePin(COLUMN2,0,1);
    stateSW2 = readPin(ROW1, 1) & !stateSW1;
    stateSW6 = readPin(ROW2, 1) & !stateSW5;
    stateSW10 = readPin(ROW3, 1) & !stateSW9;
    writePin(COLUMN2,0,0);
    writePin(COLUMN3,0,1);
    stateSW3 = readPin(ROW1, 1) & !stateSW2;
    stateSW7 = readPin(ROW2, 1) & !stateSW6;
    stateSW11 = readPin(ROW3, 1) & !stateSW10;
    writePin(COLUMN3,0,0);
    writePin(COLUMN4,0,1);
    stateSW4 = readPin(ROW1, 1) & !stateSW3;
    stateSW8 = readPin(ROW2, 1) & !stateSW7;
    stateSW12 = readPin(ROW3, 1) & !stateSW11;
    //*-------------Matriz-------------*/

    //*-------------Confi-------------*/
    actual_con = stateSW6;
    if(anterior_con!=actual_con && actual_con==1){
      anterior_con = actual_con;
      apreto_con = timer_hw->timelr;
    }
    if(anterior_con != actual_con && actual_con ==0){
      suelto_con = timer_hw->timelr;
      anterior_con = actual_con;
      total_con = suelto_con - apreto_con;
    }


    if(total_con >= 5e6 && !modo_conf)
    {
      if(veces_con != 10)
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
      
      else if(veces_con >= 10)
      {
        modo_conf = 1;
        menu = 1;
        total_con = 0;
        veces_con = 0;
      }
    }

    else if(total_con >= 5e6 && modo_conf)
    {
      if(veces_con != 10)
      {
        if(tiempo_actual_con >= (tiempo_anterior_con + 1e6))
        {
          sio_hw->gpio_togl |= 1 << led;
          sio_hw->gpio_togl |= 1 << led1;
          sio_hw->gpio_togl |= 1 << led2;
          tiempo_anterior_con= tiempo_actual_con;
          veces_con++;
        }
      }
      else if(veces_con >= 10)
      {
        modo_conf = 0;
        menu = 0;
        total_con=0;
        veces_con=0;
      }
    }

    if(stateSW1){
      if(MARTIN==1)
        MARTIN=0;
      else
        MARTIN=1;
    }
    //*-------------Confi-------------*/

    //*-------------Cal-------------*/
    actual_cal = stateSW7;
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
        menu = 1;
        total_cal = 0;
        veces_cal = 0;
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
        menu = 0;
        total_cal = 0;
        veces_cal = 0;
        offset = angle;
      }
    }
    //*-------------Cal-------------*/

    //*-------------Mem-------------*/
    actual_mem = stateSW7;
    if(anterior_mem != actual_mem && actual_mem == 1){
      anterior_mem = actual_mem;
      apreto_mem = timer_hw->timelr;
    }
    if(anterior_mem != actual_mem && actual_mem == 0){
      suelto_mem = timer_hw->timelr;
      anterior_mem = actual_mem;
      total_mem = suelto_mem - apreto_mem;
    }
    
    tiempo_actual_mem = timer_hw->timelr;

    if(total_mem >= 5e6 && !modo_mem)
    {
      if(veces_mem != 10)
      {
        if(tiempo_actual_mem >= (tiempo_anterior_mem + 1e6))
        {
          sio_hw->gpio_togl |= 1 << led;
          sio_hw->gpio_togl |= 1 << led1;
          sio_hw->gpio_togl |= 1 << led2;
          tiempo_anterior_mem = tiempo_actual_mem;
          veces_mem++;
        }
      }
      
      else if(veces_mem >= 10)
      {
        modo_mem = 1;
        menu = 1;
        total_mem = 0;
        veces_mem = 0;
      }
    }

    else if(total_mem >= 5e6 && modo_mem)
    {
      if(veces_mem != 10) 
      {
        if(tiempo_actual_mem >= (tiempo_anterior_mem + 1e6))
        {
          sio_hw->gpio_togl |= 1 << led;
          sio_hw->gpio_togl |= 1 << led1;
          sio_hw->gpio_togl |= 1 << led2;
          tiempo_anterior_mem = tiempo_actual_mem;
          veces_mem++;
        }
      }
      else if(veces_mem >= 10)
      {
        modo_mem = 0;
        menu = 0;
        total_mem = 0;
        veces_mem = 0;
      }
    }
    //! TODO access mem menu
    if(tiempo_envio_trama >= 1000){
      tiempo_envio_trama = 0;
      for(int i = 5; i < 50; i++){
        trama[i]=0;
      }

      trama[0]='!';
      trama[1]='S';
      trama[2]='a';
      trama[3]='a';
      trama[4]='a';

      if(angle>=0){
        trama[1]='+';
      }
      else{
        trama[1]='-';
        angle=angle*-1; 
      }
      trama[4]=(angle%10)+48; 
      trama[3]=((angle/10)%10)+48;
      trama[2]=((angle/10)/10)+48;

      if(stateSW1)strcat(trama,"-B1");
      if(stateSW2)strcat(trama,"-B2");
      if(stateSW3)strcat(trama,"-B3");
      if(stateSW4)strcat(trama,"-B4");
      if(stateSW5)strcat(trama,"-B5");
      if(stateSW6)strcat(trama,"-B6");
      if(stateSW7)strcat(trama,"-B7");
      if(stateSW8)strcat(trama,"-B8");
      if(stateSW9)strcat(trama,"-B9");
      if(stateSW10)strcat(trama,"-B10");
      if(stateSW11)strcat(trama,"-B11");
      if(stateSW12)strcat(trama,"-B12");
      if(stateSW13)strcat(trama,"-B13");
      if(stateSW14)strcat(trama,"-B14");
      

      if(l_encoder1 != encoder1){ 
        l_encoder1 = encoder1;

        strcat(trama,"-E1");
        char aux[4];
        aux[0] = (encoder1%10)+48;
        aux[1] = ((encoder1/10)%10)+48;
        aux[2] = ((encoder1/10)/10)+48;
        aux[3] = '\0';
        strcat(trama, aux);
      }

      if(l_encoder2 != encoder2){
        l_encoder2 = encoder2;

        strcat(trama,"-E2");
        char aux[4];
        aux[0] = (encoder2%10)+48;
        aux[1] = ((encoder2/10)%10)+48;
        aux[2] = ((encoder2/10)/10)+48;
        aux[3] = '\0';
        strcat(trama, aux);
      }

      if(l_encoder3 != encoder3){
        l_encoder3 = encoder3;

        char aux[4];
        aux[0] = (encoder3%10)+48;
        aux[1] = ((encoder3/10)%10)+48;
        aux[2] = ((encoder3/10)/10)+48;
        aux[3] = '\0';
        strcat(trama, aux);
      }

      if(l_encoder4 != encoder4){
        l_encoder4 = encoder4;

        strcat(trama,"-E4");
        char aux[4];
        aux[0] = (encoder4%10)+48;
        aux[1] = ((encoder4/10)%10)+48;
        aux[2] = ((encoder4/10)/10)+48;
        aux[3] = '\0';
        strcat(trama, aux);
      }

      if(l_encoder5 != encoder5){
        l_encoder5 = encoder5;

        strcat(trama,"-E5");
        char aux[4];
        aux[0] = (encoder5%10)+48;
        aux[1] = ((encoder5/10)%10)+48;
        aux[2] = ((encoder5/10)/10)+48;
        aux[3] = '\0';
        strcat(trama, aux);
      }
      
      if(menu && stateSW9 && linea == 0)
        strcat(trama,"-R1");
      
      if(menu && stateSW9 && linea == 0)
        strcat(trama,"-R0");
      
      if(menu && stateSW9 && linea == 0){
        strcat(trama,"-S1");
        replay = 1;
      }  
      
      if(menu && stateSW9 && linea == 0){  
        strcat(trama,"-S0");
        replay = 0;
      }
      strcat(trama,"?");
      mandar_trama = 1;
    }

    if(mandar_trama == 1){
      if(uart0_hw->fr & UART_UARTFR_TXFE_BITS){  // Verifico que el puerto de tx esta ok 
        uart0_hw->dr = trama[num]; // Manda el caracter correspondiente de la trama

        if(trama[num] != '?')
        { // Exepto que sea el caracter de fin, va incrementando
          num++;
        }
        else
        { // Si estamos en el final, reseteo todo
          num = 0;
          mandar_trama = 0;
        }
      }
    }
    //*-------------Mem-------------*/

    //*-------------LCD-------------*/

    //Buttons & encoders
    if(stateSW1 && !reset) updateButtons(1);
    if(stateSW2 && !reset) updateButtons(2);
    if(stateSW3 && !reset) updateButtons(3);
    if(stateSW4 && !reset) updateButtons(4);
    if(stateSW5 && !reset) updateButtons(5);
    if(stateSW6 && !reset) updateButtons(6);
    if(stateSW7 && !reset) updateButtons(7);
    if(stateSW8 && !reset) updateButtons(8);
    if(stateSW9 && !reset) updateButtons(9);
    if(stateSW10 && !reset) updateButtons(10);
    if(stateSW11 && !reset) updateButtons(11);
    if(stateSW12 && !reset) updateButtons(12);
    if(stateSW13 && !reset) updateButtons(13);
    if(stateSW14 && !reset) updateButtons(14);

    if((timer_hw->timelr > (last_lcd + 2e6)) && reset == 1 && !menu){
      last_lcd = timer_hw->timelr;
      update_angle();
      reset = 0;
    }

    if(menu){
      if(modo_cal){
        lcd_clear();
        //sleep_ms(30);
        lcd_set_cursor(0, 0);
        //sleep_ms(30);
        char *bbb = "Modo calibracion";
        lcd_string(bbb);
      }

      if(modo_conf){
        lcd_clear();
        //sleep_ms(30);
        lcd_set_cursor(0, 0);
        //sleep_ms(30);
        char *bbb = "Fuente: ";
        lcd_string(bbb);
        char *ccc[] = {"juego", "sensores"};
        lcd_set_cursor(0, 8);
        if(datos!=last_datos){
          last_datos = datos;
          if(datos)
            lcd_string(ccc[0]);
          else
            lcd_string(ccc[1]);
        }

          if(torque != last_torque){
            last_torque = torque;
            lcd_set_cursor(1,0);
            char *ddd = "Torque: ";
            lcd_string(ddd);

            char eee[3];
            itoa(torque, eee, 10);

            lcd_set_cursor(1,7);
            lcd_string(eee);
          }
      }

      if(modo_mem){

        if(l_encoder1 != encoder1){ //TODO make the line change with the encoder
          if(((l_encoder1-encoder1) > 0) && linea != 3)
            linea++;
          else if(((l_encoder1-encoder1) < 0) && linea != 0)
            linea--;
        }

        if((linea == 0 || linea == 1) && change){
          change = 0;
          lcd_clear();
          lcd_set_cursor(0,0);
          lcd_string(mem_strings[0]);
          lcd_set_cursor(1,0);
          lcd_string(mem_strings[1]);
          if(linea == 0){
            lcd_set_cursor(0,7);
            lcd_string(mem_strings[4]);
          }
          else{
            lcd_set_cursor(1,4);
            lcd_string(mem_strings[4]);
          }
        }

        if((linea == 2 || linea == 3) && change){
          change = 0;
          lcd_clear();
          lcd_set_cursor(0,0);
          lcd_string(mem_strings[2]);
          lcd_set_cursor(1,0);
          lcd_string(mem_strings[3]);
          if(linea == 2){
            lcd_set_cursor(0,11);
            lcd_string(mem_strings[4]);
          }
          else{
            lcd_set_cursor(1,6);
            lcd_string(mem_strings[4]);
          }
        }
      }
    }
    //*-------------LCD-------------*/

  }
  return 0;
}
//*------------- MAIN -------------*/

//*------------- SYSTICK -------------*/
void SysTick_Handler(void){ //Entra cada 1ms
  tiempo_envio_trama++;

  hid_task(angle, 0, 0, 0, 0, 0, buttons);

  static int t = 0;
  if (t++ > 100){
    t = 0;

    Mpu6050_comm(&xaxis, &yaxis, &zaxis, MPU6050_adress);

    int angle1 = atan2((xaxis / 16384.0), (zaxis / 16384.0)) * (180 / pi);
    
    angle = (angle1 * 128) / 180;
    //update_angle();
    if(matriz != 3) matriz++;
    else matriz = 0;
  }
  return;
}
//*------------- SYSTICK -------------*/


void beginUart(){
  uart0_hw->ibrd = 813;
  uart0_hw->fbrd = 52;
  // Largo de la palabra
  // 11 -> 8 bits
  uart0_hw->lcr_h = (uart0_hw->lcr_h & ~UART_UARTLCR_H_WLEN_LSB) | (0b11) << UART_UARTLCR_H_WLEN_LSB;
  // Bits de stop
  // 0 -> 1 bit
  uart0_hw->lcr_h = (uart0_hw->lcr_h & ~UART_UARTLCR_H_STP2_LSB) | (0b0) << UART_UARTLCR_H_STP2_LSB;
  // Seleccion de paridad
  // 0 -> Paridad impar
  uart0_hw->lcr_h = (uart0_hw->lcr_h & ~UART_UARTLCR_H_EPS_LSB) | (0b0) << UART_UARTLCR_H_EPS_LSB;
  // Enable de paridad
  // 0 -> Paridad desabilitada
  uart0_hw->lcr_h = (uart0_hw->lcr_h & ~UART_UARTLCR_H_PEN_BITS) | (0b0) << UART_UARTLCR_H_PEN_BITS;
  // Habilito UART, TX y RX
  uart0_hw->cr = UART_UARTCR_UARTEN_BITS | UART_UARTCR_TXE_BITS | UART_UARTCR_RXE_BITS;
  // Configuro el pin 0 para la funcion de UART (TX)
  padsbank0_hw->io[0] = padsbank0_hw->io[0] & (~PADS_BANK0_GPIO0_OD_BITS); 
  padsbank0_hw->io[0] = padsbank0_hw->io[0] | PADS_BANK0_GPIO0_IE_BITS;    
  iobank0_hw->io[0].ctrl = GPIO_FUNC_UART;
  // Configuro el pin 1 para la funcion de UART (RX)
  padsbank0_hw->io[1] = padsbank0_hw->io[1] & (~PADS_BANK0_GPIO0_OD_BITS);
  padsbank0_hw->io[1] = padsbank0_hw->io[1] | PADS_BANK0_GPIO0_IE_BITS;   
  iobank0_hw->io[1].ctrl = GPIO_FUNC_UART;
  return;
}

void updateButtons(uint8_t btn){
  reset = 1;
  lcd_clear();
  update_angle();
  lcd_set_cursor(1, 0);
  lcd_string(button_strings[btn]);
  return;
}

void update_angle(){
  lcd_clear();
  char jjj[4];
  itoa(angle, jjj, 10);
  char *angulo = jjj;
  char * m1 = "Angulo: ";
  lcd_set_cursor(0, 0);
  lcd_string(m1);
  lcd_set_cursor(0, 8);
  lcd_string(angulo);
  return;
}

void Mpu6050_init(int16_t address){
  uint8_t buf[2];
  buf[0] = 0x6B; // registro de administracion de energia
  buf[1] = 0x00; // establezco un 0 para despertar al MPU6050
  i2c_write_blocking(i2c1, address, buf, 2, false);
  
  uint8_t buffer = 0x1C;
  i2c_write_blocking(i2c1, address, &buffer, 1, true); // envio el registro a escrbir
  buffer = 0b00000000;
  i2c_write_blocking(i2c1, address, &buffer, 1, false); // seteo el full scale range en 2g
  return;
}

void Mpu6050_comm(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z, int16_t address){
  uint8_t buffer[8];

  buffer[0] = 0x3B;

  i2c_write_blocking(i2c1, address, buffer, 1, true); // envio el registro a leer
  i2c_read_blocking(i2c1, address, buffer, 6, false); // leo el mpu6050

  *accel_x = (buffer[0] << 8) | buffer[1]; // guardo la informacion del eje x
  *accel_y = (buffer[2] << 8) | buffer[3]; // guardo la informacion del eje y
  *accel_z = (buffer[4] << 8) | buffer[5]; // guardo la informacion del eje z
  return;
}