// functions to operate the ILI9163C on the PIC32
// adapted from https://github.com/sumotoy/TFT_ILI9163C/blob/master/TFT_ILI9163C.cpp

// pin connections:
// VCC - 3.3V
// GND - GND
// CS - B7
// RESET - 3.3V
// A0 - B15
// SDA - A1
// SCK - B14
// LED - 3.3V

// B8 is turned into SDI1 but is not used or connected to anything
#include "ILI9163C.h"
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <stdio.h>
#include <math.h> // used in HW4
#define CS LATBbits.LATB7   // chip select pin
#define PI 3.14159265 // used in HW4

// addresses used for IMU
#define WHO_AM_I 0x0F
#define IMU_ADDR 0b11010110 // full 8 bit address, has pull up resistors
#define IMU_7bit 0b1101011 // 7 bit address - shift over for 1/0 r/w
#define CTRL1_XL 0x10 // turn on accelerometer
#define CTRL2_G 0x11 // turn on gyroscope
#define CTRL3_C 0x12 // turn on IF_INC --- enable multiple read
#define OUT_TEMP_L 0x20 // call this for multiple read

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = ON // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV =  DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0x1000 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY=  OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

////////////////////////// function prototypes /////////////////////////////////
void PIC32_init(void);
void SPI1_init(void);
void LCD_drawChar(unsigned char, unsigned char, unsigned char);
void LCD_drawStr(unsigned char, unsigned char, unsigned char *, int);

// LCD function prototypes included in the header file

void i2c_master_start(void);
void i2c_master_restart(void);
void i2c_master_send(unsigned char byte);
unsigned char i2c_master_recv(void);
void i2c_master_ack(int val);
void i2c_master_stop(void);
void i2c_master_setup(void);

// IMU functions
void imu_setup(void);
unsigned char I2C_read_single(void);
void I2C_read_multiple(char, char, unsigned char *, char);
void bit_shift(void);
void OC_setup(void);

// initialize variables
unsigned char outputs[14]; // length 14 b/c 14 registers to read from linearly
signed short temp = 0;
signed short g_x = 0, g_y = 0, g_z = 0;
signed short xl_x = 0, xl_y = 0, xl_z = 0;

unsigned char write_string[100];


int main() {

    __builtin_disable_interrupts();

    PIC32_init();
    SPI1_init();
    LCD_init();
    i2c_master_setup();
    imu_setup();
    LCD_clearScreen(BLACK);
        
    __builtin_enable_interrupts();
    
//    LCD_drawChar(10,10,'H');

//    // Part A of HW 5
//    // Print out "Hello World 1337" where 1337 is an int variable
//    unsigned char x_start = 28; 
//    unsigned char y_start = 32;
    int write_var = 1337;
    int total_char = sprintf(write_string, "Hello World %d", write_var);
//    LCD_drawStr(x_start,y_start,write_string,total_char);

//    // print out the number of characters on screen 
//    int test_char = sprintf(test_string, "%d",total_char);
//    unsigned char test_string[100];
//    LCD_drawStr(0,100,test_string,test_char);

    while(1){
        
        _CP0_SET_COUNT(0); // core timer = 0, runs at half CPU speed
        // read slower than 50 Hz to read on LCD
        while (_CP0_GET_COUNT() < 960000){;} // read at 25 Hz -- 960k / 24 MHz
        
        // values = I2C_read_single(); // read from who_am_i register
        I2C_read_multiple(IMU_7bit, OUT_TEMP_L, outputs, 14); // multiple reads
        // data is read from OUT_TEMP_L, moving "upward" -- 14 registers total
        temp = (outputs[0] | (outputs[1] << 8));
        g_x = (outputs[2] | (outputs[3] << 8));
        g_y = (outputs[4] | (outputs[5] << 8));
        g_z = (outputs[6] | (outputs[7] << 8));
        xl_x = (outputs[8] | (outputs[9] << 8));
        xl_y = (outputs[10] | (outputs[11] << 8));
        xl_z = (outputs[14] | (outputs[13] << 8));
        
        // print the x values of accelerometer onto LCD 
        int total_char = sprintf(write_string, "Accel in x: %5.3f g", (float)(xl_x * 2.0 /32767));
        LCD_drawStr(0, 0,write_string,total_char);    
        
    }
}

/////////////////////////////////// I2C ///////////////////////////////////////
void i2c_master_setup(void) {
    // maximum baud = 400 kHz
    // use 100 kHz for now, use 400 kHz later
    // Fsck = 100 kHz, PGD = 104 ns (the typical value, on page 269), Pblck = 48 MHz
    I2C2BRG = 233;            // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2
                                    // look up PGD for your PIC32
    I2C2CONbits.ON = 1;               // turn on the I2C2 module

    // I2C2 are analog inputs by default --- turn them off
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
}

// Start a transmission on the I2C bus
void i2c_master_start(void) {
    I2C2CONbits.SEN = 1;            // send the start bit
    while(I2C2CONbits.SEN) { ; }    // wait for the start bit to be sent
}

void i2c_master_restart(void) {
    I2C2CONbits.RSEN = 1;           // send a restart
    while(I2C2CONbits.RSEN) { ; }   // wait for the restart to clear
}

void i2c_master_send(unsigned char byte) { // send a byte to slave
  I2C2TRN = byte;                   // if an address, bit 0 = 0 for write, 1 for read
  while(I2C2STATbits.TRSTAT) { ; }  // wait for the transmission to finish
  if(I2C2STATbits.ACKSTAT) {        // if this is high, slave has not acknowledged
    // ("I2C2 Master: failed to receive ACK\r\n");
  }
}

unsigned char i2c_master_recv(void) { // receive a byte from the slave
    I2C2CONbits.RCEN = 1;             // start receiving data
    while(!I2C2STATbits.RBF) { ; }    // wait to receive the data
    return I2C2RCV;                   // read and return the data
}

void i2c_master_ack(int val) {        // sends ACK = 0 (slave should send another byte)
                                      // or NACK = 1 (no more bytes requested from slave)
    I2C2CONbits.ACKDT = val;          // store ACK/NACK in ACKDT
    I2C2CONbits.ACKEN = 1;            // send ACKDT
    while(I2C2CONbits.ACKEN) { ; }    // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) {          // send a STOP:
  I2C2CONbits.PEN = 1;                // comm is complete and master relinquishes bus
  while(I2C2CONbits.PEN) { ; }        // wait for STOP to complete
}

//////////////////////////////// IMU ///////////////////////////////////////////
void imu_setup(void){
    // accelerometer set up
    //  Set the sample rate to 1.66 kHz, 2g sensitivity, x filter.
    unsigned char xl_setup = 0b10000000;
    i2c_master_start();
    i2c_master_send(IMU_ADDR);
    i2c_master_send(CTRL1_XL);
    i2c_master_send(xl_setup);
    i2c_master_stop();

    // gyroscope set up
    // ample rate to 1.66 kHz, 245 dps sensitivity, x filter.
    unsigned char g_setup = 0b10000000;
    i2c_master_start();
    i2c_master_send(IMU_ADDR);
    i2c_master_send(CTRL2_G);
    i2c_master_send(g_setup);

    // multiple read set up
    // if_inc bit must be 1 to enable
    unsigned char read_setup = 0b00000100;
    i2c_master_start();
    i2c_master_send(IMU_ADDR);
    i2c_master_send(CTRL3_C);
    i2c_master_send(read_setup);
    i2c_master_stop();
}

// currently written to read from the who_am_i register
unsigned char I2C_read_single(void){
    i2c_master_start();
    i2c_master_send(IMU_ADDR);
    i2c_master_send(WHO_AM_I); // read from the who_am_i register to get logic
    i2c_master_restart();
    i2c_master_send(0b11010111); // send the read command, 1 lsb means read
    unsigned char r = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    return r;
}

void I2C_read_multiple(char address, char reg, unsigned char * data, char length){
    i2c_master_start();
    i2c_master_send((address << 1)); // shift address 1 -- 0 in lsb = write
    i2c_master_send(reg); // this should be out_temp_L
    i2c_master_restart();
    i2c_master_send((address<< 1)| 0x01); // put a 1 in lsb = read

    int i;
    for (i = 0; i < length; i++){ // go through the 14 registers, starting at out_temp_l
        outputs[i] = i2c_master_recv();
        if((i+1) == length){ // statement should be true for last iteration
            i2c_master_ack(1); // or NACK = 1 (no more bytes requested from slave)
        }
        else{ // every iteration but last
            i2c_master_ack(0); // sends ACK = 0 (slave should send another byte)
        }
    }
    i2c_master_stop();
}

// takes character array to write to the LCD screen 
// number_char is number of individual characters in array
void LCD_drawStr(unsigned char x, unsigned char y, unsigned char * array, int number_char){
    int i = 0;
    while (array[i]){
        LCD_drawChar(x,y,array[i]);
        i++;
        x += 5; 
        if (i == number_char){
            break;
        }

    }
}

// write single character to lcd screen
void LCD_drawChar(unsigned char x, unsigned char y, unsigned char write_char){
    
    int i, j;
    unsigned char LCD_byte;
    
    unsigned char ascii_pos = write_char - 0x20;
    
    for (i = 0; i < 5; i++){
        LCD_byte = ASCII[ascii_pos][i];
        for (j = 7; j >= 0; j--){
            unsigned char color_bit = (LCD_byte >> j) & 1;
            if (color_bit == 1){
                LCD_drawPixel(x+i,y+j,WHITE);
            }
            else{
                LCD_drawPixel(x+i,y+j,BLACK);
            }
        }
    }
}

////////////////////////////////// PIC 32 INITIALIZATION ///////////////////////
void PIC32_init(void){
    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);
    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;
    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;
    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
}

/////////////////////////////// LCD SPI SETUP //////////////////////////////////
void SPI1_init() {
	SDI1Rbits.SDI1R = 0b0100; // B8 is SDI1
    RPA1Rbits.RPA1R = 0b0011; // A1 is SDO1
    TRISBbits.TRISB7 = 0; // SS is B7
    LATBbits.LATB7 = 1; // SS starts high

    // A0 / DAT pin
    ANSELBbits.ANSB15 = 0;
    TRISBbits.TRISB15 = 0;
    LATBbits.LATB15 = 0;
	
	SPI1CON = 0; // turn off the spi module and reset it
    SPI1BUF; // clear the rx buffer by reading from it
    SPI1BRG = 1; // baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0; // clear the overflow bit
    SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1; // master operation
    SPI1CONbits.ON = 1; // turn on spi1
}

unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void LCD_command(unsigned char com) {
    LATBbits.LATB15 = 0; // DAT
    LATBbits.LATB7 = 0; // CS
    spi_io(com);
    LATBbits.LATB7 = 1; // CS
}

void LCD_data(unsigned char dat) {
    LATBbits.LATB15 = 1; // DAT
    LATBbits.LATB7 = 0; // CS
    spi_io(dat);
    LATBbits.LATB7 = 1; // CS
}

void LCD_data16(unsigned short dat) {
    LATBbits.LATB15 = 1; // DAT
    LATBbits.LATB7 = 0; // CS
    spi_io(dat>>8);
    spi_io(dat);
    LATBbits.LATB7 = 1; // CS
}

void LCD_init() {
    int time = 0;
    LCD_command(CMD_SWRESET);//software reset
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/2) {} //delay(500);

	LCD_command(CMD_SLPOUT);//exit sleep
    time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/200) {} //delay(5);

	LCD_command(CMD_PIXFMT);//Set Color Format 16bit
	LCD_data(0x05);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/200) {} //delay(5);

	LCD_command(CMD_GAMMASET); // default gamma curve 3
	LCD_data(0x04);//0x04
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_GAMRSEL); // Enable Gamma adj
	LCD_data(0x01);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_NORML);

	LCD_command(CMD_DFUNCTR);
	LCD_data(0b11111111);
	LCD_data(0b00000110);

    int i = 0;
	LCD_command(CMD_PGAMMAC); // Positive Gamma Correction Setting
	for (i=0;i<15;i++){
		LCD_data(pGammaSet[i]);
	}

	LCD_command(CMD_NGAMMAC); // Negative Gamma Correction Setting
	for (i=0;i<15;i++){
		LCD_data(nGammaSet[i]);
	}

	LCD_command(CMD_FRMCTR1); // Frame Rate Control (In normal mode/Full colors)
	LCD_data(0x08);//0x0C//0x08
	LCD_data(0x02);//0x14//0x08
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_DINVCTR);//display inversion
	LCD_data(0x07);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_PWCTR1);//Set VRH1[4:0] & VC[2:0] for VCI1 & GVDD
	LCD_data(0x0A);//4.30 - 0x0A
	LCD_data(0x02);//0x05
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_PWCTR2);//Set BT[2:0] for AVDD & VCL & VGH & VGL
	LCD_data(0x02);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_VCOMCTR1);//Set VMH[6:0] & VML[6:0] for VOMH & VCOML
	LCD_data(0x50);//0x50
	LCD_data(99);//0x5b
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_VCOMOFFS);
	LCD_data(0);//0x40
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_CLMADRS);//Set Column Address
	LCD_data16(0x00);
    LCD_data16(_GRAMWIDTH);

	LCD_command(CMD_PGEADRS);//Set Page Address
	LCD_data16(0x00);
    LCD_data16(_GRAMHEIGH);

	LCD_command(CMD_VSCLLDEF);
	LCD_data16(0); // __OFFSET
	LCD_data16(_GRAMHEIGH); // _GRAMHEIGH - __OFFSET
	LCD_data16(0);

	LCD_command(CMD_MADCTL); // rotation
    LCD_data(0b00001000); // bit 3 0 for RGB, 1 for GBR, rotation: 0b00001000, 0b01101000, 0b11001000, 0b10101000

	LCD_command(CMD_DISPON);//display ON
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_RAMWR);//Memory Write
}

void LCD_drawPixel(unsigned short x, unsigned short y, unsigned short color) {
    // check boundary
    LCD_setAddr(x,y,x+1,y+1);
    LCD_data16(color);
}

void LCD_setAddr(unsigned short x0, unsigned short y0, unsigned short x1, unsigned short y1) {
    LCD_command(CMD_CLMADRS); // Column
    LCD_data16(x0);
	LCD_data16(x1);

	LCD_command(CMD_PGEADRS); // Page 
	LCD_data16(y0);
	LCD_data16(y1);

	LCD_command(CMD_RAMWR); //Into RAM
}

void LCD_clearScreen(unsigned short color) {
    int i;
    LCD_setAddr(0,0,_GRAMWIDTH,_GRAMHEIGH);
		for (i = 0;i < _GRAMSIZE; i++){
			LCD_data16(color);
		}
}

