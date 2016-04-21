#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <stdio.h>
#include <math.h>
#define CS LATBbits.LATB7   // chip select pin
#define PI 3.14159265
// addresses used for i2c
#define IODIR 0x00
#define GPIO 0x09
#define OLAT 0x0A
#define DEV 0x40
#define IOPIN 0xF0

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

// function prototypes
void PIC32_init(void);

// spi functions
void setVoltage(unsigned char channel, unsigned char voltage);
void initSPI1(void);

// i2c functions
void i2c_master_setup(void);
void i2c_master_start(void);
void i2c_master_restart(void);
void i2c_master_send(unsigned char byte);
unsigned char i2c_master_recv(void);
void i2c_master_ack(int val);
void i2c_master_stop(void);
void i2c_expander(void);
void setExpander(unsigned char pin, char level);
unsigned char getExpander(void);

int main() {

    __builtin_disable_interrupts();

    PIC32_init();
    initSPI1();
    i2c_master_setup();
    i2c_expander();

    __builtin_enable_interrupts();

    // for debugging - make sure code is working
    // remember: TRIS = set I/O, LAT = write to bit
    TRISBbits.TRISB4 = 1; // Set pushbutton (B4) as input
    TRISAbits.TRISA4 = 0; // Set green LED(A4) as output ON
    LATAbits.LATA4 = 0;

    // initialize counter + main variables
    int plot_counter = 0;
    int i;
    double sine_wave[200];
    double tri_wave[200];

    // since DAC updating 1000 times a second and 10 Hz sine wave --- 100 points per period
    for (i = 0; i < 200; i++){
        // use the sine function in math.h
        // 200 iterations - two full sine periods, one triangle period
        // amplitude = 255/2, shifted up so max = 255 (3.3V) and min = 0 (0V))
        sine_wave[i] = (255.0/2.0)*sin((i/99.0)*2*PI) + (255.0/2.0);
        tri_wave[i] = 255.0*(i/199.0);
    }

    while(1) {

        _CP0_SET_COUNT(0); // core timer = 0, runs at half CPU speed
        while (_CP0_GET_COUNT() < 24000){;} // do nothing
        setVoltage(0,((unsigned char) sine_wave[plot_counter]));
        setVoltage(1,((unsigned char) tri_wave[plot_counter]));
        if (plot_counter == 199){
            plot_counter = 0;
        }
        else{
            plot_counter++;
        }

        if ((getExpander() & 0x80) == 0x80) {
            setExpander(0x01,1);
            LATAbits.LATA4 = !LATAbits.LATA4;
        }
        else {
            setExpander(0x01,0);
        }
    }
}

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

/////////////////////////////////// SPI ////////////////////////////////////////

void initSPI1(void) {
    // Set up the chip select pin as an output
    // when a command is beginning (clear CS to low) and when it
    // is ending (set CS high)

    RPB13Rbits.RPB13R = 0b0011; // B13 = SDO1
    SDI1Rbits.SDI1R = 0b0100;   // B8 = SDI1

    TRISBbits.TRISB7 = 0;       // use pin B7 for CHIP SELECT
    CS = 1; // Chip select line keep high

    // Setup SPI1
    SPI1CON = 0;              // turn off the spi module and reset it
    SPI1BUF;                  // clear the rx buffer by reading from it
    SPI1BRG = 1;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0;  // clear the overflow bit
    SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1;    // master operation
    SPI1CONbits.ON = 1;       // turn on SPI 1
}

// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void setVoltage(unsigned char channel, unsigned char voltage){
    // Channel represented by A = 0, B = 1
    // MCP4902 only has resolution of n = 8, so range on DAC input pin is 0 to 255

    short temp_command = 0x0000; // use this as temp for shifting over voltage
    temp_command = (temp_command | voltage) << 4; // shift over 4 because last 4 least significant bits are ignored

    short send_command = 0x7000; // buffered, gain of 1x, and active mode operation --
                                 // leave the first bit blank since it is the channel
    if (channel == 1){
        send_command |= 0x8000; // Most significant bit = 1 when channel is B
    }

    send_command |= temp_command;
//    short send_command = 0b1111011111110000; // test command -- should be half of 3.3V

    CS = 0;   // SS must be 0 -- SPI becomes the slave
    spi_io((send_command & 0xFF00) >> 8); // spi_io only takes char 8 bits, send the configuration bits first
    spi_io((send_command & 0x00FF));      // send the least significant byte
    CS = 1;   // raise the chip select line, ending communication
}
/////////////////////////////////// SPI ////////////////////////////////////////

/////////////////////////////////// I2C ///////////////////////////////////////
void i2c_master_setup(void) {
    // I2C operates at 100 kHz, 400Khz, and 1.7 MHz
    // Fsck = 100 kHz, PGD = 104 ns (the typical value, on page 269), Pblck = 48 MHz
    I2C2BRG = 233;            // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2
                                    // look up PGD for your PIC32
    I2C2CONbits.ON = 1;               // turn on the I2C2 module

    // I2C2 are analog inputs by default --- turn them off
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;

    // page 8 on pdf
    i2c_master_start();
    i2c_master_send(DEV); // first send the device opcode
    i2c_master_send(IODIR); // then send register address
    i2c_master_send(IOPIN); // write to the bits
    i2c_master_stop();
}

void i2c_expander(void){
    // port (GPIO) register sets either high or low
    i2c_master_start();
    i2c_master_send(DEV);
    i2c_master_send(GPIO);
    i2c_master_send(0x00); // set outputs as low first
    i2c_master_stop();
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

void setExpander(unsigned char pin, char level){
    unsigned char value_bits;
    if (level == 0){
        value_bits = 0x00;
    }
    else{
        value_bits = 0xFF;
    }
    value_bits = value_bits & pin;
    i2c_master_start();
    i2c_master_send(DEV);
    i2c_master_send(GPIO);
    i2c_master_send(value_bits);
    i2c_master_stop();
}

unsigned char getExpander(void) {
    i2c_master_start();
    i2c_master_send(DEV);
    i2c_master_send(GPIO); // read from the GPIO register to get logic
    i2c_master_restart();
    i2c_master_send(0b01000001); // send the read command
    unsigned char r = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    return r;
}

/////////////////////////////////// HOMEWORK 1 ////////////////////////////////
//
//	      // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
//		    // remember the core timer runs at half the CPU speed
//        // since CPU runs at 48 MHz, core timer runs at 24 MHz
//        _CP0_SET_COUNT(0); // core timer = 0, runs at half CPU speed
//
//        while (_CP0_GET_COUNT() < 12000){;} // 2000 counts = 0.5 ms
//        while (PORTBbits.RB4 == 0){;} // When low, input = 0 since switch tied to ground
//        LATAbits.LATA4 = !LATAbits.LATA4;
////        LATBbits.LATB7 = !LATBbits.LATB7;
//    }
//    return 0;
