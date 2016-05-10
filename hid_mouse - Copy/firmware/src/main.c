/*******************************************************************************
  MPLAB Harmony Project Main Source File

  Company:
    Microchip Technology Inc.
  
  File Name:
    main.c

  Summary:
    This file contains the "main" function for an MPLAB Harmony project.

  Description:
    This file contains the "main" function for an MPLAB Harmony project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state 
    machines of all MPLAB Harmony modules in the system and it calls the 
    "SYS_Tasks" function from within a system-wide "super" loop to maintain 
    their correct operation. These two functions are implemented in 
    configuration-specific files (usually "system_init.c" and "system_tasks.c")
    in a configuration-specific folder under the "src/system_config" folder 
    within this project's top-level folder.  An MPLAB Harmony project may have
    more than one configuration, each contained within it's own folder under
    the "system_config" folder.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

//Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "system/common/sys_module.h"   // SYS function prototypes
#include <stdio.h>
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
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

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

void PIC32_init(void);

void i2c_master_start(void);
void i2c_master_restart(void);
void i2c_master_send(unsigned char byte);
unsigned char i2c_master_recv(void);
void i2c_master_ack(int val);
void i2c_master_stop(void);
void i2c_master_setup(void);

void imu_setup(void);
unsigned char I2C_read_single(void);
void I2C_read_multiple(char, char, unsigned char *, char);

// global variables 
unsigned char outputs[14]; // length 14 b/c 14 registers to read from linearly
signed short temp = 0;
signed short g_x = 0, g_y = 0, g_z = 0;
signed short xl_x = 0, xl_y = 0, xl_z = 0;

int main ( void )
{
    /* Initialize all MPLAB Harmony modules, including application(s). */
    SYS_Initialize ( NULL );

    __builtin_disable_interrupts();

    PIC32_init();
    i2c_master_setup();
    imu_setup();

    __builtin_enable_interrupts();
    
    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        // APP_Tasks within SYS Tasks is where the mouse movement takes place
        // Place the IMU readings and conversions in APP_Tasks
        // Move mouse according to IMU
        // Turn APP_Tasks to take inputs?
        SYS_Tasks ( ); 

    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}

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
        data[i] = i2c_master_recv();
        if((i+1) == length){ // statement should be true for last iteration
            i2c_master_ack(1); // or NACK = 1 (no more bytes requested from slave)
        }
        else{ // every iteration but last
            i2c_master_ack(0); // sends ACK = 0 (slave should send another byte)
        }
    }
    i2c_master_stop();
}


/*******************************************************************************
 End of File
*/

