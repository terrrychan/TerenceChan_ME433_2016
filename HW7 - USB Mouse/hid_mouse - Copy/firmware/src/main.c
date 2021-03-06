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
#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
#include <math.h> // used in HW4
//#include "ILI9163C.h"

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

//void SPI1_init(void);
//void LCD_drawChar(unsigned char, unsigned char, char);
//void LCD_drawStr(unsigned char, unsigned char, char *, int);
//void LCD_init(void);

// LCD function prototypes included in the header file


//// IMU functions
//void imu_setup(void);
//unsigned char I2C_read_single(void);
//void I2C_read_multiple(char, char, unsigned char *, char);


//// global variables 
//unsigned char outputs[14]; // length 14 b/c 14 registers to read from linearly
//signed short temp = 0;
//signed short g_x = 0, g_y = 0, g_z = 0;
//signed short xl_x = 0, xl_y = 0, xl_z = 0;
//char write_string[100];

int main ( void )
{
    /* Initialize all MPLAB Harmony modules, including application(s). */
    SYS_Initialize ( NULL );

    
    
//    unsigned char x_start = 28; 
//    unsigned char y_start = 32;
//    int write_var = 1337;
//    int total_char = sprintf(write_string, "Hello blal %d", write_var);
    

    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        // APP_Tasks within SYS Tasks is where the mouse movement takes place
        // send app tasks the IMU readings 
        // app tasks changes coordinates accordingly
        
//        _CP0_SET_COUNT(0); // core timer = 0, runs at half CPU speed
//        while (_CP0_GET_COUNT() < 240000){;} // read at 50 Hz -- 480k / 24 MHz
//        I2C_read_multiple(IMU_7bit, OUT_TEMP_L, outputs, 14); // multiple reads
//        // data is read from OUT_TEMP_L, moving "upward" -- 14 registers total
//        temp = (outputs[0] | (outputs[1] << 8));
//        g_x = (outputs[2] | (outputs[3] << 8));
//        g_y = (outputs[4] | (outputs[5] << 8));
//        g_z = (outputs[6] | (outputs[7] << 8));
//        xl_x = (outputs[8] | (outputs[9] << 8));
//        xl_y = (outputs[10] | (outputs[11] << 8));
//        xl_z = (outputs[14] | (outputs[13] << 8));

        SYS_Tasks ( ); 
//        LCD_drawStr(x_start,y_start,write_string,total_char);


    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}



///*************************8
// */ 
//// takes character array to write to the LCD screen 
//// number_char is number of individual characters in array
//void LCD_drawStr(unsigned char x, unsigned char y, char * array, int number_char){
//    int i = 0;
//    while (array[i]){
//        LCD_drawChar(x,y,array[i]);
//        i++;
//        x += 5; 
//        if (i == number_char){
//            break;
//        }
//
//    }
//}
//
//// write single character to lcd screen
//void LCD_drawChar(unsigned char x, unsigned char y, char write_char){
//    
//    int i, j;
//    char LCD_byte;
//    
//    unsigned char ascii_pos = write_char - 0x20;
//    
//    for (i = 0; i < 5; i++){
//        LCD_byte = ASCII[ascii_pos][i];
//        for (j = 7; j >= 0; j--){
//            unsigned char color_bit = (LCD_byte >> j) & 1;
//            if (color_bit == 1){
//                LCD_drawPixel(x+i,y+j,WHITE);
//            }
//            else{
//                LCD_drawPixel(x+i,y+j,BLACK);
//            }
//        }
//    }
//}
//
///////////////////////////////// LCD SPI SETUP //////////////////////////////////
//void SPI1_init() {
//	SDI1Rbits.SDI1R = 0b0100; // B8 is SDI1
//    RPA1Rbits.RPA1R = 0b0011; // A1 is SDO1
//    TRISBbits.TRISB7 = 0; // SS is B7
//    LATBbits.LATB7 = 1; // SS starts high
//
//    // A0 / DAT pin
//    ANSELBbits.ANSB15 = 0;
//    TRISBbits.TRISB15 = 0;
//    LATBbits.LATB15 = 0;
//	
//	SPI1CON = 0; // turn off the spi module and reset it
//    SPI1BUF; // clear the rx buffer by reading from it
//    SPI1BRG = 1; // baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
//    SPI1STATbits.SPIROV = 0; // clear the overflow bit
//    SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
//    SPI1CONbits.MSTEN = 1; // master operation
//    SPI1CONbits.ON = 1; // turn on spi1
//}
//
//unsigned char spi_io(unsigned char o) {
//  SPI1BUF = o;
//  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
//    ;
//  }
//  return SPI1BUF;
//}
//
//void LCD_command(unsigned char com) {
//    LATBbits.LATB15 = 0; // DAT
//    LATBbits.LATB7 = 0; // CS
//    spi_io(com);
//    LATBbits.LATB7 = 1; // CS
//}
//
//void LCD_data(unsigned char dat) {
//    LATBbits.LATB15 = 1; // DAT
//    LATBbits.LATB7 = 0; // CS
//    spi_io(dat);
//    LATBbits.LATB7 = 1; // CS
//}
//
//void LCD_data16(unsigned short dat) {
//    LATBbits.LATB15 = 1; // DAT
//    LATBbits.LATB7 = 0; // CS
//    spi_io(dat>>8);
//    spi_io(dat);
//    LATBbits.LATB7 = 1; // CS
//}
//
//void LCD_init() {
//    int time = 0;
//    LCD_command(CMD_SWRESET);//software reset
//    time = _CP0_GET_COUNT();
//    while (_CP0_GET_COUNT() < time + 48000000/2/2) {} //delay(500);
//
//	LCD_command(CMD_SLPOUT);//exit sleep
//    time = _CP0_GET_COUNT();
//	while (_CP0_GET_COUNT() < time + 48000000/2/200) {} //delay(5);
//
//	LCD_command(CMD_PIXFMT);//Set Color Format 16bit
//	LCD_data(0x05);
//	time = _CP0_GET_COUNT();
//	while (_CP0_GET_COUNT() < time + 48000000/2/200) {} //delay(5);
//
//	LCD_command(CMD_GAMMASET); // default gamma curve 3
//	LCD_data(0x04);//0x04
//	time = _CP0_GET_COUNT();
//	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);
//
//	LCD_command(CMD_GAMRSEL); // Enable Gamma adj
//	LCD_data(0x01);
//	time = _CP0_GET_COUNT();
//	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);
//
//	LCD_command(CMD_NORML);
//
//	LCD_command(CMD_DFUNCTR);
//	LCD_data(0b11111111);
//	LCD_data(0b00000110);
//
//    int i = 0;
//	LCD_command(CMD_PGAMMAC); // Positive Gamma Correction Setting
//	for (i=0;i<15;i++){
//		LCD_data(pGammaSet[i]);
//	}
//
//	LCD_command(CMD_NGAMMAC); // Negative Gamma Correction Setting
//	for (i=0;i<15;i++){
//		LCD_data(nGammaSet[i]);
//	}
//
//	LCD_command(CMD_FRMCTR1); // Frame Rate Control (In normal mode/Full colors)
//	LCD_data(0x08);//0x0C//0x08
//	LCD_data(0x02);//0x14//0x08
//	time = _CP0_GET_COUNT();
//	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);
//
//	LCD_command(CMD_DINVCTR);//display inversion
//	LCD_data(0x07);
//	time = _CP0_GET_COUNT();
//	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);
//
//	LCD_command(CMD_PWCTR1);//Set VRH1[4:0] & VC[2:0] for VCI1 & GVDD
//	LCD_data(0x0A);//4.30 - 0x0A
//	LCD_data(0x02);//0x05
//	time = _CP0_GET_COUNT();
//	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);
//
//	LCD_command(CMD_PWCTR2);//Set BT[2:0] for AVDD & VCL & VGH & VGL
//	LCD_data(0x02);
//	time = _CP0_GET_COUNT();
//	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);
//
//	LCD_command(CMD_VCOMCTR1);//Set VMH[6:0] & VML[6:0] for VOMH & VCOML
//	LCD_data(0x50);//0x50
//	LCD_data(99);//0x5b
//	time = _CP0_GET_COUNT();
//	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);
//
//	LCD_command(CMD_VCOMOFFS);
//	LCD_data(0);//0x40
//	time = _CP0_GET_COUNT();
//	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);
//
//	LCD_command(CMD_CLMADRS);//Set Column Address
//	LCD_data16(0x00);
//    LCD_data16(_GRAMWIDTH);
//
//	LCD_command(CMD_PGEADRS);//Set Page Address
//	LCD_data16(0x00);
//    LCD_data16(_GRAMHEIGH);
//
//	LCD_command(CMD_VSCLLDEF);
//	LCD_data16(0); // __OFFSET
//	LCD_data16(_GRAMHEIGH); // _GRAMHEIGH - __OFFSET
//	LCD_data16(0);
//
//	LCD_command(CMD_MADCTL); // rotation
//    LCD_data(0b00001000); // bit 3 0 for RGB, 1 for GBR, rotation: 0b00001000, 0b01101000, 0b11001000, 0b10101000
//
//	LCD_command(CMD_DISPON);//display ON
//	time = _CP0_GET_COUNT();
//	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);
//
//	LCD_command(CMD_RAMWR);//Memory Write
//}
//
//void LCD_drawPixel(unsigned short x, unsigned short y, unsigned short color) {
//    // check boundary
//    LCD_setAddr(x,y,x+1,y+1);
//    LCD_data16(color);
//}
//
//void LCD_setAddr(unsigned short x0, unsigned short y0, unsigned short x1, unsigned short y1) {
//    LCD_command(CMD_CLMADRS); // Column
//    LCD_data16(x0);
//	LCD_data16(x1);
//
//	LCD_command(CMD_PGEADRS); // Page 
//	LCD_data16(y0);
//	LCD_data16(y1);
//
//	LCD_command(CMD_RAMWR); //Into RAM
//}
//
//void LCD_clearScreen(unsigned short color) {
//    int i;
//    LCD_setAddr(0,0,_GRAMWIDTH,_GRAMHEIGH);
//		for (i = 0;i < _GRAMSIZE; i++){
//			LCD_data16(color);
//		}
//}



/*******************************************************************************
 End of File
*/

