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
#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
#include "i2c.h"

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************
void PIC32_init(void);

int main ( void )
{
    /* Initialize all MPLAB Harmony modules, including application(s). */
    SYS_Initialize ( NULL );

    __builtin_disable_interrupts();

    PIC32_init();   

    __builtin_enable_interrupts();


    
    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );

    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
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

    /*
    // initializing the motor pins and stuff! 
    // yellow wires (B13, B9) = phase
    // white wires (B15, B8) = motor PWM
    
    ANSELBbits.ANSB13 = 0;
    int initial_duty = 0;
    
    // ------------ MOTOR DIRECTION ------------- // 
    // MUST BE TIED HIGH??
    TRISAbits.TRISA4 = 0; // Set green LED(A4) as output ON
    LATAbits.LATA4 = 0;
    
    // ------------ PWM FOR MOTORS -------------- //
    // PWM 1 --- USING OC1
    // CORRESPONDS TO THE "TOP" MOTOR
    ANSELBbits.ANSB15 = 0;
    RPB15Rbits.RPB15R = 0b0101; // OC1
    
    // standard oc skeleton code
    // set duty cycle initially to be 50%
    T2CONbits.TCKPS = 0b011;        // timer prescaler N = 8
    PR2 = 5999;                     // (PR2+1)N/48MHz --- value must be between 1k and 10k
    TMR2 = 0;                       // set timer2 to 0
    T2CONbits.ON = 1;               // turn on timer2

    // B15 = OC1
    OC1CONbits.OCTSEL = 0;          // set OC1 to use timer2
    OC1CONbits.OCM = 0b110;         // PWM mode without fault pin; other OC1CON bits are defaults
    OC1RS = initial_duty;                   // duty cycle = OC1RS/(PR2+1) = 50%
    OC1R = OC1RS;                    // OC1R for just in case it rolls over
    OC1CONbits.ON = 1;              // turn on OC1 
    
    // PWM 2 -- USING OC2
    // USE TIMER 2 FOR OC2 AS WELL
    
    RPB8Rbits.RPB8R = 0b0101; // OC2    
    T2CONbits.TCKPS = 0b011;        // timer prescaler N = 8
    PR2 = 5999;                     // (PR2+1)N/48MHz --- value must be between 1k and 10k
    TMR2 = 0;                       // set timer2 to 0
    T2CONbits.ON = 1;               // turn on timer2

    // B8 = OC2
    OC2CONbits.OCTSEL = 0;          // set OC2 to use timer2
    OC2CONbits.OCM = 0b110;         // PWM mode without fault pin; other OC2CON bits are defaults
    OC2RS = initial_duty;           // duty cycle = OC2RS/(PR2+1) = 50%
    OC2R = OC2RS;                   // OC2R for just in case it rolls over
    OC2CONbits.ON = 1;              // turn on OC2
    */
}

/*******************************************************************************
 End of File
*/  

