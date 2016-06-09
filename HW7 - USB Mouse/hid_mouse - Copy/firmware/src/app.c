/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
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

#include "app.h"
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "system/common/sys_module.h"   // SYS function prototypes
#include <stdio.h>
#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
#include <math.h> // used in HW4
//#include "ILI9163C.h"

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


void PIC32_init(void);

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

// global variables 
unsigned char outputs[14]; // length 14 b/c 14 registers to read from linearly
signed short temp = 0;
signed short g_x = 0, g_y = 0, g_z = 0;
signed short xl_x = 0, xl_y = 0, xl_z = 0;

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

/* Mouse Report */
MOUSE_REPORT mouseReport APP_MAKE_BUFFER_DMA_READY;
MOUSE_REPORT mouseReportPrevious APP_MAKE_BUFFER_DMA_READY;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void APP_USBDeviceHIDEventHandler(USB_DEVICE_HID_INDEX hidInstance,
        USB_DEVICE_HID_EVENT event, void * eventData, uintptr_t userData)
{
    APP_DATA * appData = (APP_DATA *)userData;

    switch(event)
    {
        case USB_DEVICE_HID_EVENT_REPORT_SENT:

            /* This means the mouse report was sent.
             We are free to send another report */

            appData->isMouseReportSendBusy = false;
            break;

        case USB_DEVICE_HID_EVENT_REPORT_RECEIVED:

            /* Dont care for other event in this demo */
            break;

        case USB_DEVICE_HID_EVENT_SET_IDLE:

             /* Acknowledge the Control Write Transfer */
           USB_DEVICE_ControlStatus(appData->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            /* save Idle rate received from Host */
            appData->idleRate = ((USB_DEVICE_HID_EVENT_DATA_SET_IDLE*)eventData)->duration;
            break;

        case USB_DEVICE_HID_EVENT_GET_IDLE:

            /* Host is requesting for Idle rate. Now send the Idle rate */
            USB_DEVICE_ControlSend(appData->deviceHandle, &(appData->idleRate),1);

            /* On successfully receiving Idle rate, the Host would acknowledge back with a
               Zero Length packet. The HID function driver returns an event
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the application upon
               receiving this Zero Length packet from Host.
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates this control transfer
               event is complete */ 

            break;

        case USB_DEVICE_HID_EVENT_SET_PROTOCOL:
            /* Host is trying set protocol. Now receive the protocol and save */
            appData->activeProtocol = *(USB_HID_PROTOCOL_CODE *)eventData;

              /* Acknowledge the Control Write Transfer */
            USB_DEVICE_ControlStatus(appData->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case  USB_DEVICE_HID_EVENT_GET_PROTOCOL:

            /* Host is requesting for Current Protocol. Now send the Idle rate */
             USB_DEVICE_ControlSend(appData->deviceHandle, &(appData->activeProtocol), 1);

             /* On successfully receiving Idle rate, the Host would acknowledge
               back with a Zero Length packet. The HID function driver returns
               an event USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the
               application upon receiving this Zero Length packet from Host.
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates
               this control transfer event is complete */
             break;

        case USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT:
            break;

        default:
            break;
    }
}

/*******************************************************************************
  Function:
    void APP_USBDeviceEventHandler (USB_DEVICE_EVENT event,
        USB_DEVICE_EVENT_DATA * eventData)

  Summary:
    Event callback generated by USB device layer.

  Description:
    This event handler will handle all device layer events.

  Parameters:
    None.

  Returns:
    None.
*/

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context)
{
    USB_DEVICE_EVENT_DATA_CONFIGURED * configurationValue;
    switch(event)
    {
        case USB_DEVICE_EVENT_SOF:
            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            appData.setIdleTimer++;
            break;
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:
        
            /* Device got deconfigured */
            
            appData.isConfigured = false;
            appData.isMouseReportSendBusy = false;
            appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            appData.emulateMouse = true;
            BSP_LEDOn ( APP_USB_LED_1 );
            BSP_LEDOn ( APP_USB_LED_2 );
            BSP_LEDOff ( APP_USB_LED_3 );

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Device is configured */
            configurationValue = (USB_DEVICE_EVENT_DATA_CONFIGURED *)eventData;
            if(configurationValue->configurationValue == 1)
            {
                appData.isConfigured = true;
                
                BSP_LEDOff ( APP_USB_LED_1 );
                BSP_LEDOff ( APP_USB_LED_2 );
                BSP_LEDOn ( APP_USB_LED_3 );

                /* Register the Application HID Event Handler. */

                USB_DEVICE_HID_EventHandlerSet(appData.hidInstance,
                        APP_USBDeviceHIDEventHandler, (uintptr_t)&appData);
            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:
            BSP_LEDOff ( APP_USB_LED_1 );
            BSP_LEDOn ( APP_USB_LED_2 );
            BSP_LEDOn ( APP_USB_LED_3 );
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;

    } 
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/********************************************************
 * Application switch press routine
 ********************************************************/

void APP_ProcessSwitchPress(void)
{
    /* This function checks if the switch is pressed and then
     * debounces the switch press*/
    if(BSP_SWITCH_STATE_PRESSED == (BSP_SwitchStateGet(APP_USB_SWITCH_1)))
    {
        if(appData.ignoreSwitchPress)
        {
            /* This means the key press is in progress */
            if(appData.sofEventHasOccurred)
            {
                /* A timer event has occurred. Update the debounce timer */
                appData.switchDebounceTimer ++;
                appData.sofEventHasOccurred = false;
                if(appData.switchDebounceTimer == APP_USB_SWITCH_DEBOUNCE_COUNT)
                {
                    /* Indicate that we have valid switch press. The switch is
                     * pressed flag will be cleared by the application tasks
                     * routine. We should be ready for the next key press.*/
                    appData.isSwitchPressed = true;
                    appData.switchDebounceTimer = 0;
                    appData.ignoreSwitchPress = false;
                }
            }
        }
        else
        {
            /* We have a fresh key press */
            appData.ignoreSwitchPress = true;
            appData.switchDebounceTimer = 0;
        }
    }
    else
    {
        /* No key press. Reset all the indicators. */
        appData.ignoreSwitchPress = false;
        appData.switchDebounceTimer = 0;
        appData.sofEventHasOccurred = false;
    }
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    appData.deviceHandle  = USB_DEVICE_HANDLE_INVALID;
    appData.isConfigured = false;
    appData.emulateMouse = true;
    appData.hidInstance = 0;
    appData.isMouseReportSendBusy = false;
    appData.isSwitchPressed = false;
    appData.ignoreSwitchPress = false;
    
    
    PIC32_init();
    i2c_master_setup();
    imu_setup();
    
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void ) // move based on just xl_x and xl_y for now 
{
    static int8_t   vector = 0;
    static uint8_t  movement_length = 0;
    static bool     sent_dont_move = false;
    
    short var1, var2;
    _CP0_SET_COUNT(0); // core timer = 0, runs at half CPU speed
    while (_CP0_GET_COUNT() < 240000){;} // read at 50 Hz -- 480k / 24 MHz
    I2C_read_multiple(IMU_7bit, OUT_TEMP_L, outputs, 14); // multiple reads
    // data is read from OUT_TEMP_L, moving "upward" -- 14 registers total
//    temp = (outputs[0] | (outputs[1] << 8));
//    g_x = (outputs[2] | (outputs[3] << 8));
//    g_y = (outputs[4] | (outputs[5] << 8));
//    g_z = (outputs[6] | (outputs[7] << 8));
    xl_x = (outputs[8] | (outputs[9] << 8));
    xl_y = (outputs[10] | (outputs[11] << 8));
//    xl_z = (outputs[14] | (outputs[13] << 8));
    
    var1 = xl_x; 
    var2 = xl_y;
    
    

//    int8_t dir_table[] ={-4,-4,-4, 0, 4, 4, 4, 0};
    
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
		    /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0,
                    DRV_IO_INTENT_READWRITE );

            if(appData.deviceHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle,
                        APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }
            break;
        }

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device is configured. The 
             * isConfigured flag is updated in the
             * Device Event Handler */

            if(appData.isConfigured)
            {
                appData.state = APP_STATE_MOUSE_EMULATE;
            }
            break;

        case APP_STATE_MOUSE_EMULATE:

            APP_ProcessSwitchPress();

            /* The following logic rotates the mouse icon when
             * a switch is pressed */

            if(appData.isSwitchPressed)
            {
                /* Toggle the mouse emulation with each switch press */
                appData.emulateMouse ^= 1;
                appData.isSwitchPressed = false;
            }
            
            appData.emulateMouse = 1; // make sure it runs into this loop 
            
            if(appData.emulateMouse)
            {
                sent_dont_move = false;

//                if(movement_length > 50)
                if(movement_length > 10) // decrease the movement length so less delay
                {
                    appData.mouseButton[0] = MOUSE_BUTTON_STATE_RELEASED;
                    appData.mouseButton[1] = MOUSE_BUTTON_STATE_RELEASED;
//                    appData.xCoordinate =(int8_t)dir_table[vector & 0x07];
//                    appData.yCoordinate =(int8_t)dir_table[(vector+2) & 0x07];
                    // swapped var2 and var1 so that y on gyro = horizontal on screen
                    appData.xCoordinate = (int8_t)(var2/1000);
                    appData.yCoordinate = (int8_t)(-var1/1000); // value represents sensitivity
                    vector ++;
                    movement_length = 0;
                }
            }
            else
            { 
                appData.mouseButton[0] = MOUSE_BUTTON_STATE_RELEASED;
                appData.mouseButton[1] = MOUSE_BUTTON_STATE_RELEASED;
                appData.xCoordinate = 0;
                appData.yCoordinate = 0;
            }

            if(!appData.isMouseReportSendBusy)
            {
                if(((sent_dont_move == false) && (!appData.emulateMouse)) || (appData.emulateMouse))
                {

                    /* This means we can send the mouse report. The
                       isMouseReportBusy flag is updated in the HID Event Handler. */

                    appData.isMouseReportSendBusy = true;

                    /* Create the mouse report */

                    MOUSE_ReportCreate(appData.xCoordinate, appData.yCoordinate,
                            appData.mouseButton, &mouseReport);

                    if(memcmp((const void *)&mouseReportPrevious, (const void *)&mouseReport,
                            (size_t)sizeof(mouseReport)) == 0)
                    {
                        /* Reports are same as previous report. However mouse reports
                         * can be same as previous report as the co-ordinate positions are relative.
                         * In that case it needs to be send */
                        if((appData.xCoordinate == 0) && (appData.yCoordinate == 0))
                        {
                            /* If the coordinate positions are 0, that means there
                             * is no relative change */
                            if(appData.idleRate == 0)
                            {
                                appData.isMouseReportSendBusy = false;
                            }
                            else
                            {
                                /* Check the idle rate here. If idle rate time elapsed
                                 * then the data will be sent. Idle rate resolution is
                                 * 4 msec as per HID specification; possible range is
                                 * between 4msec >= idlerate <= 1020 msec.
                                 */
                                if(appData.setIdleTimer * APP_USB_CONVERT_TO_MILLISECOND
                                        >= appData.idleRate * 4)
                                {
                                    /* Send REPORT as idle time has elapsed */
                                    appData.isMouseReportSendBusy = true;
                                }
                                else
                                {
                                    /* Do not send REPORT as idle time has not elapsed */
                                    appData.isMouseReportSendBusy = false;
                                }
                            }
                        }

                    }
                    if(appData.isMouseReportSendBusy == true)
                    {
                        /* Copy the report sent to previous */
                        memcpy((void *)&mouseReportPrevious, (const void *)&mouseReport,
                                (size_t)sizeof(mouseReport));
                        
                        /* Send the mouse report. */
                        USB_DEVICE_HID_ReportSend(appData.hidInstance,
                            &appData.reportTransferHandle, (uint8_t*)&mouseReport,
                            sizeof(MOUSE_REPORT));
                        appData.setIdleTimer = 0;
                    }
                    movement_length ++;
                    sent_dont_move = true;
                }
            }

            break;

        case APP_STATE_ERROR:

            break;

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
 
/************************** TERENCE'S CODE ***********************************/
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

