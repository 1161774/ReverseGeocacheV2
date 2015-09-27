//*****************************************************************************
//
// project.c - Simple project to use as a starting point for more complex
//             projects.
//
// Copyright (c) 2013-2015 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.1.1.71 of the Tiva Firmware Development Package.
//
//*****************************************************************************


/***********************************************************************************
 * 	Reverse Geocache
 * 	Connections:
 *
 * 		PC4 <-- GPS-Rx
 * 		PC5 --> GPS-Tx
 *		(UART1)
 *
 * 		PD0 <-- PushButton
 * 		PD1 --> GPS Power
 * 		PD2 --> LCD Backlight power
 * 		PD3 --> Lock	??
 * 		PD6 --> Unlock	??
 *
 * 		PE4 <-> I2C SCL
 * 		PE5 <-> I2C SDA
 *		(I2C2)
 *
 *********************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <math.h>
#include <inttypes.h>

#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.c"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "MCCOG21605B6W-SPTLYI.h"

#define LED_RED		GPIO_PIN_1
#define LED_BLUE	GPIO_PIN_2
#define	LED_GREEN	GPIO_PIN_3

#define LED_BASE	GPIO_PORTF_BASE

#define PUSHBUTTON	GPIO_PIN_0
#define GPS_POWER	GPIO_PIN_1
#define LCD_POWER	GPIO_PIN_2
#define LOCK_LID	GPIO_PIN_3
#define UNLOCK_LID	GPIO_PIN_6

#define PUSHBUTTON_BASE GPIO_PORTD_BASE
#define GPS_POWER_BASE  GPIO_PORTD_BASE
#define LCD_POWER_BASE  GPIO_PORTD_BASE
#define LOCK_LID_BASE   GPIO_PORTD_BASE
#define UNLOCK_LID_BASE GPIO_PORTD_BASE


#define CMDLOCK		1
#define CMDUNLOCK	2
#define STLOCKED	1
#define STUNLOCKED	2

uint32_t g_LockStatus = STUNLOCKED;

#define LCDON		1
#define LCDOFF		2

#define GPSON		1
#define GPSOFF		2

struct _GPS_GPGGA{
	float UTCTime;
	float Latitude;
	char NSIndicator;
	float Longitude;
	char EWIndicator;
	unsigned int Quality;
	unsigned int Satellites;
	float HDOP;
	float Altitude;
	char AltUnits;	//will be m for metres
	float GeoidHeight;
	char GUnits;	//will be m for metres
	float DGPSTime;	//will be blank
	float DGPSID;	//will be blank
};
struct _GPS_GPGGA GPS_GPGGA;


struct _GeoData{
	float Latitude;
	float Longitude;
	float TargetLatitude;
	float TargetLongitude;
	float Range;
};
struct _GeoData GeoData;


#define MESSAGE_LENGTH 		100
#define GPS_MESSAGE_LENGTH	200

struct _GPSReadBuffer
{
	char RxBuffer[GPS_MESSAGE_LENGTH];
	uint32_t ReadIndex;
	bool StartOfMessage;
	bool MessageReady;
};
struct _GPSReadBuffer GPSRead;


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD 			M_PI/180


//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Simple Project (project)</h1>
//!
//! A very simple example that can be used as a starting point for more complex
//! projects.  Most notably, this project is fully TI BSD licensed, so any and
//! all of the code (including the startup code) can be used as allowed by that
//! license.
//!
//! The provided code simply toggles a GPIO using the Tiva Peripheral Driver
//! Library.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART0(void)
{
	//
	// Enable the GPIO Peripheral used by the UART.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
	// Enable UART0
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	//
	// Configure GPIO Pins for UART mode.
	//
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// Use the internal 16MHz oscillator as the UART clock source.
	//
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

	//
	// Initialize the UART for console I/O.
	//
	UARTStdioConfig(0, 115200, 16000000);
}

void
ConfigureUART1(void)
{
	//
	// Enable the GPIO Peripheral used by the UART.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	//
	// Enable UART1
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

	//
	// Configure GPIO Pins for UART mode.
	//
	ROM_GPIOPinConfigure(GPIO_PC4_U1RX);
	ROM_GPIOPinConfigure(GPIO_PC5_U1TX);
	ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

	//
	// Use the internal 16MHz oscillator as the UART clock source.
	//
	ROM_UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);

    //
    // Configure the UART for 9600, 8-N-1 operation.
    //
    UARTConfigSetExpClk(UART1_BASE, 16000000, 9600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    ROM_IntEnable(INT_UART1);
    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX);

}

void I2C2Configure()
{

	//
	// The I2C0 peripheral must be enabled before use.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

	//
	// The GPIO_E peripheral must be enabled before use.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	//
	// Configure the pin muxing for I2C2 functions on port E4 and E5.
	//
	GPIOPinConfigure(GPIO_PE4_I2C2SCL);
	GPIOPinConfigure(GPIO_PE5_I2C2SDA);

	//
	// Select the I2C function for these pins.  This function will also
	// configure the GPIO pins pins for I2C operation, setting them to
	// open-drain operation with weak pull-ups.  Consult the data sheet
	// to see which functions are allocated per pin.
	//
	GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
	GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);

	//
	// Enable and initialize the I2C2 master module.  Use the system clock for
	// the I2C2 module.  The last parameter sets the I2C data transfer rate.
	// If false the data rate is set to 100kbps and if true the data rate will
	// be set to 400kbps.  For this example we will use a data rate of 100kbps.
	//
	I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);

	//
	// Tell the master module what address it will place on the bus when
	// communicating with the slave.  Set the address to SLAVE_ADDRESS
	// (as set in the slave module).  The receive parameter is set to false
	// which indicates the I2C Master is initiating a writes to the slave.  If
	// true, that would indicate that the I2C Master is initiating reads from
	// the slave.
	//
	I2CMasterSlaveAddrSet(I2C2_BASE, SLAVE_ADDRESS, false);


}





//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UART1IntHandler(void)
{
    uint32_t ui32Status;
    char receiveChar;
    // Get the interrrupt status.
    ui32Status = ROM_UARTIntStatus(UART1_BASE, true);

    // Clear the asserted interrupts.
    ROM_UARTIntClear(UART1_BASE, ui32Status);

//    UARTprintf("GPSint\n");

    // Loop while there are characters in the receive FIFO, and there isn't already a message pending read.
    while(ROM_UARTCharsAvail(UART1_BASE) && !GPSRead.MessageReady)
    {
    	// Read a character from the FIFO.
    	receiveChar = UARTCharGetNonBlocking(UART1_BASE);

    	// All GPS Messages start with a '$' symbol.
    	if(receiveChar == '$')
    	{
    		// Get ready to read...
    		GPSRead.StartOfMessage = true;
    		GPSRead.ReadIndex = 0;
    	}

    	if(GPSRead.StartOfMessage)
    	{
			// We should have read enough data to see if the GPS unit has transmitted the message that we're interested in.
    		if(GPSRead.ReadIndex == 6)
    		{
        		// Peek to see if it's a GPGGA message...
    			if(!(GPSRead.RxBuffer[3] == 'G' && GPSRead.RxBuffer[4] == 'G' && GPSRead.RxBuffer[5] == 'A'))
    			{
    				// It's not, don't worry about reading the rest of the message.
    				GPSRead.StartOfMessage = false;
    			}
    		}

    		// Chuck the character into the buffer.
    		GPSRead.RxBuffer[GPSRead.ReadIndex++] = receiveChar;

    		// Check for the end of the message, or the buffer is full.
    		if(receiveChar == 0x0D ||  receiveChar == 0x0A || GPSRead.ReadIndex >= GPS_MESSAGE_LENGTH)
    		{
    			// Stop reading, signal to the program that a new message is ready to be parsed.
    			GPSRead.StartOfMessage = false;
    			GPSRead.MessageReady = true;
    		}
    	}
    }
}



void parseString()
{

	char message[MESSAGE_LENGTH];
	uint32_t i, commaCount;
	bool goodCopy;

	for(i=0;i<MESSAGE_LENGTH;i++)
	{
		message[i] = 0;
	}

	UARTprintf("Parsing GPS string...");
	goodCopy = false;
	i=0;
	commaCount = 0;
	while(true){
		message[i] = GPSRead.RxBuffer[i+7];
		i++;
		if (GPSRead.RxBuffer[i+7] == '*')
		{
			goodCopy = (commaCount == 13);
			break;
		}
		else if(GPSRead.RxBuffer[i+7] == ',')
		{
			commaCount++;
		}
		else if (GPSRead.RxBuffer[i+7] == '*' || i >= MESSAGE_LENGTH)
		{
			goodCopy = false;
			break;
		}
	}



	if(goodCopy)
	{
		UARTprintf("Success\n");
//		UARTprintf("%s",message);
		sscanf(message,"%f,%f,%c,%f,%c,%d,%d,%f,%f,%c,%f,%c,%f,%f",
				&(GPS_GPGGA.UTCTime),
				&(GPS_GPGGA.Latitude),
				&(GPS_GPGGA.NSIndicator),
				&(GPS_GPGGA.Longitude),
				&(GPS_GPGGA.EWIndicator),
				&(GPS_GPGGA.Quality),
				&(GPS_GPGGA.Satellites),
				&(GPS_GPGGA.HDOP),
				&(GPS_GPGGA.Altitude),
				&(GPS_GPGGA.AltUnits),
				&(GPS_GPGGA.GeoidHeight),
				&(GPS_GPGGA.GUnits),
				&(GPS_GPGGA.DGPSTime),
				&(GPS_GPGGA.DGPSID));
	}
	else
	{
		UARTprintf("Failed\n");
	}
}


void BoxLock(uint32_t Cmd)
{
	if(Cmd == CMDLOCK && g_LockStatus == STUNLOCKED)
	{
		g_LockStatus = STLOCKED;
		GPIOPinWrite(LOCK_LID_BASE,LOCK_LID,0);
	    UARTprintf("****** BOX LOCKED ******\n");
		SysCtlDelay(SysCtlClockGet()/12);
		GPIOPinWrite(LOCK_LID_BASE,LOCK_LID,LOCK_LID);
	}
	else if(Cmd == CMDUNLOCK && g_LockStatus == STLOCKED)
	{
		g_LockStatus = STUNLOCKED;
		GPIOPinWrite(UNLOCK_LID_BASE,UNLOCK_LID,0);
		UARTprintf("***** BOX UNLOCKED *****\n");
		SysCtlDelay(SysCtlClockGet()/12);
		GPIOPinWrite(UNLOCK_LID_BASE,UNLOCK_LID,UNLOCK_LID);
	}
}

void LCDBacklight(uint32_t Cmd)
{
	if(Cmd == LCDON)
	{
		GPIOPinWrite(LCD_POWER_BASE, LCD_POWER, 0);
	    UARTprintf("LCD backlight powered on\n");
	}
	else if(Cmd == LCDOFF)
	{
		GPIOPinWrite(LCD_POWER_BASE, LCD_POWER, LCD_POWER);
	    UARTprintf("LCD backlight powered off\n");
	}
}

void GPSPower(uint32_t Cmd)
{
	if(Cmd == GPSON)
	{
	    GPIOPinWrite(GPS_POWER_BASE, GPS_POWER, 0);
	    UARTprintf("GPS powered on\n");
	}
	else if(Cmd == GPSOFF)
	{
		GPIOPinWrite(GPS_POWER_BASE, GPS_POWER, GPS_POWER);
	    UARTprintf("GPS powered off\n");
	}
}

int
main(void)
{

	//
	// Set the clocking to run from the PLL at 80MHz
	//
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
			SYSCTL_XTAL_16MHZ);

	//
    // Enable the GPIO module.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Configure inputs.
    //
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, PUSHBUTTON);
    ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, PUSHBUTTON, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // Configure outputs.
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPS_POWER|LCD_POWER|LOCK_LID|UNLOCK_LID);
    ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, GPS_POWER|LCD_POWER|LOCK_LID|UNLOCK_LID, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);

    ROM_GPIOPinTypeGPIOOutput(LED_BASE, LED_RED|LED_GREEN|LED_BLUE);
	GPIOPinWrite(LED_BASE,LED_RED|LED_GREEN|LED_BLUE,0);
	//
	// Open UARTS.
	//
    ConfigureUART0();
    UARTprintf("\033[2JReverse Geocache\n");
    UARTprintf("---------------------------------\n\n");

    UARTprintf("Opening GPS Comms\n\n");
    ConfigureUART1();

	//
	// Open I2C and LCD.
	//
    UARTprintf("Opening LCD Comms\n\n");
    I2C2Configure();
    UARTprintf("Configuring LCD\n\n");
    LCDConfigure(I2C2_BASE);
	LCDBacklight(LCDON);

	char string1[] = "@Aquiring Lock...";
	char string2[] = "@Please Wait     ";
	char lcdMessage[17];

    LCDTransmitString(I2C2_BASE, strlen(string1), string1);
    LCDCommand(I2C2_BASE, LCD_ROW_2);
    LCDTransmitString(I2C2_BASE, strlen(string2), string2);

    g_LockStatus = STUNLOCKED;
    BoxLock(CMDLOCK);


    //
    // Enable processor interrupts.
    //

//    UARTprintf("Enabling interrupts\n\n");
    ROM_IntMasterEnable();

    GPSPower(GPSON);

    bool gpsLockedPrev = false, gpsLocked = false;

    //
    // Loop forever.
    //
    while(1)
    {

    	if(GPSRead.MessageReady)
    	{

#ifdef DEBUG
    		UARTprintf("%s\n", GPSRead.RxBuffer);
#endif

    		parseString();
    		GPSRead.MessageReady = false;
    	}

    	gpsLockedPrev = gpsLocked;
    	gpsLocked = (GPS_GPGGA.Latitude > 0.0 && GPS_GPGGA.Longitude > 0.0);

    	if(gpsLocked && !gpsLockedPrev)
    	{
    		UARTprintf("GPS lock aquired\n");
    	    LCDCommand(I2C2_BASE, LCD_CLEAR_DISPLAY);
    	    sprintf(lcdMessage,"%10.7f%s",GPS_GPGGA.Latitude,GPS_GPGGA.NSIndicator);
    	    LCDTransmitString(I2C2_BASE, strlen(lcdMessage), lcdMessage);
    	    LCDCommand(I2C2_BASE, LCD_ROW_2);
    	    sprintf(lcdMessage,"%10.7f%s",GPS_GPGGA.Longitude,GPS_GPGGA.EWIndicator);
    	    LCDTransmitString(I2C2_BASE, strlen(lcdMessage), lcdMessage);
    	}
    	else if(!gpsLocked && gpsLockedPrev)
    	{
    		UARTprintf("GPS lock lost\n");
    	    LCDCommand(I2C2_BASE, LCD_CLEAR_DISPLAY);
    	    LCDTransmitString(I2C2_BASE, strlen(string1), string1);
    	    LCDCommand(I2C2_BASE, LCD_ROW_2);
    	    LCDTransmitString(I2C2_BASE, strlen(string2), string2);

    	}

        if(GPIOPinRead(PUSHBUTTON_BASE, PUSHBUTTON))
        {
        	BoxLock(CMDUNLOCK);
 //       	LCDBacklight(LCDON);

        }
        else
        {
        	BoxLock(CMDLOCK);
//        	LCDBacklight(LCDOFF);
            GPSPower(GPSOFF);
        }

    }
}
