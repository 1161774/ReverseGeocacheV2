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

#define	SLAVE_ADDRESS		0x3E

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


void ConfigureI2C2(void)
{

    //
    // The I2C0 peripheral must be enabled before use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

    //
    // For this example I2C0 is used with PortB[3:2].  The actual port and
    // pins used may be different on your part, consult the data sheet for
    // more information.  GPIO port B needs to be enabled so these pins can
    // be used.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    //
    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PE4_I2C2SCL);
    GPIOPinConfigure(GPIO_PE5_I2C2SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);

    //
    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
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

void I2CTransmitChar(uint32_t I2CBase, char TransmitChar)
{
    I2CMasterDataPut(I2CBase, TransmitChar);
    I2CMasterControl(I2CBase, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(I2CBase)){}

}

void ConfigureLCD(void)
{
// 0x00 - Write command
// 0x38 - Function set
// 0x00 - Write command
// 0x39 - Function set
// 0x14 - Internal OSC frequency
// 0x79 - Contrast set
// 0x50 - Power/Icon control/Contrast set
// 0x6C - Follower control
// 0x0C - Display on/off
// 0x01 - Clear display


	//	char initString[] = {0x00, 0x38, 0x00, 0x39, 0x14, 0x79, 0x50, 0x6c, 0x0c, 0x01};	//as per datasheet

//	uint32_t index, stringLength = 10;


	I2CMasterDataPut(I2C2_BASE, 0x00);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C2_BASE)){}

	I2CMasterDataPut(I2C2_BASE, 0x38);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C2_BASE)){}

	I2CMasterDataPut(I2C2_BASE, 0x00);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C2_BASE)){}

	I2CMasterDataPut(I2C2_BASE, 0x39);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C2_BASE)){}

	I2CMasterDataPut(I2C2_BASE, 0x14);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C2_BASE)){}

	I2CMasterDataPut(I2C2_BASE, 0x79);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C2_BASE)){}

	I2CMasterDataPut(I2C2_BASE, 0x50);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C2_BASE)){}

	I2CMasterDataPut(I2C2_BASE, 0x6C);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C2_BASE)){}

	I2CMasterDataPut(I2C2_BASE, 0x0C);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C2_BASE)){}

	I2CMasterDataPut(I2C2_BASE, 0x01);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C2_BASE)){}





//    I2CMasterControl(I2CBase, I2C_MASTER_CMD_BURST_SEND_STOP);


/*	I2CMasterDataPut(I2C2_BASE, initString[0]);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C2_BASE)){}

	for(index = 1; index < stringLength; index++)
	{
		I2CMasterDataPut(I2C2_BASE, initString[index]);
		I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
	    while(I2CMasterBusy(I2C2_BASE)){}
	}
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_STOP);
    //
    // Wait until master module is done transferring.
    //
    while(I2CMasterBusy(I2C2_BASE)){}
*/
}

void I2CTransmitString(uint32_t I2CBase uint32_t TransmitLength ,char *TransmitData)
{

	//#define I2C_MASTER_CMD_BURST_SEND_START		0x00000003
	//#define I2C_MASTER_CMD_BURST_SEND_CONT		0x00000001
	//#define I2C_MASTER_CMD_BURST_SEND_FINISH		0x00000005
	//#define I2C_MASTER_CMD_BURST_SEND_STOP		0x00000004

	uint32_t index



	I2CMasterDataPut(I2CBase, TransmitData[0]);
	I2CMasterControl(I2CBase, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2CBase)){}

	for(index = 1; index < stringLength; index++)
	{
		I2CMasterDataPut(I2CBase, TransmitData[index]);
		I2CMasterControl(I2CBase, I2C_MASTER_CMD_BURST_SEND_CONT);
	    while(I2CMasterBusy(I2CBase)){}
	}
	I2CMasterControl(I2CBase, I2C_MASTER_CMD_BURST_SEND_STOP);
    //
    // Wait until master module is done transferring.
    //
    while(I2CMasterBusy(I2CBase)){}

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
	uint32_t i;
	bool goodCopy;

	for(i=0;i<MESSAGE_LENGTH;i++)
	{
		message[i] = 0;
	}

	goodCopy = false;
	i=0;
	while(true){
		message[i] = GPSRead.RxBuffer[i+7];
		i++;
		if (GPSRead.RxBuffer[i+7] == '*')
		{
			goodCopy = true;
			break;
		}
		else if (GPSRead.RxBuffer[i+7] == '*' || i >= MESSAGE_LENGTH)
		{
			goodCopy = false;
			break;
		}
	}

	if(goodCopy)
	{
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
}


void BoxLock(uint32_t Cmd)
{
	if(Cmd == CMDLOCK && g_LockStatus == STUNLOCKED)
	{
		g_LockStatus = STLOCKED;
		GPIOPinWrite(LOCK_LID_BASE,LOCK_LID,0);
		SysCtlDelay(SysCtlClockGet()/4);
		GPIOPinWrite(LOCK_LID_BASE,LOCK_LID,LOCK_LID);
	}
	else if(Cmd == CMDUNLOCK && g_LockStatus == STLOCKED)
	{
		g_LockStatus = STUNLOCKED;
		GPIOPinWrite(UNLOCK_LID_BASE,UNLOCK_LID,0);
		SysCtlDelay(SysCtlClockGet()/4);
		GPIOPinWrite(UNLOCK_LID_BASE,UNLOCK_LID,UNLOCK_LID);
	}
}

void LCDBacklight(uint32_t Cmd)
{
	if(Cmd == LCDON)
	{
		GPIOPinWrite(LCD_POWER_BASE, LCD_POWER, 0);
	}
	else if(Cmd == LCDOFF)
	{
		GPIOPinWrite(LCD_POWER_BASE, LCD_POWER, LCD_POWER);
	}
}

void GPSPower(uint32_t Cmd)
{
	if(Cmd == GPSON)
	{
		GPIOPinWrite(GPS_POWER_BASE, GPS_POWER, 0);
	}
	else if(Cmd == GPSOFF)
	{
		GPIOPinWrite(GPS_POWER_BASE, GPS_POWER, GPS_POWER);
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
    ConfigureUART1();

	//
	// Open I2C and LCD.
	//
	LCDBacklight(LCDOFF);
    ConfigureI2C2();
    ConfigureLCD();
	LCDBacklight(LCDON);



	I2CMasterDataPut(I2C2_BASE, 0x00);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C2_BASE)){}

	I2CMasterDataPut(I2C2_BASE, 0x80);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C2_BASE)){}


    I2CMasterDataPut(I2C2_BASE, 0x40);
  	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C2_BASE)){}

	I2CMasterDataPut(I2C2_BASE, 0x42);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C2_BASE)){}

	I2CMasterDataPut(I2C2_BASE, 0x45);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C2_BASE)){}

	I2CMasterDataPut(I2C2_BASE, 0x4E);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C2_BASE)){}

	I2CMasterDataPut(I2C2_BASE, 0x21);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C2_BASE)){}

//	char row1[] = {0x00, 0x80};
//	char row2[] = {0x00, 0xc0};
//	char clearLine[] = "@                ";

//	SysCtlDelay(SysCtlClockGet() / 120);

//	I2CTransmitString(I2C2_BASE, row1);
//	I2CTransmitChar(I2C2_BASE, 0x00);
//	I2CTransmitChar(I2C2_BASE, 0x80);
//	I2CTransmitChar(I2C2_BASE, 0x42);
//	I2CTransmitChar(I2C2_BASE, 0x45);
//	I2CTransmitChar(I2C2_BASE, 0x4E);
//	I2CTransmitChar(I2C2_BASE, 0x21);

//	I2CTransmitString(I2C2_BASE, clearLine);
//	I2CTransmitString(I2C2_BASE, row2);
//	I2CTransmitChar(I2C2_BASE, 0x00);
//	I2CTransmitChar(I2C2_BASE, 0xC0);
//	I2CTransmitString(I2C2_BASE, clearLine);

//	char string1[] = "@Aquiring Lock...";
//	char string2[] = "@Please Wait";
//	I2CTransmitString(I2C2_BASE, string1);
//	I2CTransmitString(I2C2_BASE, clearLine);
//	I2CTransmitString(I2C2_BASE, string2);
//	I2CTransmitString(I2C2_BASE, clearLine);


	/*
void LCD_Row1(){
		char sentence[] = {0x00, 0x80};

		I2CTransmitMultiple(address, sentence, 2);
	}

void LCD_Row2(){
		char sentence[] = {0x00, 0xc0};

		I2CTransmitMultiple(address, sentence, 2);
	}

void LCD_ClearLine(){
	char sentence[] = "@                ";

	I2CTransmitMultiple(address, sentence, 17);

}

	LCD_Row1();
	LCD_ClearLine();
	LCD_Row1();
	I2CTransmitMultiple(address, string1, strlen(string1));

	LCD_Row2();
	LCD_ClearLine();
	LCD_Row2();
	I2CTransmitMultiple(address, string2, strlen(string2));
*/

    UARTprintf("\033[2JReverse Geocache\n");
    UARTprintf("---------------------------------\n\n");

    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();

    GPSPower(GPSON);

    //
    // Loop forever.
    //
    while(1)
    {

    	if(GPSRead.MessageReady)
    	{
    		parseString();
    		GPSRead.MessageReady = false;
    	}


        if(GPIOPinRead(PUSHBUTTON_BASE, PUSHBUTTON))
        {
        	BoxLock(CMDUNLOCK);
        	LCDBacklight(LCDON);

        }
        else
        {
        	BoxLock(CMDLOCK);
        	LCDBacklight(LCDOFF);
            GPSPower(GPSOFF);
        }

    }
}
