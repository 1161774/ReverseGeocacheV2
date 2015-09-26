/*
 * MCCOG21605B6W-SPTLYI.c
 *
 *  Created on: 26 Sep 2015
 *      Author: Bowmer
 */

#include <stdbool.h>
#include <stdint.h>
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"

#ifdef DEBUG
#include "utils/uartstdio.h"
#endif

void LCDTransmitChar(uint32_t I2CBase, char TransmitChar)
{
	I2CMasterDataPut(I2CBase, TransmitChar);
	I2CMasterControl(I2CBase, I2C_MASTER_CMD_SINGLE_SEND);

#ifdef DEBUG
	UARTprintf("Sent to LCD: %x", TransmitChar);
#endif

	while(I2CMasterBusy(I2CBase)){}
	SysCtlDelay(SysCtlClockGet()/240);
}

void LCDTransmitString(uint32_t I2CBase, uint32_t TransmitLength ,char *TransmitData)
{
	uint32_t index;

	//
	// Place first character into the I2C and initiate an I2C transaction.
	//
	I2CMasterDataPut(I2CBase, TransmitData[0]);
	I2CMasterControl(I2CBase, I2C_MASTER_CMD_BURST_SEND_START);
#ifdef DEBUG
	UARTprintf("Sent to LCD: %x", TransmitData[0]);
#endif
	while(I2CMasterBusy(I2CBase)){}

	//
	// Place second through second last characters into the I2C.
	//
	for(index = 1; index < TransmitLength - 1; index++)
	{
		I2CMasterDataPut(I2CBase, TransmitData[index]);
		I2CMasterControl(I2CBase, I2C_MASTER_CMD_BURST_SEND_CONT);
#ifdef DEBUG
	UARTprintf("Sent to LCD: %x", TransmitData[index]);
#endif
		while(I2CMasterBusy(I2CBase)){}
	}

	//
	// Place last character into the I2C and complete the I2C transaction.
	//
	I2CMasterDataPut(I2CBase, TransmitData[index]);
	I2CMasterControl(I2CBase, I2C_MASTER_CMD_BURST_SEND_FINISH);
#ifdef DEBUG
	UARTprintf("Sent to LCD: %x", TransmitData[index]);
#endif
	while(I2CMasterBusy(I2CBase)){}
	SysCtlDelay(SysCtlClockGet()/240);

}

void LCDCommand(uint32_t I2CBase, uint32_t Command)
{
	//
	// LCD commands are always initiated with a null byte.
	//
	I2CMasterDataPut(I2CBase, 0x00);
	I2CMasterControl(I2CBase, I2C_MASTER_CMD_BURST_SEND_START);
#ifdef DEBUG
	UARTprintf("Sent to LCD: %x", 0x00);
#endif
	while(I2CMasterBusy(I2CBase)){}

	//
	// Send the command.
	//
	I2CMasterDataPut(I2CBase, Command);
	I2CMasterControl(I2CBase, I2C_MASTER_CMD_BURST_SEND_FINISH);
#ifdef DEBUG
	UARTprintf("Sent to LCD: %x", Command);
#endif
	while(I2CMasterBusy(I2CBase)){}
	SysCtlDelay(SysCtlClockGet()/240);
}





void LCDConfigure(uint32_t I2CBase)
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

	char initString[] = {0x00, 0x38, 0x00, 0x39, 0x14, 0x79, 0x50, 0x6c, 0x0c, 0x01};	//as per datasheet

	LCDTransmitString(I2CBase, 10, initString);

}
