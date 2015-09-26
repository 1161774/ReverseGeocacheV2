/*
 * MCCOG21605B6W-SPTLYI.h
 *
 *  Created on: 26 Sep 2015
 *      Author: Bowmer
 */

#ifndef MCCOG21605B6W_SPTLYI_H_
#define MCCOG21605B6W_SPTLYI_H_


#define	SLAVE_ADDRESS		0x3E

#define LCD_CLEAR_DISPLAY		0x01
#define LCD_RETURN_HOME			0x02

#define	LCD_ROW_1				0x80
#define	LCD_ROW_2				0xC0
#define LCD_ON					0xC0
#define LCD_OFF					0x80
#define LCD_CURSOR_BLINK_OFF	0x0C
#define LCD_CURSOR_BLINK_ON		0x0D


extern void I2C2Configure(void);
extern void LCDCommand(uint32_t I2CBase, uint32_t Command);
extern void LCDTransmitString(uint32_t I2CBase, uint32_t TransmitLength ,char *TransmitData);
extern void LCDConfigure(uint32_t I2CBase);
extern void LCDTransmitChar(uint32_t I2CBase, char TransmitChar);


#endif /* MCCOG21605B6W_SPTLYI_H_ */
