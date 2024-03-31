/*
 * File: I2C.c
 * Author: Armstrong Subero
 * PIC: 16F876 w/Int OSC @ 20MHz, 5v
 * Program: Library file to configure PIC16F876 I2C module
 * Compiler: XC8
 * Program Version: 1.1
 *                *Added additional comments
 *                
 * Program Description: This Library allows you to control PIC16F876 I2C
 *                      
 * Created on November 12th, 2016, 7:05 AM
 * 
 * https://github.com/Apress/programming-pic-microcontrollers-w-xc8/blob/master/Chapter%209/EEPROM/24_I2C1.X/I2C.c
 * 
 * Modified by Rtek1000 (Mar, 31, 2024)
 * PIC: 16F876 @ 20MHz
 * - Added Send_I2C_ControlByte2 function to generic usage
 * 
 */

/*******************************************************************************
 *Includes and defines
 ******************************************************************************/

#ifndef __I2C_H__
#define __I2C_H__

#include <xc.h>

void I2C_Init(void);
void Send_I2C_Data(unsigned int databyte);
unsigned int Read_I2C_Data(void);
void Send_I2C_ControlByte2(unsigned int slaveAddress,unsigned int RW_bit);
void Send_I2C_ControlByte(unsigned int BlockAddress,unsigned int RW_bit);
void Send_I2C_StartBit(void);
void Send_I2C_StopBit(void);
void Send_I2C_ACK(void);
void Send_I2C_NAK(void);

#endif //#ifndef __I2C_H__