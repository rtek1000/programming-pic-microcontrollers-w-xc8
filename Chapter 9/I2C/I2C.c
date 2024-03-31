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
 * Modified: Rtek1000
 * PIC16F876, 20MHz, 400kHz
 * - Added Send_I2C_ControlByte2 function for generic use
 * 
 */

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 20000000
#endif

#include "I2C.h"


void I2C_Init(void){
    
    //**********************************************************************************
    // Setup MSSP as I2C Master mode, clock rate of 400Khz
    //**********************************************************************************

    SSPCONbits.SSPM=0x08;       // I2C Master mode, clock = Fosc/(4 * (SSPADD+1))
    SSPCONbits.SSPEN=1;         // enable MSSP port

    // **************************************************************************************
    // The SSPADD register value  is used to determine the clock rate for I2C communication.
    //
    // Equation for I2C clock rate:   Fclock = Fosc/[(SSPADD +1)*4]
    //
    // For this example we want the the standard 100Khz I2C clock rate and our
    // internal Fosc is 20Mhz so we get:  400000 = 20000000/[(SSPADD+1)*4]
    // or solving for SSPADD = [(20000000/400000)-4]/4
    // and we get SSPADD = 12

    SSPADD = 12;                // set Baud rate clock divider
    // **************************************************************************************
  

    __delay_ms(10); // let everything settle.
}

//**************************************************************************************
// Send one byte to SEE
//**************************************************************************************
void Send_I2C_Data(unsigned int databyte)
{
    PIR1bits.SSPIF=0;          // clear SSP interrupt bit
    SSPBUF = databyte;          // send databyte
    while(!PIR1bits.SSPIF);    // Wait for interrupt flag to go high indicating transmission is complete
}


//**************************************************************************************
// Read one byte from SEE
//**************************************************************************************
unsigned int Read_I2C_Data(void)
{
    PIR1bits.SSPIF=0;          // clear SSP interrupt bit
    SSPCON2bits.RCEN=1;         // set the receive enable bit to initiate a read of 8 bits from the serial eeprom
    while(!PIR1bits.SSPIF);    // Wait for interrupt flag to go high indicating transmission is complete    
    return (SSPBUF);            // Data from eeprom is now in the SSPBUF so return that value
}

//**************************************************************************************
// Send control byte to SEE (slave address and the R/W bit)
//**************************************************************************************
//**************************************************************************************
void Send_I2C_ControlByte2(unsigned int slaveAddress,unsigned int RW_bit)
{
    PIR1bits.SSPIF=0;          // clear SSP interrupt bit

    // and 'R' is the Read/Write bit

    SSPBUF = ((slaveAddress << 1) + RW_bit);  // send the control byte
    
    while(!PIR1bits.SSPIF);    // Wait for interrupt flag to go high indicating transmission is complete
}

//**************************************************************************************
// Send control byte to SEE (this includes 4 bits of device code, block select bits and the R/W bit)
//**************************************************************************************
// Notes:
// 1) The device code for serial eeproms is defined as '1010' which we are using in this example
// 2) RW_bit can only be a one or zero
// 3) Block address is only used for SEE devices larger than 4K, however on
// some other devices these bits may become the hardware address bits that allow you
// to put multiple devices of the same type on the same bus.  Read the datasheet
// on your particular serial eeprom device to be sure.
//**************************************************************************************
void Send_I2C_ControlByte(unsigned int BlockAddress,unsigned int RW_bit)
{
    PIR1bits.SSPIF=0;          // clear SSP interrupt bit

    // Assemble the control byte from device code, block address bits and R/W bit
    // so it looks like this: CCCCBBBR
    // where 'CCCC' is the device control code
    // 'BBB' is the block address
    // and 'R' is the Read/Write bit

    SSPBUF = (((0b1010 << 4) | (BlockAddress <<1)) + RW_bit);  // send the control byte
    
    while(!PIR1bits.SSPIF);    // Wait for interrupt flag to go high indicating transmission is complete
}

//**************************************************************************************
// Send start bit to SEE
//**************************************************************************************
void Send_I2C_StartBit(void)
{
    PIR1bits.SSPIF=0;          // clear SSP interrupt bit
    SSPCON2bits.SEN=1;          // send start bit
    while(!PIR1bits.SSPIF);    // Wait for the SSPIF bit to go back high before we load the data buffer
}

//**************************************************************************************
// Send stop bit to SEE
//**************************************************************************************
void Send_I2C_StopBit(void)
{
    PIR1bits.SSPIF=0;          // clear SSP interrupt bit
    SSPCON2bits.PEN=1;          // send stop bit
    while(!PIR1bits.SSPIF);    // Wait for interrupt flag to go high indicating transmission is complete
}


//**************************************************************************************
// Send ACK bit to SEE
//**************************************************************************************
void Send_I2C_ACK(void)
{
   PIR1bits.SSPIF=0;          // clear SSP interrupt bit
   SSPCON2bits.ACKDT=0;        // clear the Acknowledge Data Bit - this means we are sending an Acknowledge or 'ACK'
   SSPCON2bits.ACKEN=1;        // set the ACK enable bit to initiate transmission of the ACK bit to the serial eeprom
   while(!PIR1bits.SSPIF);    // Wait for interrupt flag to go high indicating transmission is complete
}

//**************************************************************************************
// Send NAK bit to SEE
//**************************************************************************************
void Send_I2C_NAK(void)
{
    PIR1bits.SSPIF=0;           // clear SSP interrupt bit
    SSPCON2bits.ACKDT=1;        // set the Acknowledge Data Bit- this means we are sending a No-Ack or 'NAK'
    SSPCON2bits.ACKEN=1;        // set the ACK enable bit to initiate transmission of the ACK bit to the serial eeprom
    while(!PIR1bits.SSPIF);    // Wait for interrupt flag to go high indicating transmission is complete
}
