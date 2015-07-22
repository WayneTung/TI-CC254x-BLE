/**************************************************************************************************
  Filename:       simpleBLEperipheral.h
  Revised:        $Date: 2014-08-28 11:59:12 -0700 (Thu, 28 Aug 2014) $
  Revision:       $Revision: 1420 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  definitions and prototypes.

  Copyright 2010 - 2011 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

#ifndef SIMPLEBLEPERIPHERAL_H
#define SIMPLEBLEPERIPHERAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Simple BLE Peripheral Task Events
#define SBP_START_DEVICE_EVT                         0x0001
#define SBP_PERIODIC_EVT                                  0x0002

// << Wayne >> << Clock >> ++
#define CLOCK_UPDATE_EVT                                0x0010
// << Wayne >> << Clock >> --

// << Wayne >> << 128-bit UUID  >> ++
#define DB_UUID_C    0x0001
#define DB_UUID_A    0x0001    
#define DB_UUID_S    0xFFFF                   
// << Wayne >> << 128-bit UUID  >> --

// << Wayne >> << 128-bit UUID >> ++
#define dB_UUID(uuid_C,uuid_A,uuid_S)       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, LO_UINT16(uuid_S), HI_UINT16(uuid_S), LO_UINT16(uuid_A), HI_UINT16(uuid_A), LO_UINT16(uuid_C), HI_UINT16(uuid_C), 0xDB, 0xFF, 0x10, 0xF5 
// << Wayne >> << 128-bit UUID >> --
// << Wayne >> << Device ID >> ++
#define dB_DevID(id)       {0xFF, 0xDB, HI_UINT16(id), LO_UINT16(id), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
// << Wayne >> << Device D >> --
// << Wayne >> << dBCmd >> ++
#define DBCMD_COMFIRM_TICKET                          "s,et,01,dic,"
#define DBCMD_COMFIRM_TICKET_LEN                  12
#define DBCMD_READ_EXCHANGE_NUMBER          "s,et,01,rct,"
#define DBCMD_READ_EXCHANGE_NUMBER_LEN  12      
// << Wayne >> << dBCmd >> --

/*********************************************************************
 * MACROS
 */
// UART macros
#if defined (SERIAL_INTERFACE)
#define UART_SEND_STRING(str, len)                      sendSerialString( (str), (len))
  #if UART_DEBUG_MSG == TRUE
  #define UART_SEND_DEBUG_MSG(str, len)            sendSerialString( "[DMSG] ", 7 ); sendSerialString( (str), (len))
  #else
  #define UART_SEND_DEBUG_MSG(str, len)             
  #endif
#else
#define UART_SEND_STRING(str, len)                      
#define UART_SEND_DEBUG_MSG(str, len) 
#endif
/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void SimpleBLEPeripheral_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events );

uint8 Application_StartAdvertise(uint16 duration, uint16 interval);
uint8 Application_StopAdvertise();
uint8 Application_TerminateConnection();

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLEPERIPHERAL_H */
