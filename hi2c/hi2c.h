/*~A:Module Header*/
/*****************************************************************************************************************
* Copyright by BSH Bosch und Siemens Hausgeraete GmbH - EDS                                                       
*                                                                                                                 
*-----------------------------------------------------------------------------------------------------------------
*                                                                                                                 
* Use, reproduction and dissemination of this software or parts of it, is not permitted without express           
* written authority of BSH EDS. The user is allowed to use this software exclusively for the development          
* of electronic boards for BSH. Usage for other purposes is strictly prohibited (e.g. development of              
* electronic boards for third parties). All rights, including copyright, rights created by patent grant           
* or registration, and rights by protection of utility patents, are reserved. Violations will be                  
* prosecuted by civil and criminal law.                                                                           
*                                                                                                                 
* The software was created and approved by acknowledged rules of technology. Because of the complexity            
* of embedded controller software the user of this software has to perform his own tests to ensure                
* proper functionality in his environment. The software was developed for usage as a software library.            
* The user is sole responsible for every other usage.  The user may modify the software for adaptations           
* needed for other microcontrollers, as well as to another compilers at own risk.                                 
* BSH will not support any adaptations or changes. BSH is authorised to use all adaptations for own purposes      
* free of charge.                                                                                                 
*                                                                                                                 
* BSH assumes no liability for the functionality or reliability of the software even in concrete                  
* applications. BSH assumes no liability for consequential damages, except in case of intention or gross          
* negligence. In any case the liability is limited to the typical and predictable damage.                         
*                                                                                                                 
* Technical changes are reserved.                                                                                 
*                                                                                                                 
******************************************************************************************************************
   PROJECT          SW_LIB_HAL
   MODULE-PREFIX    HI2C
   FILE             %PM%
   ARCHIVE          %PP%:%PI%
   PROCESSOR        independent
   COMPILER         independent
 *****************************************************************************************************************
   AUTHOR           team-S
   CREATED          8-3-2004
   LAST CHANGE      %PRT%
   LAST REVISION    %PR%
   STATUS           %PS%
 *****************************************************************************************************************
   DESCRIPTION                                                                                                    
     This file contains the exported definitions and interfaces for the implementation of the HAL I2C.
 *****************************************************************************************************************
   CHANGES
   %PL%
 *****************************************************************************************************************/

/*~E*/
/*~A:Documentation*/
/** \file
Thsi file contains the exported definitions and interfaces for the implementation of the HAL CCT.

\section CCTifc1 Interface Package 1
<TABLE>
<TR><TD><b>Interface</TD>  <TD>Description</b></TD></TR>
<TR><TD>HI2C\<unit\>_vInit(uint8_t index)</TD><TD>Initializes the operation of the HI2C.</TD></TR>
<TR><TD>HI2C\<unit\>_bSetAddr(uint16_t address)</TD><TD>Sets the address of the slave to communicate to.</TD></TR>
<TR><TD>HI2C\<unit\>_bSetTxData(uint8_t dataByte, bool stop)</TD><TD>Sends a single data byte over the I2C bus.</TD></TR>
<TR><TD>HI2C\<unit\>_ucGetRxData(void)</TD><TD>Returns the most recent received data byte from the addressed slave.</TD></TR>
<TR><TD>HI2C\<unit\>_vTriggerReceive(bool stop)</TD><TD>Trigger a reception.</TD></TR>
<TR><TD>HI2C\<unit\>_vHandleEvent(void)</TD><TD>A handler called, if the transmission/reception process has been finished.</TD></TR>
<TR><TD>HI2C\<unit\>_vEnableEvent(void)</TD><TD>Enables the call of the event handler.</TD></TR>
<TR><TD>HI2C\<unit\>_vDisableEvent(void)</TD><TD>Disables the call of the event handler.</TD></TR>
<TR><TD>HI2C<unit>_ucGetError(void)</TD><TD>Returns the collected error information.</TD></TR>
</TABLE>
\n
\n
\n
The following picture visualises the Interface of HI2C:\n
\n
\image html hi2c.jpg "Picture 1: Overview HAL I2C"
*/
/*~E*/
#ifndef __HI2C_H

#define __HI2C_H
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
/*~A:Includes*/

/*~E*/
/*~A:Definitions and Declarations*/
/*~A:DEFINITIONS*/
/* DEFINITIONS ****************************************************************************************/

/*~T*/
/* Definitions for HI2C<unit>_ucGetError */
/**
\b description: \n
 This is the define that represents the error indication of a message not acknowledged by the slave.
 The error is  returned by HI2C<unit>_ucGetError().
*/
#define HI2C_NACK    (1u<<0u)

/*~T*/
/**
\b description: \n
 This is the define that represents the error indication of a message timeout by the slave.
 The error is  returned by HI2C<unit>_ucGetError().
*/
#define HI2C_TIMEOUT (1u<<1u)

/*~T*/

/*~E*/
/*~A:TYPE DEFINITIONS*/
/* TYPE DEFINITIONS ***********************************************************************************/

/*~E*/
/*~A:DECLARATIONS*/
/* DECLARATIONS **************************************************************************************/
/*~T*/
/**

\b \<unit\>: 0 \n

\b description: \n
 This is the external reference to the maximum waitstate delay. This delay controls
 the number of cycles the master shall wait for a slave acknowledge.

<B>HC08 implementation:</B>
 cycles on the HC08 implementation is a loop with takes at least 20 processor cycles
*/
extern const uint8_t HI2C0_ucMaxWaitState;

/*~T*/
/* private data */
extern uint8_t HI2C0_ucError;
extern uint8_t HI2C0_ucLastRx;
extern bool  HI2C0_bEventEnabled;

/*~E*/
#ifdef __PCGCC__
#include "hi2c_Stub.h"
#else

/*~E*/
/*~A:GLOBAL FUNCTIONS*/
/* GLOBAL FUNCTIONS **********************************************************************************/

/*~A:HI2C<unit>_vInit*/
/** Initializes the operation of the HI2C.

\b package: \n
 HI2CPackage1\n
 HI2CPackage2\n

\b \<unit\>: 0 \n

\b description: \n
 This function initializes the operation of the HI2C<unit>. It uses the respective
 configuration parameters defined in HI2C<unit>_Config. The call of the handler
 function HI2C<unit>_vHandleEvent is disabled after initialization. After initialization the
 HI2C<unit> is ready to accept further requests

\note This function is not reentrant.
      A configuration HI2C<unit>_Config is not required for initialization.

\param ucIndex: Indexes the configuration set to be used for initialization.\n
   \b type: uint8_t\n
   \b range: 0x00..0xFF

\return none
*/

/*~T*/
extern void HI2C0_vInit(uint8_t ucIndex);

/*~E*/
/*~A:HI2C<unit>_bSetAddr*/
/** Sets the address of the slave to communicate to.

\b package: \n
 HI2CPackage1\n
 HI2CPackage2\n

\b \<unit\>: 0\n

\b description: \n
 This interface sets the address of the slave to communicate to. The read/write information is
 contained in the address information. After the call of this interface the transmission of the
 address starts. Depending on the read/write bit of the address further calls of HI2C<unit>_bSetTxData
 or HI2C<unit>_ucGetRxData are possible until the stop information is passed to the interface.\n
 \b HI2CPackage1: \m
  This function completes the transmission and returns if
  the address is sent.\n
 \b HI2CPackage2: \n
  Once the transmission is completed, the respective handler
  HI2C<unit>_vHandleEvent is called.

\param uiAddress: This parameter is the address (including direction) to communicate to.\n
   \b type: uint16_t\n
   \b range: 0x00 .. 0xFF for 8 bit addressses

\return: The function returns true, if the setting of the address was successful,
         false otherwise.

 \b HI2CPackage1: \n
  The interface returns false, if the partner that shall be addressed is
  not responding (no acknowledge) or the partner inserts too many
  waitstates (by holding SCL low).
  If the event handler is enabled, the event handler is immediately
  called by this function once the reception is completed.

 \b HI2CPackage2: \n
  If the bus is currently busy due to another transmission/reception, this
  interface returns false.\n

 \b type: bool\n
 \b range: true, false
*/

/*~T*/
extern bool HI2C0_bSetAddr(uint8_t ucAddress);

/*~E*/
/*~A:HI2C<unit>_bSetTxData*/
/** Sends a single data byte over the I2C bus.

\b package: \n
 HI2CPackage1\n
 HI2CPackage2\n

\b \<unit\>: 0\n

\b description: \n
 This interface sends a single data byte over the I2C bus. After the call of this
 interface the transmission process starts.

  \b HI2CPackage1: \n
   This function completes the transmission and returns, if the date has been
   sent. If the event handler is enabled the event handler is called immediately
   once the reception is completed.\n

  \b HI2CPackage2: \n
   Once the transmission has been finished the respective handler HI2C<unit>_vHandleEvent
   is be called.

\param ucDataByte: This parameter contains the data byte to be transmitted.\n
   \b type: uint8_t\n
   \b range: 0x00..0xFF

\param bStop: This parameter defines if the stop condition has to be
              transmitted after the data byte.\n
   \b type: bool\n
   \b range: true, transmit stop condition

\return The function returns true, if the setting of the data was successful, false otherwise.\n

 \b HI2CPackage1: \n
  The interface returns false if the partner is not responding (no
  acknowledge) or the partner inserts too many waitstates (by holding SCL low).\n

 \b HI2CPackage2: \n
  This interface shall return false if any previous transmissions
  (addressing) has been failed or a previous transmissions (addressing) is
  still in progress.\n

 \b type: bool\n
 \b range: true, false
*/

/*~T*/
extern bool HI2C0_bSetTxData(uint8_t ucDataByte, bool bStop);

/*~E*/
/*~A:HI2C<unit>_ucGetRxData*/
/** Returns the most recent received data byte.

\b package: \n
 HI2CPackage1\n
 HI2CPackage2\n

\b \<unit\>: 0\n

\b description: \n
 This interface returns the most recent received data byte from the addressed
 slave.

\param none

\return This interface returns the received data byte.\n
  \b type: uint8_t\n
  \b range: 0x00..0xFF
*/

/*~T*/
#define HI2C0_ucGetRxData()   (HI2C0_ucLastRx)

/*~E*/
/*~A:HI2C<unit>_vTriggerReceive*/
/** Triggers a reception.

\b package: \n
 HI2CPackage1\n
 HI2CPackage2\n

\b \<unit\>: 0\n

\b description: \n
 \b HI2CPackage1: \m
  The interface triggers a reception and saves the received data byte once
  the reception has been finished. The stop information passed via the interface
  is used to generate the stop condition if requested.
  If the event handler is enabled the event handler will be immediately called by
  this function once the reception is completed.

 \b HI2CPackage2: \n
  This interface triggers the reception of the next data byte. The stop
  information passed via the interface shall is used to generate the stop
  condition if requested.

\param bStop: This information is used to signal the slave whether the received byte
              shall be the last one.\n
   \b type: bool\n
   \b range:
           - true, generate STOP condition
           - false, generate no STOP condition

\return none
*/

/*~T*/
extern uint8_t HI2C0_vTriggerReceive(bool bStop);

/*~E*/
/*~A:HI2C<unit>_vHandleEvent*/
/** A handler called, if the transmission/reception process to/from the slave has been finished.

\b package: \n
 HI2CPackage1\n
 HI2CPackage2\n

\b \<unit\>: 0\n

\b description: \n
 This handler is called, if the transmission/reception process to/from the slave
 has been finished or upon occurrence of an error condition.

 \b HI2CPackage1: \n
  The implementation calls this function on the same interrupt level as the
  caller of the HI2C<unit>_vTriggerReceive or HI2C<unit>_bSetTxData
  function.\n

\note This function must be implemented by the user.

\param none

\return none
*/

/*~T*/
extern void HI2C0_vHandleEvent(void);

/*~E*/
/*~A:HI2C<unit>_vEnableEvent*/
/** Enables the call of the event handler.

\b package: \n
 HI2CPackage1\n
 HI2CPackage2\n

\b \<unit\>: 0\n

\b description: \n
 This function enables the call of the event handler HI2C<unit>_vHandleEvent.

\note This function must be implemented by the user.

\param none

\return none
*/

/*~T*/
#define HI2C0_vEnableEvent() { HI2C0_bEventEnabled = 1u; }

/*~E*/
/*~A:HI2C<unit>_vDisableEvent*/
/** Disables the call of the event handler.

\b package: \n
 HI2CPackage1\n
 HI2CPackage2\n

\b \<unit\>: 0 \n

\b description: \n
 This function disables the call of the event handler HI2C<unit>_vHandleEvent.

\param none

\return none
*/

/*~T*/
#define HI2C0_vDisableEvent() { HI2C0_bEventEnabled = 0u; }

/*~E*/
/*~A:HI2C<unit>_ucGetError*/
/** Returns the collected error information.

\b package: \n
 HI2CPackage1\n
 HI2CPackage2\n

\b \<unit\>: 0 \n

\b description: \n
 This interface returns the collected error information. The error information
 is cleared on read. In case an error is detected that has terminated the
 communication, the handler HI2C<unit>_vHandleEvent is be called.

\param none

\return
 \b type:  uint8_t\n
 \b range:
           - HI2C_NACK    This information is set whenever the acknowledge of the addressed partner was
                          not detected.

           - HI2C_TIMEOUT This information is set whenever the partner has inserted to many waitstates
                          (stuck SCL).
*/

/*~T*/
#define HI2C0_ucGetError()  (HI2C0_ucError)

/*~E*/
/*~A:HI2C<unit>_vSendStop*/
/** Generates a stop condition on the I2C bus.

\b package: \n
 HI2CPackage1\n
 HI2CPackage2\n

\b \<unit\>: 0\n

\b description: \n
 This interface generates a stop condition on the I2C bus.

\param none

\return none
 
*/
/*~T*/
extern void HI2C0_vSendStop(void);
/*~E*/
/*~A:HI2C<unit>_bForceBusRelease*/
/** Generates clock pulses and a stop condition to force a slave to release the bus.

\b package: \n
 HI2CPackage1\n
 HI2CPackage2\n

\b \<unit\>: 0 \n

\b description: \n
 This interface generates clock pulses and a stop condition to force a slave to release the
 bus.

\param none

\return
       - 0: bus release has been failed
       - 1: bus release was successfull
 
*/
/*~T*/
extern bool HI2C0_bForceBusRelease(void);
/*~E*/
/*~A:HI2C<unit>_vBitDelayH*/
/** Performs the delay for H level of signal.

\b package: \n
 HI2CPackage1\n
 HI2CPackage2\n

\b \<unit\>: 0\n

\b description: \n
 User defined function that perform the delay for high phase of SCL
 (5 탎ec for 100 kHz mode or 1.3 탎ec for 400 kHz mode)
 To achieve the best performance the call/return overhead can be taken
 into acount.

\param none

\return none
*/
/*~T*/
extern void HI2C0_vBitDelayH(void);

/*~E*/
/*~A:HI2C<unit>_vBitDelayL*/
/** Performs the delay for L level of signal.

\b package: \n
 HI2CPackage1\n
 HI2CPackage2\n

\b \<unit\>: 0\n

\b description: \n
 User defined function that perform the delay for low phase of SCL
 (5 탎ec for 100 kHz mode or 1.3 탎ec for 400 kHz mode)
 To achieve the best performance the call/return overhead can be taken
 into acount.

\param none

\return none
*/
/*~T*/
extern void HI2C0_vBitDelayL(void);

/*~E*/
/*~E*/
/*~E*/
#endif // __PCGCC__

#endif // __HI2C_H
