#ifndef _MFX_L4_I2C_H
#define _MFX_L4_I2C_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/**
\b description: \n
 This is the define that represents the error indication of a message not acknowledged by the slave.
 The error is  returned by HI2C<unit>_ucGetError().
*/
#define HI2C_NACK    (1u<<0u)

/**
\b description: \n
 This is the define that represents the error indication of a message timeout by the slave.
 The error is  returned by HI2C<unit>_ucGetError().
*/
#define HI2C_TIMEOUT (1u<<1u)

/**
\b description: \n
 This is the external reference to the maximum waitstate delay. This delay controls
 the number of cycles the master shall wait for a slave acknowledge.
<B>HC08 implementation:</B>
 cycles on the HC08 implementation is a loop with takes at least 20 processor cycles
*/
extern const uint8_t HI2Cmfx_ucMaxWaitState;

extern uint8_t HI2Cmfx_ucError;
extern uint8_t HI2Cmfx_ucLastRx;
extern bool  HI2Cmfx_bEventEnabled;

/************************************************************************************/

/** Initializes the port of the HI2C.
\b package: \n
 HI2CPackage1\n
 HI2CPackage2\n
\b \<unit\>: 0 \n
\b description: \n
 This function initializes the special features for the port of the HI2C<unit>. For instance in case
 of the Stm32 the clock for the port is enabled in this function.
\note
\param none
\return none
*/
#define HI2C_SPECIAL_INIT
extern void HI2Cmfx_vInitPort(void);


/***************************************************************************************************$FHB
$name: HI2C<unit>_vOutputSCL
package:
 HI2CPackage1
 HI2CPackage2
prototype:
 void HI2C<unit>_vOutputSCL(void);
 <unit>: 0
description:
 User defined function that shall set the SCL pin to output
parameters:
 none
return:
 type:
           void
$HE****************************************************************************************************/
extern void HI2Cmfx_vOutputSCL(void);


/***************************************************************************************************$FHB
$name: HI2C<unit>_vOutputSDA
package:
 HI2CPackage1
 HI2CPackage2
prototype:
 void HI2C<unit>_vOutputSDA(void);
 <unit>: 0
description:
 User defined function that shall set the SDA pin to output
parameters:
 none
return:
 type:
           void
$HE****************************************************************************************************/
extern void HI2Cmfx_vOutputSDA(void);


/***************************************************************************************************$FHB
$name: HI2C<unit>_vInputSCL
package:
 HI2CPackage1
 HI2CPackage2
prototype:
 void HI2C<unit>_vInputSCL(void);
 <unit>: 0
description:
 User defined function that shall set the SCL pin to input
parameters:
 none
return:
 type:
            void
$HE****************************************************************************************************/
extern void HI2Cmfx_vInputSCL(void);


/***************************************************************************************************$FHB
$name: HI2C<unit>_vInputSDA
package:
 HI2CPackage1
 HI2CPackage2
prototype:
 void HI2C<unit>_vInputSDA(void);
 <unit>: 0
description:
 User defined function that shall set the SDA pin to input
parameters:
 none
return:
 type:
           void
$HE****************************************************************************************************/
extern void HI2Cmfx_vInputSDA(void);


/***************************************************************************************************$FHB
$name: HI2C<unit>_vClrSCL
package:
 HI2CPackage1
 HI2CPackage2
prototype:
 void HI2C<unit>_vClrSCL(void);
 <unit>: 0
description:
 User defined function that shall set the SCL pin to a low level
parameters:
 none
return:
 type:
           void
$HE****************************************************************************************************/
extern void HI2Cmfx_vClrSCL(void);


/***************************************************************************************************$FHB
$name: HI2C<unit>_vSetSCL
package:
 HI2CPackage1
 HI2CPackage2
prototype:
 void HI2C<unit>_vSetSCL(void);
 <unit>: 0
description:
 User defined function that shall set the SCL pin to a high level
parameters:
 none
return:
 type:
           void
$HE****************************************************************************************************/
extern void HI2Cmfx_vSetSCL(void);


/***************************************************************************************************$FHB
$name: HI2C<unit>_vClrSDA
package:
 HI2CPackage1
 HI2CPackage2
prototype:
 void HI2C<unit>_vClrSDA(void);
 <unit>: 0
description:
 User defined function that shall set the SDA pin to a low level
parameters:
 none
return:
 type:
           void
$HE****************************************************************************************************/
extern void HI2Cmfx_vClrSDA(void);


/***************************************************************************************************$FHB
$name: HI2C<unit>_vSetSDA
package:
 HI2CPackage1
 HI2CPackage2
prototype:
 void HI2C<unit>_vSetSDA(void);
 <unit>: 0
description:
 User defined function that shall set the SDA pin to a high level
parameters:
 none
return:
 type:
           void
$HE****************************************************************************************************/
extern void HI2Cmfx_vSetSDA(void);


/***************************************************************************************************$FHB
$name: HI2C<unit>_bGetSCL
package:
 HI2CPackage1
 HI2CPackage2
prototype:
 void HI2C<unit>_bGetSCL(void);
 <unit>: 0
description:
 User defined function that shall return the current pin level of SCL
parameters:
 none
return:
 type:
           void
$HE****************************************************************************************************/
extern bool HI2Cmfx_bGetSCL(void);

/***************************************************************************************************$FHB
$name: HI2C<unit>_bGetSDA
package:
 HI2CPackage1
 HI2CPackage2
prototype:
 void HI2C<unit>_bGetSDA(void);
 <unit>: 0
description:
 User defined function that shall return the current pin level of SDA
parameters:
 none
return:
 type:
            void
$HE****************************************************************************************************/
extern bool HI2Cmfx_bGetSDA(void);


/************************************************************************************/


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

extern void HI2Cmfx_vInit(uint8_t ucIndex);

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
extern bool HI2Cmfx_bSetAddr(uint8_t ucAddress);

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
extern bool HI2Cmfx_bSetTxData(uint8_t ucDataByte, bool bStop);

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
#define HI2Cmfx_ucGetRxData()   (HI2Cmfx_ucLastRx)

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
extern uint8_t HI2Cmfx_vTriggerReceive(bool bStop);

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
extern void HI2Cmfx_vHandleEvent(void);

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
#define HI2Cmfx_vEnableEvent() { HI2Cmfx_bEventEnabled = 1u; }

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
#define HI2Cmfx_vDisableEvent() { HI2Cmfx_bEventEnabled = 0u; }

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
#define HI2Cmfx_ucGetError()  (HI2Cmfx_ucError)

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
extern void HI2Cmfx_vSendStop(void);

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
extern bool HI2Cmfx_bForceBusRelease(void);

/** Performs the delay for H level of signal.

\b package: \n
 HI2CPackage1\n
 HI2CPackage2\n

\b \<unit\>: 0\n

\b description: \n
 User defined function that perform the delay for high phase of SCL
 (5 µsec for 100 kHz mode or 1.3 µsec for 400 kHz mode)
 To achieve the best performance the call/return overhead can be taken
 into acount.

\param none

\return none
*/
extern void HI2Cmfx_vBitDelayH(void);

/** Performs the delay for L level of signal.

\b package: \n
 HI2CPackage1\n
 HI2CPackage2\n

\b \<unit\>: 0\n

\b description: \n
 User defined function that perform the delay for low phase of SCL
 (5 µsec for 100 kHz mode or 1.3 µsec for 400 kHz mode)
 To achieve the best performance the call/return overhead can be taken
 into acount.

\param none

\return none
*/
extern void HI2Cmfx_vBitDelayL(void);

#endif // _MFX_L4_I2C_H
