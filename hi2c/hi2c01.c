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
   PROCESSOR        Motorola MC68HC908AB32
   COMPILER         Metrowerks 5.0
 *****************************************************************************************************************
   AUTHOR           team-S
   CREATED          09-03-2004
   LAST CHANGE      %PRT%
   LAST REVISION    %PR%
   STATUS           %PS%
 *****************************************************************************************************************
   DESCRIPTION
     This module contains the implementation of HAL I2C package 01
 *****************************************************************************************************************
   CHANGES
   %PL%
 *****************************************************************************************************************/

/* Includes */

/* ------ Test instrumentation ------ */
// UlBr 28.01.15: Hello Marek, libraries should be delivered  test instrumented and with Stub files, but for HAL its different.
// The HAL is only stubed when you build for PC and therefore the stub files are exisitng.
// You may also instrument your code when you are testing your HAL and then deliver a lib.
// But if you deliver the HAL as a C-file you have to add an additional define switch to prevent the activation of the test instrumentation in the applicating project
// when it builds with MODULE_TEST. Best regards Ulrike
//#if defined(ALL_MODULES_UNDER_TEST) || defined(HI2C01_UNDER_TEST)
//#define MODULE_UNDER_TEST // This Module is under test
//#endif
//#include "TestSupport.h" //Included even if not under test to use empty defines
/* ---------------------------------- */
#include <stdio.h>
#include "hi2c.h"
#include "hi2c01xs.h"

/* TYPE DEFINITIONS ***********************************************************************************/

/* PROTOTYPES *****************************************************************************************/

/* VARIABLES ******************************************************************************************/

uint8_t HI2C0_ucError;
uint8_t HI2C0_ucLastRx;
bool  HI2C0_bEventEnabled;

/* FUNCTIONS */
static void HI2C0_vWaitForSlave(void);
static void HI2C0_vMakeStopCondition(void);

/* CONSTANTS ******************************************************************************************/

/* DEFINITIONS ****************************************************************************************/

/* HI2C0_vWaitForSlave() */
/* set SCL to a high state and wait until the slave releases it too */
static void HI2C0_vWaitForSlave(void)

{
   uint8_t ucWait = HI2C0_ucMaxWaitState;

   HI2C0_vInputSCL();
   HI2C0_vSetSCL();

   while (HI2C0_bGetSCL() == 0u)

   {
      ucWait--;

      if (ucWait == 0u)

      {
         /* SCL stuck? ... */
         HI2C0_ucError |= HI2C_TIMEOUT;
         return;

      }
   }
   HI2C0_vOutputSCL();
   HI2C0_vBitDelayH();

}

/* HI2C0_vMakeStopCondition() */
static void HI2C0_vMakeStopCondition(void)

{
   /* generate stop condition */
   HI2C0_vOutputSDA();
   HI2C0_vClrSDA();
   HI2C0_vBitDelayL();
   HI2C0_vWaitForSlave();
   HI2C0_vSetSDA();
   HI2C0_vBitDelayH();

   /* release bus */
   HI2C0_vInputSCL();
   HI2C0_vInputSDA();

}

/* HI2C0_vInit() */
/*lint -save -efunc(715,HI2C0_vInit)    disable lint args not used informational*/
void HI2C0_vInit(uint8_t ucIndex)

{
#ifdef HI2C_SPECIAL_INIT 
   HI2C0_vInitPort(); /* Initialize the special features of the periphery. */

#endif
   /* preset for port latch */
   HI2C0_vInputSCL();
   HI2C0_vInputSDA();

}

/* HI2C0_bSetAddr() */
bool HI2C0_bSetAddr(uint8_t ucAddress)

{
   /* no errors so far */
   HI2C0_ucError = 0u;

   HI2C0_vSetSDA();
   HI2C0_vSetSCL();

   /* generate start condition */
   HI2C0_vOutputSDA();
   HI2C0_vOutputSCL();
   HI2C0_vBitDelayH();
   HI2C0_vClrSDA()   ;
   HI2C0_vBitDelayH();
   HI2C0_vClrSCL()   ;
   HI2C0_vBitDelayL();

   /* transmit address */
   return HI2C0_bSetTxData(ucAddress, 0u);

}

/* HI2C0_bSetTxData() */
bool HI2C0_bSetTxData(uint8_t ucDataByte, bool bStop)

{
   uint8_t ucCounter;

   /* transmit all 8 data bits */
   ucCounter = 8u;

   do
   {
      /* send each bit, MSB first */

      if ((ucDataByte & 0x80u) != 0u)

      {
         HI2C0_vSetSDA();

      }
      else
      {
         HI2C0_vClrSDA();

      }
      ucDataByte = (uint8_t)(ucDataByte << 1u);

      /* generate clock */
      HI2C0_vSetSCL();
      HI2C0_vBitDelayH();
      HI2C0_vClrSCL();
      HI2C0_vBitDelayL();
      --ucCounter;

   }
   while (ucCounter > 0u);

   /* listen for ACK */
   HI2C0_vSetSDA();
   HI2C0_vInputSDA();
   HI2C0_vWaitForSlave();

   if (HI2C0_bGetSDA() != 0u)

   {
      /* ack didn't happen, may be nothing out there */
      HI2C0_ucError |= HI2C_NACK;

   }
   HI2C0_vClrSCL();
   HI2C0_vBitDelayL();
   HI2C0_vSetSDA();
   HI2C0_vOutputSDA();

   if ((bStop != 0u) || (HI2C0_ucError != 0u))

   {
      /* generate stop condition */
      HI2C0_vMakeStopCondition();

   }
   /* call event handler */

   if (HI2C0_bEventEnabled != 0u)

   {
      HI2C0_vHandleEvent();

   }
   return (bool)(HI2C0_ucError == 0u);

}

/* HI2C0_vTriggerReceive() */
uint8_t HI2C0_vTriggerReceive(bool bStop)

{
   uint8_t ucCounter;

   /* switch to input since we want to receive data */
   HI2C0_vInputSDA();

   /* receive the bits -- starting with the MSB */
   ucCounter = 8u;

   do {
      HI2C0_vSetSCL();
      HI2C0_vBitDelayH();

      HI2C0_ucLastRx <<= 1u;

      if (HI2C0_bGetSDA() != 0u) {
         HI2C0_ucLastRx |= 1u;
      }
      HI2C0_vClrSCL();
      HI2C0_vBitDelayL();
      --ucCounter;
   }
   while (ucCounter > 0u);

   /* send ACK according to the stop flag */
   HI2C0_vOutputSDA();

   if (bStop != 0u) {
      /* no acknowledge */
      HI2C0_vSetSDA();
   }
   else {
      /* acknowledge */
      HI2C0_vClrSDA();
   }
   HI2C0_vWaitForSlave();
   HI2C0_vClrSCL();
   HI2C0_vSetSDA();
   HI2C0_vBitDelayL();

   if (bStop != 0u) {
      HI2C0_vMakeStopCondition();
   }
   /* call event handler */

   if (HI2C0_bEventEnabled != 0u) {
      HI2C0_vHandleEvent();
   }
   return HI2C0_ucLastRx;
}

/* HI2C_vSendStop() */
void HI2C0_vSendStop(void)

{
   /* Send stop-condition.  Changing of SDA during SCL = OUTPUT_HIGH is */
   /* only allowed while sending start- or stop-condition.  For generating */
   /* stop-condition both SDA and SCL must be LOW */
   HI2C0_vClrSCL();
   HI2C0_vClrSDA();
   HI2C0_vOutputSCL();
   HI2C0_vOutputSDA();

   HI2C0_vBitDelayL();              /* clock low period */
   HI2C0_vSetSCL();              /* change SCL edge to OUTPUT_HIGH level */
   HI2C0_vBitDelayH();              /* Stop condition setup time */
   HI2C0_vSetSDA();;             /* change SDA edge to OUTPUT_HIGH level */

   HI2C0_vInputSCL();
   HI2C0_vInputSDA();

}

/* HI2C0_bForceBusRelease() */
bool HI2C0_bForceBusRelease(void)

{
   uint16_t uiCounter;
   bool bSdaStatus;

   HI2C0_vOutputSCL();
   HI2C0_vInputSDA();

   for (uiCounter = 0u; uiCounter < 15u; uiCounter++)

   {
      HI2C0_vClrSCL();          /* security: send some clocks to free the bus-lines */
      HI2C0_vBitDelayL();          /* clock low period */
      HI2C0_vSetSCL();
      HI2C0_vBitDelayH();

   }
   HI2C0_vSendStop();

   /* check BUS-lines */
   for (uiCounter = 0u; uiCounter < 5u; uiCounter++)

   {
      bSdaStatus = HI2C0_bGetSDA();

      if ( bSdaStatus != 0u )

      {
         break;

      }
   }
   return (bSdaStatus);

}

/* ------ Test instrumentation ------ */
//#if defined(MODULE_TEST)
//#include "hi2c01_Stub.c"
//#endif
/* ---------------------------------- */
