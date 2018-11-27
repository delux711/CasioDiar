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
   PROCESSOR        ARM Cortex-M3
   COMPILER         RealView
 *****************************************************************************************************************
   AUTHOR           Simku Frantisek
   CREATED          20091016
   LAST CHANGE      %PRT%
   LAST REVISION    %PR%
   STATUS           %PS%
 *****************************************************************************************************************
   DESCRIPTION
     Configuration for I2C.
     
 *****************************************************************************************************************
   CHANGES
   %PL%

 *****************************************************************************************************************/

#ifndef __HI2C01XS_H
#define __HI2C01XS_H
#include <stdio.h>
#include <stdbool.h>

        
/* DEFINITIONS ****************************************************************************************/
/* TYPE DEFINITIONS ***********************************************************************************/
/* GLOBAL FUNCTIONS **********************************************************************************/

extern uint32_t HI2C_ulNewPortStatus;


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
extern void HI2C0_vInitPort(void);


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

extern void HI2C0_vOutputSCL(void);


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

extern void HI2C0_vOutputSDA(void);


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

extern void HI2C0_vInputSCL(void);


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

extern void HI2C0_vInputSDA(void);


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

extern void HI2C0_vClrSCL(void);


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

extern void HI2C0_vSetSCL(void);


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

extern void HI2C0_vClrSDA(void);


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

extern void HI2C0_vSetSDA(void);


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

extern bool HI2C0_bGetSCL(void);


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

extern bool HI2C0_bGetSDA(void);


#endif
