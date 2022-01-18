/*******************************************************************************
* This file has been modified by ASH WARE Inc. as allowed per the original 
* license (see bottom of file), to add features and fix issues.
*******************************************************************************/

/*******************************************************************************
*
*  FREESCALE SEMICONDUCTOR INC. 2012-2014
*  ALL RIGHTS RESERVED
*  COPYRIGHT (c)
*
********************************************************************************
*
*  FILE NAME:  etpuc_cam.c
*
*  DESCRIPTION:
*    This eTPU function logs TCR2 values (engine angles) captured on input
*    transitions into an array. This log can be used e.g. to recognize between  
*    engine half-cycle 0-360 or 360-720. 
*
*******************************************************************************/

/*******************************************************************************
*  Includes
*******************************************************************************/
#include <etpu_std.h>
#include "etpuc_cam.h"
#include "etpuc_crank.h"
#include "etpuc_set.h"

/*******************************************************************************
*  eTPU Function Parameters:
*
*   log          - array of input transitions captured. Each log consist of
*                  - transition polarity (0-falling, 1-rising) in upper 8 bits
*                  - transition TCR2 angle in lower 24 bits.
*   log_size     - Size of the cam_log array - number of 32-bit words allocated
*                  in eTPU DATA RAM for cam_log.
*   log_idx      - index to the cam_log array where the next log will be written
*                  to. It is reset to zero on a link (from Crank).
*   log_count    - number of logs during the last log cycle (cam_log_idx is
*                  copied to cam_log_count before resetting).   
*   error        - Error status bits. Any time a bit is set, the channel IRQ is
*                  raised. Written by eTPU, cleared by CPU.
*
********************************************************************************
*
*  Channel Flag usage
*    Flag0 is not used.
*    Flag1 is not used.
*
*******************************************************************************/


/*******************************************************************************
*  eTPU Function
*******************************************************************************/

/**************************************************************************
* THREAD NAME: INIT
* DESCRIPTION: Initialize the channel to run the CAM function.
**************************************************************************/
_eTPU_thread CAM::INIT(_eTPU_matches_disabled)
{
	/* Stop the channel */
	/* Disable event handling */
	channel.MTD = MTD_DISABLE;
	/* Disable match detection */
	channel.MRLE = 0;
	/* Reset all latches */
	channel.TDL = TDL_CLEAR;
	channel.LSR = LSR_CLEAR;
	channel.MRLA = MRL_CLEAR;
	channel.MRLB = MRL_CLEAR;

	/* Initialize the channel */
	/* Set channel mode: single match single transition */
	channel.PDCM = PDCM_SM_ST;
	/* Time base selection */
	channel.TBSA = TBS_M1C2GE;
	channel.TBSB = TBS_M1C2GE;
	/* Input pin action control */
	if(cc.FM1 == CAM_FM1_LOG_RISING)
	{
		if(cc.FM0 == CAM_FM0_LOG_FALLING)
		{
			channel.IPACA = IPAC_EITHER;
			channel.IPACB = IPAC_EITHER;
		}
		else
		{
			channel.IPACA = IPAC_RISING;
			channel.IPACB = IPAC_RISING;
		}
	}
	else
	{
		if(cc.FM0 == CAM_FM0_LOG_FALLING)
		{
			channel.IPACA = IPAC_FALLING;
			channel.IPACB = IPAC_FALLING;
		}
		else
		{
			channel.IPACA = IPAC_NO_DETECT;
			channel.IPACB = IPAC_NO_DETECT;
		}
	}
	/* Output pin action control */
	channel.OPACA = OPAC_NO_CHANGE;
	channel.OPACB = OPAC_NO_CHANGE;

	/* Enable event handling */
	channel.MTD = MTD_ENABLE;	
}

/**************************************************************************
* THREAD NAME: RESET
* DESCRIPTION: Reset the log. Test if count == 0.
*              The link from Cam is expected once per cycle.	
**************************************************************************/
_eTPU_thread CAM::RESET(_eTPU_matches_enabled)
{
	log_count = log_idx;
	log_idx = 0;
	channel.LSR = LSR_CLEAR;
	
	if(log_count == 0)
	{
	  /* there was zero transitions logged from the previous reset */
	  error |= CAM_ERROR_ZERO_TRANS;
	  channel.CIRC = CIRC_INT_FROM_SERVICED;
	}
}

/**************************************************************************
* THREAD NAME: LOG A SINGLE TRANSITION
* DESCRIPTION: Log a transition into the log array.
**************************************************************************/
_eTPU_thread CAM::LOG_1_TRANS(_eTPU_matches_enabled)
{
  struct CAM_LOG *ptr;
	
	channel.TDL = TDL_CLEAR;			
	if(log_idx < log_size)
	{
		ptr = log + log_idx++;
		ptr->trans = CAM_FALLING;
		if(channel.PSS == 1)
		{
			ptr->trans = CAM_RISING;
		}
		ptr->angle = erta;
	}
	else
	{
		/* there is no more space in the log_array */
		error |= CAM_ERROR_LOG_OVERFLOW;
		channel.CIRC = CIRC_INT_FROM_SERVICED;
	}
}

/**************************************************************************
* THREAD NAME: LOG TWO TRANSITIONS
* DESCRIPTION: Log two captured transitions into the log array.
*              This may be executed only if the first transition was not
*              serviced yet.		
**************************************************************************/
_eTPU_thread CAM::LOG_2_TRANS(_eTPU_matches_enabled)
{
  struct CAM_LOG *ptr;

	channel.TDL = TDL_CLEAR;			
	if(log_idx < log_size)
	{
		ptr = log + log_idx++;
	  ptr->trans = CAM_FALLING;
#if defined(__TARGET_ETPU1__)
	  if(channel.PSS == 1)
#else
	  if(channel.PRSS == 1)
#endif
	  {
		  ptr->trans = CAM_RISING;
		}
	  ptr->angle = erta;

		if(log_idx < log_size)
		{
			log_idx++;
			ptr++;
		  ptr->trans = CAM_FALLING;
		  if(channel.PSS == 1)
		  {
			  ptr->trans = CAM_RISING;
			}
		  ptr->angle = ertb;
		}
		else
		{
		  /* there is no more space in the log_array */
		  error |= CAM_ERROR_LOG_OVERFLOW;
		  channel.CIRC = CIRC_INT_FROM_SERVICED;
		}
	}
	else
	{
	  /* there is no more space in the log_array */
	  error |= CAM_ERROR_LOG_OVERFLOW;
	  channel.CIRC = CIRC_INT_FROM_SERVICED;
	}
}


DEFINE_ENTRY_TABLE(CAM, CAM, standard, inputpin, autocfsr)
{
    //           HSR LSR M1 M2 PIN F0 F1 vector
    ETPU_VECTOR1(7,  x,  x, x, x,  x, x, INIT),

    //           HSR LSR M1 M2 PIN F0 F1 vector
    ETPU_VECTOR1(1,  x,  x, x, 0,  0, x, RESET),
    ETPU_VECTOR1(1,  x,  x, x, 0,  1, x, RESET),
    ETPU_VECTOR1(1,  x,  x, x, 1,  0, x, RESET),
    ETPU_VECTOR1(1,  x,  x, x, 1,  1, x, RESET),
    ETPU_VECTOR1(0,  1,  1, 1, x,  0, x, RESET),
    ETPU_VECTOR1(0,  1,  1, 1, x,  1, x, RESET),
    ETPU_VECTOR1(0,  1,  0, 0, 0,  0, x, RESET),
    ETPU_VECTOR1(0,  1,  0, 0, 0,  1, x, RESET),
    ETPU_VECTOR1(0,  1,  0, 0, 1,  0, x, RESET),
    ETPU_VECTOR1(0,  1,  0, 0, 1,  1, x, RESET),
    ETPU_VECTOR1(0,  1,  0, 1, x,  0, x, RESET),
    ETPU_VECTOR1(0,  1,  0, 1, x,  1, x, RESET),
    ETPU_VECTOR1(0,  1,  1, 0, x,  0, x, RESET),
    ETPU_VECTOR1(0,  1,  1, 0, x,  1, x, RESET),

    //           HSR LSR M1 M2 PIN F0 F1 vector
    ETPU_VECTOR1(0,  0,  0, 1, 0,  0, x, LOG_1_TRANS),
    ETPU_VECTOR1(0,  0,  0, 1, 0,  1, x, LOG_1_TRANS),
    ETPU_VECTOR1(0,  0,  0, 1, 1,  0, x, LOG_1_TRANS),
    ETPU_VECTOR1(0,  0,  0, 1, 1,  1, x, LOG_1_TRANS),

    //           HSR LSR M1 M2 PIN F0 F1 vector
    ETPU_VECTOR1(0,  0,  1, 1, 0,  0, x, LOG_2_TRANS),
    ETPU_VECTOR1(0,  0,  1, 1, 0,  1, x, LOG_2_TRANS),
    ETPU_VECTOR1(0,  0,  1, 1, 1,  0, x, LOG_2_TRANS),
    ETPU_VECTOR1(0,  0,  1, 1, 1,  1, x, LOG_2_TRANS),

    // unused/invalid entries
    ETPU_VECTOR1(2,  x,  x, x, x,  x, x, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(3,  x,  x, x, x,  x, x, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(4,  x,  x, x, x,  x, x, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(5,  x,  x, x, x,  x, x, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(6,  x,  x, x, x,  x, x, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,  0,  1, 0, 0,  0, x, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,  0,  1, 0, 0,  1, x, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,  0,  1, 0, 1,  0, x, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,  0,  1, 0, 1,  1, x, _Error_handler_unexpected_thread),
};


/*******************************************************************************
*  Export interface information to Host CPU program. 
*******************************************************************************/
/* [MISRA 2004 Rule 3.4] usage of #pragma write documented in the Pragma Write 
   Manual, see https://www.ashware.com/Manuals */
#pragma write h, (::ETPUfilename (cpu/etpu_cam_auto.h));
#pragma write h, (/**************************************************************** );
#pragma write h, (* WARNING: This file is automatically generated. DO NOT EDIT IT! );
#pragma write h, (*);
#pragma write h, (* FILE NAME: etpu_cam_auto.h  );
#if defined(__TARGET_ETPU1__)
#pragma write h, (* ARCHITECTURE: eTPU );
#else
#pragma write h, (* ARCHITECTURE: eTPU2 );
#endif
#pragma write h, (*);
#pragma write h, (* This file was generated by: __FILE__ on __DATE__, __TIME__ );
#pragma write h, (*);
#pragma write h, (* This file provides an interface between eTPU code and CPU       );
#pragma write h, (* code. All references to the eTPU function should be made with   );
#pragma write h, (* information in this file. This allows only symbolic             );
#pragma write h, (* information to be referenced which allows the eTPU code to be   );
#pragma write h, (* optimized without effecting the CPU code.                       );
#pragma write h, (*****************************************************************/);
#pragma write h, (#ifndef _ETPU_CAM_AUTO_H_ );
#pragma write h, (#define _ETPU_CAM_AUTO_H_ );
#pragma write h, ( );
#pragma write h, (/* Function Configuration Information */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_FUNCTION_NUMBER) ::ETPUfunctionnumber(CAM) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_TABLE_SELECT) ::ETPUentrytype(CAM) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_NUM_PARMS) ::ETPUram(CAM) );
#pragma write h, ( );
#pragma write h, (/* Host Service Request Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_HSR_INIT)   CAM_HSR_INIT );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_HSR_RESET)  CAM_HSR_RESET );
#pragma write h, ( );
#pragma write h, (/* Function Mode Bit Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_FM0_LOG_FALLING)  (CAM_FM0_LOG_FALLING) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_FM1_LOG_RISING)   (CAM_FM1_LOG_RISING << 1) );
#pragma write h, ( );
#pragma write h, (/* Parameter Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_OFFSET_LOG       ) ::ETPUlocation (CAM, log      ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_OFFSET_LOG_SIZE  ) ::ETPUlocation (CAM, log_size ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_OFFSET_LOG_IDX   ) ::ETPUlocation (CAM, log_idx  ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_OFFSET_LOG_COUNT ) ::ETPUlocation (CAM, log_count) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_OFFSET_ERROR     ) ::ETPUlocation (CAM, error    ) );
#pragma write h, ( );
#pragma write h, (/* Cam Log */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_LOG_ANGLE_MASK    ) 0x00FFFFFF );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_LOG_TRANS_MASK    ) (1<<24) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_LOG_TRANS_FALLING ) (CAM_FALLING<<24) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_LOG_TRANS_RISING  ) (CAM_RISING<<24) );
#pragma write h, ( );
#pragma write h, (/* Errors */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_ERROR_NO           ) CAM_ERROR_NO           );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_ERROR_ZERO_TRANS   ) CAM_ERROR_ZERO_TRANS   );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CAM_ERROR_LOG_OVERFLOW ) CAM_ERROR_LOG_OVERFLOW );
#pragma write h, ( );
#pragma write h, (#endif );

/*********************************************************************
 *
 * Copyright:
 *	Freescale Semiconductor, INC. All Rights Reserved.
 *  You are hereby granted a copyright license to use, modify, and
 *  distribute the SOFTWARE so long as this entire notice is
 *  retained without alteration in any modified and/or redistributed
 *  versions, and that such modified versions are clearly identified
 *  as such. No licenses are granted by implication, estoppel or
 *  otherwise under any patents or trademarks of Freescale
 *  Semiconductor, Inc. This software is provided on an "AS IS"
 *  basis and without warranty.
 *
 *  To the maximum extent permitted by applicable law, Freescale
 *  Semiconductor DISCLAIMS ALL WARRANTIES WHETHER EXPRESS OR IMPLIED,
 *  INCLUDING IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A
 *  PARTICULAR PURPOSE AND ANY WARRANTY AGAINST INFRINGEMENT WITH
 *  REGARD TO THE SOFTWARE (INCLUDING ANY MODIFIED VERSIONS THEREOF)
 *  AND ANY ACCOMPANYING WRITTEN MATERIALS.
 *
 *  To the maximum extent permitted by applicable law, IN NO EVENT
 *  SHALL Freescale Semiconductor BE LIABLE FOR ANY DAMAGES WHATSOEVER
 *  (INCLUDING WITHOUT LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS,
 *  BUSINESS INTERRUPTION, LOSS OF BUSINESS INFORMATION, OR OTHER
 *  PECUNIARY LOSS) ARISING OF THE USE OR INABILITY TO USE THE SOFTWARE.
 *
 *  Freescale Semiconductor assumes no responsibility for the
 *  maintenance and support of this software
 ********************************************************************/

/*******************************************************************************
*
*  REVISION HISTORY:
*
*  FILE OWNER: Milan Brejl [r54529]
*  Revision 1.0  2014/03/16  r54529
*  Minor comment and formating improvements. MISRA compliancy check.
*  Ready for eTPU Engine Control Library release 1.0.
*
*  Revision 0.1  2012/06/12  r54529
*  Initial version.
*
*******************************************************************************/
