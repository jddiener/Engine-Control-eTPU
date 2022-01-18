/*******************************************************************************
* This file has been modified by ASH WARE Inc. as allowed per the original 
* license (see bottom of file), to add features and fix issues.
*******************************************************************************/

/*******************************************************************************
*
*  FREESCALE SEMICONDUCTOR INC. 2013-2014
*  ALL RIGHTS RESERVED
*  COPYRIGHT (c)
*
********************************************************************************
*
*  FILE NAME:  etpuc_tg.c
*
*  DESCRIPTION:
*    This eTPU function generates a Crank & Cam tooth pattern.
*
*******************************************************************************/

/*******************************************************************************
*  Includes
*******************************************************************************/
#include <etpu_std.h>
#include "etpuc_tg.h"
#include "etpuc_set.h"

/*******************************************************************************
*  eTPU Function Parameters:
*
*   tooth_tcr1_time        - TCR1 time of the first tooth edge
*   tooth_period_actual    - TCR1 tooth period of the actual tooth
*   tooth_period_target    - TCR1 tooth period to be reached by 
*                            an acceleration/deceleration profile
*   accel_ratio            - an unsigned fractional value determining the 
*                            acceleration/deceleration. 
*                            Each tooth, the tooth_period_actual is updated:
*                              tooth_period_actual += accel_ratio * (tooth_period_target - tooth_period_actual)
*   *p_cam_tooth_first     - pointer to the first tooth number where the Cam 
*                            is toggled, in an array
*   *p_cam_tooth           - pointer to the next tooth number where the Cam 
*                            will be toggled
*   teeth_till_gap         - number of physical teeth gap to gap
*   teeth_in_gap           - number if missing teeth in the gap.
*                            If there is an additional tooth instead of the gap,
*                            this parameter must be set to 0.
*   teeth_per_cycle        - number of teeth (including missing teeth in gap)
*                            per an engine cycle (720 degrees). It must be
*                            a multiple of (teeth_till_gap + teeth_in_gap).
*   tooth_counter_gap      - it counts from 1 to teeth_till_gap + teeth_in_gap
*   tooth_counter_cycle    - it counts from 1 to teeth_per_cycle
*   cam_chan               - Cam channel number
*   generation_disable     - disables the generation of Crank output.
*
********************************************************************************
*
*  Channel Flag usage
*    Flag0 is not used.
*    Flag1 is not used.
*
********************************************************************************
*
*  Channel Function Mode (FM) bits usage
*    FM0 is used to select the initial output polarity:
*      - TG_FM0_POLARITY_LOW
*      - TG_FM0_POLARITY_HIGH
*    FM1 is used to identify CRANK and CAM channels:
*      - TG_FM1_CRANK
*      - TG_FM1_CAM
*
********************************************************************************
*
*  Channel Interrupt usage
*    The channel interrupt on the TG channel is set in each CRANK gap. 
*
*******************************************************************************/

/*******************************************************************************
*  eTPU Function
*******************************************************************************/

/**************************************************************************
* THREAD NAME: INIT
* DESCRIPTION: Initialize the channel to run the TG function.
**************************************************************************/
_eTPU_thread TG::INIT(_eTPU_matches_disabled)
{
	/* Stop the channel */
	/* Disable event handling */
	channel.MTD = MTD_DISABLE;
	/* Disable match detection */
	channel.MRLE = MRLE_DISABLE;
	/* Reset all latches */
	channel.TDL = TDL_CLEAR;
	channel.LSR = LSR_CLEAR;
	channel.MRLA = MRL_CLEAR;
	channel.MRLB = MRL_CLEAR;

	/* Initialize the channel */
	/* Set channel mode: either match non-blocking single transition */
	channel.PDCM = PDCM_EM_NB_ST;
	/* Time base selection */
	channel.TBSA = TBS_M1C1GE;
	channel.TBSB = TBS_M1C1GE;
	/* Input pin action control */
	channel.IPACA = IPAC_NO_DETECT;
	channel.IPACB = IPAC_NO_DETECT;
	/* Output pin action control */
	if(cc.FM0 == TG_FM0_POLARITY_LOW)
	{
		channel.PIN = PIN_SET_LOW;
		channel.OPACA = OPAC_MATCH_HIGH;
		channel.OPACB = OPAC_MATCH_LOW;
	}
	else
	{
		channel.PIN = PIN_SET_HIGH;
		channel.OPACA = OPAC_MATCH_LOW;
		channel.OPACB = OPAC_MATCH_HIGH;
	}
	/* Enable output pin buffer */
	channel.TBSA = TBSA_SET_OBE;

	if(cc.FM1 == TG_FM1_CRANK)
	{
		/* Output pin action control */
		if((tooth_period_target <= 0)
		|| (generation_disable == TG_GENERATION_DISABLED))
		{
			channel.OPACA = OPAC_NO_CHANGE;
			channel.OPACB = OPAC_NO_CHANGE;
		}
		/* reset values */
		tooth_counter_cycle = 1;
		tooth_counter_gap = 1;
		p_cam_tooth = p_cam_tooth_first;
		tooth_tcr1_time = tcr1 + tooth_period_actual;

		/* Schedule Match A - the first tooth */
		erta = tooth_tcr1_time;
		channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
		
		/* Enable event handling */
		channel.MTD = MTD_ENABLE;
	}
}

/**************************************************************************
* THREAD NAME: FIRST_EDGE
* DESCRIPTION: Update counters, schedule next FIRST_EDGE, care of gap,
*              possibly toggle Cam output.
**************************************************************************/
_eTPU_thread TG::FIRST_EDGE(_eTPU_matches_disabled)
{
	/* Count till gap */
	tooth_counter_gap++;
	if(tooth_counter_gap > (teeth_till_gap + teeth_in_gap))
	{
		/* restart gap */
		tooth_counter_gap = 1;
		channel.CIRC = CIRC_INT_FROM_SERVICED;
	}
	/* Calculate acceleration/deceleration */
	if(tooth_period_target > 0)
	{
		if (tooth_period_actual > tooth_period_target)
		{
			tooth_period_actual -= muliur(tooth_period_actual - tooth_period_target, accel_ratio) + 1;
		}
		if (tooth_period_actual < tooth_period_target)
		{
			tooth_period_actual += muliur(tooth_period_target - tooth_period_actual, accel_ratio) + 1;
		}
	}
	/* Tooth or gap? */
	if((tooth_period_target <= 0)
	|| (generation_disable == TG_GENERATION_DISABLED))
	{
		channel.OPACA = OPAC_NO_CHANGE;
		channel.OPACB = OPAC_NO_CHANGE;
	}
	else
	{
		if(cc.FM0 == TG_FM0_POLARITY_LOW)
		{
			channel.OPACA = OPAC_MATCH_HIGH;
			channel.OPACB = OPAC_MATCH_LOW;
		}
		else
		{
			channel.OPACA = OPAC_MATCH_LOW;
			channel.OPACB = OPAC_MATCH_HIGH;
		}
	}
	
	/* Schedule the tooth */
	erta = tooth_tcr1_time + tooth_period_actual;
	tooth_tcr1_time = erta;
	ertb = erta - (tooth_period_actual >> 1);
	channel.MRLA = MRL_CLEAR;
	channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
	channel.MRLB = MRL_CLEAR;
	channel.ERWB = ERW_WRITE_ERT_TO_MATCH;
	
	/* Toggle CAM output */
	if(tooth_counter_cycle == *p_cam_tooth)
	{
		p_cam_tooth++;
		chan = cam_chan;
		channel.OPACA = OPAC_MATCH_TOGGLE;
		channel.PIN = PIN_AS_OPACA;
	}
	/* Count cycle */
	tooth_counter_cycle++;
	if(tooth_counter_cycle > teeth_per_cycle)
	{
		/* restart cycle */
		tooth_counter_cycle = 1;
		p_cam_tooth = p_cam_tooth_first;
	}
}

/**************************************************************************
* THREAD NAME: SECOND_EDGE
* DESCRIPTION: Schedule next SECOND_EDGE.
**************************************************************************/
_eTPU_thread TG::SECOND_EDGE(_eTPU_matches_disabled)
{
	channel.MRLB = MRL_CLEAR;
	
	/* Tooth or gap? */
	if((tooth_counter_gap > teeth_till_gap)
	|| (tooth_period_target <= 0)
	|| (generation_disable == TG_GENERATION_DISABLED))
	{
		channel.OPACA = OPAC_NO_CHANGE;
		channel.OPACB = OPAC_NO_CHANGE;
	}
	else
	{
		if(cc.FM0 == TG_FM0_POLARITY_LOW)
		{
			channel.OPACA = OPAC_MATCH_HIGH;
			channel.OPACB = OPAC_MATCH_LOW;
		}
		else
		{
			channel.OPACA = OPAC_MATCH_LOW;
			channel.OPACB = OPAC_MATCH_HIGH;
		}
	}
}


DEFINE_ENTRY_TABLE(TG, TG, alternate, outputpin, autocfsr)
{
	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR2(6,7,   x,  x, x, x,  x, x, INIT),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  1, 0, 0,  0, 0, FIRST_EDGE),
	ETPU_VECTOR1(0,     x,  1, 0, 0,  1, 0, FIRST_EDGE),
	ETPU_VECTOR1(0,     x,  1, 0, 0,  0, 1, FIRST_EDGE),
	ETPU_VECTOR1(0,     x,  1, 0, 0,  1, 1, FIRST_EDGE),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  0, 0, FIRST_EDGE),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  1, 0, FIRST_EDGE),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  0, 1, FIRST_EDGE),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  1, 1, FIRST_EDGE),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  0, 0, FIRST_EDGE),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  1, 0, FIRST_EDGE),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  0, 1, FIRST_EDGE),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  1, 1, FIRST_EDGE),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  0, 0, FIRST_EDGE),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  1, 0, FIRST_EDGE),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  0, 1, FIRST_EDGE),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  1, 1, FIRST_EDGE),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  0, 1, 0,  0, 0, SECOND_EDGE),
	ETPU_VECTOR1(0,     x,  0, 1, 0,  1, 0, SECOND_EDGE),
	ETPU_VECTOR1(0,     x,  0, 1, 0,  0, 1, SECOND_EDGE),
	ETPU_VECTOR1(0,     x,  0, 1, 0,  1, 1, SECOND_EDGE),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  0, 0, SECOND_EDGE),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  1, 0, SECOND_EDGE),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  0, 1, SECOND_EDGE),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  1, 1, SECOND_EDGE),

    // unused/invalid entries
	ETPU_VECTOR2(2,3,   x,  x, x, 0,  0, x, _Error_handler_unexpected_thread),
	ETPU_VECTOR2(2,3,   x,  x, x, 0,  1, x, _Error_handler_unexpected_thread),
	ETPU_VECTOR2(2,3,   x,  x, x, 1,  0, x, _Error_handler_unexpected_thread),
	ETPU_VECTOR2(2,3,   x,  x, x, 1,  1, x, _Error_handler_unexpected_thread),
	ETPU_VECTOR3(1,4,5, x,  x, x, x,  x, x, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     1,  0, 0, 0,  x, x, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     1,  0, 0, 1,  x, x, _Error_handler_unexpected_thread),
};


/*******************************************************************************
*  Export interface information to Host CPU program.
*******************************************************************************/
/* [MISRA 2004 Rule 3.4] usage of #pragma write documented in the Pragma Write 
   Manual, see https://www.ashware.com/Manuals */
#pragma write h, (::ETPUfilename (cpu/etpu_tg_auto.h));
#pragma write h, (/**************************************************************** );
#pragma write h, (* WARNING: This file is automatically generated. DO NOT EDIT IT! );
#pragma write h, (*);
#pragma write h, (* FILE NAME: etpu_tg_auto.h  );
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
#pragma write h, (#ifndef _ETPU_TG_AUTO_H_ );
#pragma write h, (#define _ETPU_TG_AUTO_H_ );
#pragma write h, ( );
#pragma write h, (/* Function Configuration Information */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_FUNCTION_NUMBER) ::ETPUfunctionnumber(TG) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_TABLE_SELECT) ::ETPUentrytype(TG) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_NUM_PARMS) ::ETPUram(TG) );
#pragma write h, ( );
#pragma write h, (/* Host Service Request Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_HSR_INIT)         TG_HSR_INIT );
#pragma write h, ( );
#pragma write h, (/* Function Mode Bit Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_FM0_POLARITY_LOW)  TG_FM0_POLARITY_LOW );
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_FM0_POLARITY_HIGH) TG_FM0_POLARITY_HIGH );
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_FM1_CRANK)         (TG_FM1_CRANK << 1) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_FM1_CAM)           (TG_FM1_CAM << 1) );
#pragma write h, ( );
#pragma write h, (/* Parameter Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_OFFSET_TOOTH_TCR1_TIME    ) ::ETPUlocation (TG, tooth_tcr1_time    ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_OFFSET_TOOTH_PERIOD_ACTUAL) ::ETPUlocation (TG, tooth_period_actual) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_OFFSET_TOOTH_PERIOD_TARGET) ::ETPUlocation (TG, tooth_period_target) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_OFFSET_ACCEL_RATIO        ) ::ETPUlocation (TG, accel_ratio        ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_OFFSET_P_CAM_TOOTH_FIRST  ) ::ETPUlocation (TG, p_cam_tooth_first  ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_OFFSET_P_CAM_TOOTH        ) ::ETPUlocation (TG, p_cam_tooth        ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_OFFSET_TEETH_TILL_GAP     ) ::ETPUlocation (TG, teeth_till_gap     ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_OFFSET_TEETH_IN_GAP       ) ::ETPUlocation (TG, teeth_in_gap       ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_OFFSET_TEETH_PER_CYCLE    ) ::ETPUlocation (TG, teeth_per_cycle    ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_OFFSET_TOOTH_COUNTER_GAP  ) ::ETPUlocation (TG, tooth_counter_gap  ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_OFFSET_TOOTH_COUNTER_CYCLE) ::ETPUlocation (TG, tooth_counter_cycle) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_OFFSET_CAM_CHAN           ) ::ETPUlocation (TG, cam_chan           ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_OFFSET_GENERATION_DISABLE ) ::ETPUlocation (TG, generation_disable ) );
#pragma write h, ( );
#pragma write h, (/* Generation Disable Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_GENERATION_ALLOWED)         TG_GENERATION_ALLOWED);
#pragma write h, (::ETPUliteral(#define FS_ETPU_TG_GENERATION_DISABLED)        TG_GENERATION_DISABLED);
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
*  Revision 0.4  2013/10/29  r54529
*  generation_disable switch added.
*
*  Revision 0.3  2013/07/25  r54529
*  FM1 (polarity) option separated for Crank and Cam.
*
*  Revision 0.2  2013/06/19  r54529
*  Acceleration and deceleration added.
*
*  Revision 0.1  2012/11/27  r54529
*  Initial version.
*
*******************************************************************************/
