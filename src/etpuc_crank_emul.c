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
*  FILE NAME:  etpuc_crank_emul.c
*
*  DESCRIPTION:
*    This eTPU function generates an internal angle-base without processing any
*    input signal. The Angle Base is maintained in TCR2.
*    The CRANK_EMUL replaces the CRANK eTPU function for testing purposes, when
*    no Crank signal is available, and enables to deterministically set the
*    engine speed.
*    The local channel parameter structure is the same as CRANK uses. The only
*    difference is the fact, that the tooth_period is an input parameters, 
*    contrary to CRANK where it is a measured output value.
*    
*    CRANK_EMUL is limited to emulating tooth patterns with gap. Patterns 
*    including an additional tooth are not supported.
*    
*******************************************************************************/

/*******************************************************************************
*  Includes
*******************************************************************************/
#include <etpu_std.h>
#include "etpuc_crank.h"
#include "etpuc_set.h"

/*******************************************************************************
*  eTPU Function Parameters:
*
*   teeth_till_gap         - number of physical teeth gap to gap
*   teeth_in_gap           - number if missing teeth in the gap.
*                            Addition tooth is not supported.
*   teeth_per_cycle        - number of teeth (including missing teeth in gap)
*                            per an engine cycle (720 degrees). It must be
*                            a multiple of (teeth_till_gap + teeth_in_gap).
*   teeth_per_sync         - number of teeth (including missing teeth in gap)
*                            corresponding to a segment needed for Cam log.
*                            It must be a multiple of
*                            (teeth_till_gap + teeth_in_gap).
*   tooth_counter_gap      - it counts from 1 to teeth_till_gap
*   tooth_counter_cycle    - it counts from 1 to teeth_per_cycle
*   blank_teeth            - number of teeth ignored after initialization
*   blank_time             - TCR1 time period after initialization during which
*                            teeth are ignored,
*   tcr2_ticks_per_tooth   - number of TCR2 angle ticks per tooth
*   last_tooth_tcr1_time   - TCR1 time of the last tooth transition
*   last_tooth_period      - TCR1 period between last 2 teeth
*   tcr2_adjustment        - TCR2 angle value corresponding to the angle on the
*                            first tooth after gap, at which the PRE_FULL_SYNC
*                            state was set and CPU was asked to recognize the
*                            Cam log pattern.
*   gap_ratio              - fraction used to perform the ABA gap test:
*                              gap_ratio * tooth_period_B > tooth_period_A
*   win_ratio_normal       - fraction used to derive the acceptance window for
*                            the next normal tooth
*   win_ratio_across_gap   - fraction used to derive the acceptance window for
*                            the first tooth after the gap
*   win_ratio_after_gap    - fraction used to derive the acceptance window for
*                            the second tooth after the gap
*   win_ratio_after_timeout- fraction used to derive the acceptance window for
*                            the tooth following a timeout condition
*   first_tooth_timeout    - TCR1 time after the first tooth (after blank_teeth)
*                            when a timeout will be deemed to have happened
*   link_cam               - set of 4 link numbers to send to reset the Cam log
*                            (up to 4 Cam channel numbers)
*   link_1                 - the first  set of 4 link numbers to send on stall
*   link_2                 - the second set of 4 link numbers to send on stall
*   link_3                 - the third  set of 4 link numbers to send on stall
*   link_4                 - the fourth set of 4 link numbers to send on stall
*   state                  - used to keep track of the CRANK state. See header
*                            file for possible values.
*   error                  - crank error flags. See header file for individual
*                            bits meaning. The eTPU sets them, the CPU should
*                            read and clear.
*   *tooth_period_log      - pointer to an array of tooth periods.
*                            The array must include teeth_per_cycle items.
*   err2477_tcr2_target    - used to keep track of when the Angle Counter is not
*                            in high rate mode for errata 2477 workaround
*
********************************************************************************
*
*  Channel Flag usage
*    Flag0 is not used
*    Flag1 is not used.
*
********************************************************************************
*
*  Channel Function Mode (FM) bits usage
*    FM0 is not used
*    FM1 is used to turn on logging of crank tooth periods:
*      - CRANK_FM1_LOG_TOOTH_PERIODS
*
********************************************************************************
*
*  Channel Interrupt usage
*    The channel interrupt on the CRANK channel is set on the first tooth 
*    every engine cycle. 
*
*******************************************************************************/
#define CRANK_IPH_MASK   0x1000


/*******************************************************************************
*  eTPU Function
*******************************************************************************/
	uint24_t   tooth_period;

/**************************************************************************
* THREAD NAME: INIT
* DESCRIPTION: Initialize the channel to run the CRANK_EMUL function.
**************************************************************************/
_eTPU_thread CRANK::INIT_EMUL(_eTPU_matches_disabled)
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
	/* Set channel mode: either match non blocking single transition */
	channel.PDCM = PDCM_EM_NB_ST;
	/* Time base selection */
	channel.TBSA = TBS_M1C1GE;  /* capture time to erta*/
	channel.TBSB = TBS_M1C2GE;  /* capture angle to ertb */
	/* Input pin action control */
	channel.IPACA = IPAC_NO_DETECT;
	channel.IPACB = IPAC_NO_DETECT;
	/* Output pin action control */
	channel.OPACA = OPAC_MATCH_TOGGLE;
	channel.OPACB = OPAC_NO_CHANGE;

	/* Default values */
	eng_trr_norm = trr = 0xffffff;
	tpr = 0;
	tcr2 = 0;
	eng_pos_state = ENG_POS_SEEK;
	eng_cycle_tcr2_start = eng_cycle_tcr2_ticks;
	state = CRANK_SEEK;

	/* Enable event handling */
	channel.MTD = MTD_ENABLE;
}

/**************************************************************************
* THREAD NAME: ANGLE_ADJUST
* DESCRIPTION: Update TCR2 value, set ENG_POS_PRE_FULL_SYNC.
**************************************************************************/
_eTPU_thread CRANK::ANGLE_ADJUST_EMUL(_eTPU_matches_disabled)
{
	tcr2 += tcr2_adjustment;
}

/**************************************************************************
* THREAD NAME: SET_SPEED
* DESCRIPTION: On the first HSR_SET_SPEED command 
*              - set FULL_SYNC eng_pos_state
*              - start the angle counter running, using last_tooth_period
**************************************************************************/
_eTPU_thread CRANK::SET_SPEED_EMUL(_eTPU_matches_disabled)
{
	if(eng_pos_state != ENG_POS_FULL_SYNC)
	{
		/* set global eng_pos state */
		eng_pos_state = ENG_POS_FULL_SYNC;
		
		/* set state */
		state = CRANK_COUNTING;
		
		/* use user-defined tooth_period */
		tooth_period = last_tooth_period;
		
		/* schedule the first tooth match */
		erta = tcr1 + 1;
		last_tooth_tcr1_time = erta;
		channel.ERWA = ERW_WRITE_ERT_TO_MATCH;

		/* set TRR and TICKS */
		tpr = (uint16_t)(tcr2_ticks_per_tooth - 1U);
		Set_TRR(tooth_period);
		tcr2 = tcr2_ticks_per_tooth;
	}
}

/**************************************************************************
* THREAD NAME: CRANK_WITH_GAP
* DESCRIPTION: A transition or a timeout, handling a crank wheel with gap.
**************************************************************************/
_eTPU_thread CRANK::CRANK_WITH_GAP_EMUL(_eTPU_matches_enabled)
{
	uint8_t  tmp;
	
	channel.MRLA = MRL_CLEAR;
	switch(state)
	{
	case CRANK_SEEK:
	case CRANK_BLANK_TEETH:
	case CRANK_FIRST_TRANS:
	case CRANK_SECOND_TRANS:
	case CRANK_TEST_POSSIBLE_GAP:
	case CRANK_VERIFY_GAP:
	case CRANK_COUNTING_TIMEOUT:
		/* set state */
		state = CRANK_COUNTING;
		/* continue to normal processing at CRANK_COUNTING */

	case CRANK_COUNTING:
		/**************************************************************
		* STATE: T7 - COUNTING
		* DESCRIPTION: 
		*   Transition detected in normal window.
		*   Calculate tooth period and record transition time.
		*   Increment tooth counters.
		*   Check if the next tooth is the last before gap.
		*   Adjust TCR2 rate.
		*   Expect next transition in normal window. 
		**************************************************************/
		/* write IPH */
		tpr |= CRANK_IPH_MASK;
		/* use saved last_tooth_tcr1_time and tooth_period */
		erta = last_tooth_tcr1_time;
		tooth_period = last_tooth_period;
		/* increment tooth counters */
		tooth_counter_gap++;
		tooth_counter_cycle++;
		/* test if before the gap */
		if(tooth_counter_gap == teeth_till_gap - 1)
		{
			/* there is one more teeth till the gap */
			state = CRANK_TOOTH_BEFORE_GAP;
		}
		/* set TRR */
		Set_TRR(tooth_period);
		/* log tooth period */
		ToothArray_Log(tooth_period);
		/* schedule next tooth match */
		erta += tooth_period;
		last_tooth_tcr1_time = erta;
		channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
		break;

	case CRANK_TOOTH_BEFORE_GAP:
		/**************************************************************
		* STATE: T9 - TOOTH_BEFORE_GAP
		* DESCRIPTION: 
		*   Transition detected in normal window, gap expected next.
		*   Calculate tooth period and record transition time.
		*   Increment tooth counters.
		*   Adjust TCR2 rate.
		*   Expect next transition within window across the gap. 
		**************************************************************/
		/* write IPH */
		tpr |= CRANK_IPH_MASK;
		/* use saved last_tooth_tcr1_time and tooth_period */
		erta = last_tooth_tcr1_time;
		tooth_period = last_tooth_period;
		/* increment tooth counters */
		tooth_counter_gap++;
		tooth_counter_cycle++;
		/* set TRR */
		Set_TRR(tooth_period);
		/* log tooth period */
		ToothArray_Log(tooth_period);
		/* set state */
		state = CRANK_TOOTH_AFTER_GAP;
		/* write MISSCNT */
		tpr |= misscnt_mask;
		/* schedule next tooth match */
		erta += (tooth_period * (teeth_in_gap + 1U));
		last_tooth_tcr1_time = erta;
		channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
		break;

	case CRANK_TOOTH_AFTER_GAP:
		/**************************************************************
		* STATE: T11 - TOOTH_AFTER_GAP
		* DESCRIPTION: 
		*   Transition detected in window across gap.
		*   Calculate tooth period and record transition time.
		*   Verify the gap (AB of ABA test).
		*   If gap verified, adjust TCR2 rate and tooth counters,
		*     sync-cycle or engine-cycle is finished:
		*     In ENG_POS_FIRST_HALF_SYNC, 
		*       ask CPU to decode the Cam log, 
		*       reset TCR2, reset tooth_counter_cycle,
		*       set ENG_POS_PRE_FULL_SYNC and IRQ.
		*     In ENG_POS_PRE_FULL_SYNC, there was no response from CPU,
		*       reset Cam log, reset TCR2, reset tooth_counter_cycle,
		*       set ENG_POS_FIRST_HALF_SYNC and IRQ.
		*     In ENG_POS_FULL_SYNC,
		*       reset Cam log, reset tooth_counter_cycle, 
		*       set IRQ (once per cycle in full sync)
		*       increment eng_cycle_tcr2_start by one cycle
		*     Expect next transition in window after gap.
		*   Else, gap not verified, set CRANK_ERR_TOOTH_IN_GAP,
		*     set ENG_POS_SEEK and IRQ, signal output functions and
		*     restart searching for the gap  
		**************************************************************/
		/* write IPH */
		tpr |= CRANK_IPH_MASK;
		/* use saved last_tooth_tcr1_time and tooth_period */
		erta = last_tooth_tcr1_time;
		tooth_period = last_tooth_period;
		/* set TRR */
		Set_TRR(tooth_period);
		/* set state - if the second tooth after the gap times out then
		   the state machine will revert to FIRST_TRANS */
		state = CRANK_COUNTING_TIMEOUT;
		/* set tooth counters - first tooth after gap */
		tooth_counter_gap = 1;
		tmp = tooth_counter_cycle + teeth_in_gap;
		while(++tooth_counter_cycle <= tmp)
		{
			/* log average tooth period for all teeth in gap */
			ToothArray_Log(tooth_period);
		}
		/* if the engine cycle is finished */
		if(tooth_counter_cycle >= teeth_per_cycle)
		{
			/* set channel interrupt - once per cycle in full-sync */
#if defined(__TARGET_ETPU2__)
			channel.CIRC =  CIRC_BOTH_FROM_SERVICED;  /* on eTPU2, set also DMA request */
#else
			channel.CIRC =  CIRC_INT_FROM_SERVICED;
#endif
			/* reset tooth_counter_cycle */
			tooth_counter_cycle = 1;
			/* increment eng_cycle_tcr2_start by one cycle */
			eng_cycle_tcr2_start += eng_cycle_tcr2_ticks;
		}
		/* log tooth period (after possible tooth_counter_cycle reset) */
		ToothArray_Log(tooth_period);
		/* schedule next tooth match */
		erta += tooth_period;
		last_tooth_tcr1_time = erta;
		channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
		break;

	default:
		error |= CRANK_ERR_INTERNAL;
		break;
	}
}


DEFINE_ENTRY_TABLE(CRANK, CRANK_EMUL, alternate, outputpin, autocfsr)
{
	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR2(6,7,   x,  x, x, x,  x, x, INIT_EMUL),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR3(1,4,5, x,  x, x, x,  x, x, ANGLE_ADJUST_EMUL),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR2(2,3,   x,  x, x, 0,  0, x, SET_SPEED_EMUL),
	ETPU_VECTOR2(2,3,   x,  x, x, 0,  1, x, SET_SPEED_EMUL),
	ETPU_VECTOR2(2,3,   x,  x, x, 1,  0, x, SET_SPEED_EMUL),
	ETPU_VECTOR2(2,3,   x,  x, x, 1,  1, x, SET_SPEED_EMUL),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  1, 0, 0,  0, 0, CRANK_WITH_GAP_EMUL),
	ETPU_VECTOR1(0,     x,  1, 0, 0,  1, 0, CRANK_WITH_GAP_EMUL),
	ETPU_VECTOR1(0,     x,  1, 0, 0,  0, 1, CRANK_WITH_GAP_EMUL),
	ETPU_VECTOR1(0,     x,  1, 0, 0,  1, 1, CRANK_WITH_GAP_EMUL),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  0, 0, CRANK_WITH_GAP_EMUL),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  1, 0, CRANK_WITH_GAP_EMUL),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  0, 1, CRANK_WITH_GAP_EMUL),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  1, 1, CRANK_WITH_GAP_EMUL),

    // unused/invalid entries
	ETPU_VECTOR1(0,     1,  0, 0, 0,  x, x, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     1,  0, 0, 1,  x, x, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  0, 1, 0,  0, 0, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  0, 1, 0,  1, 0, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  0, 1, 0,  0, 1, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  0, 1, 0,  1, 1, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  0, 0, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  1, 0, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  0, 1, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  1, 1, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  0, 0, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  1, 0, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  0, 1, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  1, 1, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  0, 0, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  1, 0, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  0, 1, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  1, 1, _Error_handler_unexpected_thread),
};


/*******************************************************************************
*  Export interface information to Host CPU program.
*******************************************************************************/
/* [MISRA 2004 Rule 3.4] usage of #pragma write documented in the Pragma Write 
   Manual, see https://www.ashware.com/Manuals */
#pragma write h, (::ETPUfilename (cpu/etpu_crank_emul_auto.h));
#pragma write h, (/**************************************************************** );
#pragma write h, (* WARNING: This file is automatically generated. DO NOT EDIT IT! );
#pragma write h, (*);
#pragma write h, (* FILE NAME: etpu_crank_auto.h  );
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
#pragma write h, (#ifndef _ETPU_CRANK_EMUL_AUTO_H_ );
#pragma write h, (#define _ETPU_CRANK_EMUL_AUTO_H_ );
#pragma write h, ( );
#pragma write h, (/* Function Configuration Information */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_EMUL_FUNCTION_NUMBER) ::ETPUfunctionnumber(CRANK::CRANK_EMUL) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_EMUL_TABLE_SELECT) ::ETPUentrytype(CRANK::CRANK_EMUL) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_EMUL_NUM_PARMS) ::ETPUram(CRANK) );
#pragma write h, ( );
#pragma write h, (/* Host Service Request Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_HSR_SET_SPEED)    CRANK_HSR_SET_SPEED );
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
*  Revision 0.2  2013/11/27  r54529
*  Full precision of TRR calculation.
*  
*  Revision 0.1  2012/06/12  r54529
*  Initial version.
*
*******************************************************************************/
