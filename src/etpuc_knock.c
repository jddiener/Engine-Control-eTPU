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
*  FILE NAME:  etpuc_knock.c
*
*  DESCRIPTION:
*    This eTPU function is intended to support ADC sampling of a knock signal
*    in an engine control system. 
*    The function has 2 modes:
*    - Gate mode
*    - Trigger mode
*    
*    In the Gate mode a simple angle-based pulses are generated. The output
*    signal can be used to gate the ADC, running in a continues mode.
*    In the Trigger mode a 50% duty-cycle PWM signal is generated within
*    the angle-based window. The output signal can be used to trigger the ADC.
*    
*    The number of angle-based windows is configurable. The windows are
*    defined by an array of window structures, consisting of window start angle
*    and angle windows width.
*    
*    There is an KNOCK parameter tdc_angle, relative to which all windows are 
*    defined. Positive angles precede the tdc_angle, negative angles come after.
*    
*    The KNOCK function enables to selectively generate channel interrupts and/or
*    DMA requests at:
*    - window start
*    - window end
*    - every trigger pulse (Trigger mode only)
*    
*******************************************************************************/

/*******************************************************************************
*  Includes
*******************************************************************************/
#include <etpu_std.h>
#include "etpuc_knock.h"
#include "etpuc_crank.h"
#include "etpuc_set.h"

/*******************************************************************************
*  eTPU Function Parameters:
*
*  KNOCK Channel Parameters (struct KNOCK_CHAN_PARAMS)
*  -----------------------------------------------
*  window_count - count of windows per engine cycle
*  *p_window_first - pointer to the first knock window structure
*  window_counter - counts knock windows
*  *p_window - pointer to the current knock window structure
*  tdc_angle - TCR2 angle relative to engine-cycle start
*  tdc_angle_actual - absolute TDC TCR2 angle
*  tcr2_window_start - absolute TCR2 window start angle
*  tcr2_window_end - absolute TCR2 window end angle
*  trigger_period - TCR1 trigger period
*  irq_dma_options - IRQ and DMA request selection
*
*  Knock Window Structure Parameters (struct KNOCK_WINDOW)
*  -----------------------------------------------------
*  start - TDC-relative TCR2 angle start
*  width - knock window TCR2 angle width
*
********************************************************************************
*
*  Channel Flag usage
*    Flag0 = KNOCK_FLAG0_WINDOW_NOT_ACTIVE (0)
*    Flag0 = KNOCK_FLAG0_WINDOW_ACTIVE (1)
*    Flag0 = KNOCK_FLAG1_MODE_GATE (0)     ... Operates in Gate mode
*    Flag0 = KNOCK_FLAG1_MODE_TRIGGER (1)  ... Operates in Trigger mode
*
*******************************************************************************/


/*******************************************************************************
*  eTPU Class Methods/Fragments
*******************************************************************************/

/*******************************************************************************
*  FUNCTION NAME: ReadWindow
*  DESCRIPTION: Schedule the IRQ_ANGLE, set flag.
*******************************************************************************/
_eTPU_fragment KNOCK::ScheduleStartAngle_NoReturn(void)
{
	/* semi-assembly code for coherent read from knock window array */
    register_diob diob = (int24_t)p_window;
    NOP();
	/* [MISRA 2004 Rule 2.1] Assembly language shall be encapsulated and isolated */
#asm
    ram p <- by diob++.
    ram p <- by diob; alu ertb = p.
    alu erta = p.
#endasm

    /* Calculate absolute TCR2 angles */
	ertb = tdc_angle_actual - ertb;
	tcr2_window_start = ertb;
	tcr2_window_end = ertb + erta;
    /* Schedule start angle */
	channel.MRLB = MRL_CLEAR;
	channel.ERWB = ERW_WRITE_ERT_TO_MATCH;
	
	/* Increment p_window pointer */
	p_window++;
	/* Increment window counter */
	window_counter++;
	/* Last window in cycle? */
	if(window_counter >= window_count)
	{
		/* Reset window pointer and counter */
		window_counter = 0;
		p_window = p_window_first;
		/* Update actual TDC angle for next cycle */
		tdc_angle_actual += eng_cycle_tcr2_ticks;
	}
}


/*******************************************************************************
*  eTPU Function
*******************************************************************************/

/**************************************************************************
* THREAD NAME: INIT
* DESCRIPTION: Initialize the channel to run the KNOCK function.
**************************************************************************/
_eTPU_thread KNOCK::INIT(_eTPU_matches_disabled)
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
	/* Set channel mode: match 2 single transition
	   - Match B is serviced, Match A is not serviced */
	channel.PDCM = PDCM_M2_ST;
	/* Time base selection */
	channel.TBSA = TBS_M1C1GE;  /* match on time and capture time */
	channel.TBSB = TBS_M2C1GE;  /* match on angle and capture time */
	/* Input pin action control */
	channel.IPACA = IPAC_NO_DETECT;
	channel.IPACB = IPAC_NO_DETECT;
	/* Output pin state and Output pin action control */
	if(channel.FM0 == KNOCK_FM0_ACTIVE_HIGH)
	{
		channel.PIN = PIN_SET_LOW;
		channel.OPACA = OPAC_MATCH_LOW;
		channel.OPACB = OPAC_MATCH_HIGH;
	}
	else
	{
		channel.PIN = PIN_SET_HIGH;
		channel.OPACA = OPAC_MATCH_HIGH;
		channel.OPACB = OPAC_MATCH_LOW;
	}
	/* Enable output pin buffer */
	channel.TBSA = TBSA_SET_OBE;
	/* Channel flags */
	channel.FLAG0 = KNOCK_FLAG0_WINDOW_NOT_ACTIVE;
	/* Initialize actual TDC angle */
	tdc_angle_actual = eng_cycle_tcr2_start + tdc_angle;
	/* Initialize window pointer and counter */
	p_window = p_window_first;
	window_counter = 0;
	/* Enable event handling */
	channel.MTD = MTD_ENABLE;
	
    if (eng_pos_state != ENG_POS_FULL_SYNC)
    {
        /* only fully start the knock processing once sync is achieved */
        return;
    }
	
	/* Schedule the first start angle */
	ScheduleStartAngle_NoReturn();
}

/**************************************************************************
* THREAD NAME: STOP
* DESCRIPTION: Stop the running injection sequence.
*              The next injection sequence will start normally. 
**************************************************************************/
_eTPU_thread KNOCK::STOP(_eTPU_matches_disabled)
{
	/* Disable event handling */
	channel.MTD = MTD_DISABLE;
	/* Disable match detection */
	channel.MRLE = MRLE_DISABLE;
	/* Reset all latches */
	channel.TDL = TDL_CLEAR;
	channel.LSR = LSR_CLEAR;
	channel.MRLA = MRL_CLEAR;
	channel.MRLB = MRL_CLEAR;
	/* Output pin action control */
	if(channel.FM0 == KNOCK_FM0_ACTIVE_HIGH)
	{
		channel.PIN = PIN_SET_LOW;
	}
	else
	{
		channel.PIN = PIN_SET_HIGH;
	}
}

/**************************************************************************
* THREAD NAME: WINDOW_START
* DESCRIPTION: Window start angle.
**************************************************************************/
_eTPU_thread KNOCK::WINDOW_START(_eTPU_matches_disabled)
{
	/* IRQ & DMA at window start */
	if(irq_dma_options & KNOCK_IRQ_AT_WINDOW_START)
	{
		channel.CIRC = CIRC_INT_FROM_SERVICED;
	}
	if(irq_dma_options & KNOCK_DMA_AT_WINDOW_START)
	{
		channel.CIRC = CIRC_DATA_FROM_SERVICED;
	}

	/* Channel flag */
	channel.FLAG0 = KNOCK_FLAG0_WINDOW_ACTIVE;
	
	if(cc.FM1 == KNOCK_FM1_MODE_GATE)
	{
		/* Channel flag */
		channel.FLAG1 = KNOCK_FLAG1_MODE_GATE;
		/* Output pin action control */
		channel.OPACB = OPAC_MATCH_LOW;
		if(channel.FM0 == KNOCK_FM0_ACTIVE_LOW)
		{
			channel.OPACB = OPAC_MATCH_HIGH;
		}
		/* Schedule WINDOW_END */
		ertb = tcr2_window_end;
		channel.MRLB = MRL_CLEAR;
		channel.ERWB = ERW_WRITE_ERT_TO_MATCH;
	}
	else  /* KNOCK_FM1_MODE_TRIGGER */
	{
		/* Channel flag */
		channel.FLAG1 = KNOCK_FLAG1_MODE_TRIGGER;
		/* Time base selection */
		channel.TBSB = TBS_M1C1GE;  /* match on time and capture time */
		/* Schedule end of trigger pulse */
		erta = ertb + (trigger_period >> 1);
		channel.MRLB = MRL_CLEAR;
		channel.MRLA = MRL_CLEAR;
		channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
		/* Schedule next TRIGGER */
		ertb += trigger_period;
		channel.ERWB = ERW_WRITE_ERT_TO_MATCH;
	}
}

/**************************************************************************
* THREAD NAME: WINDOW_END
* DESCRIPTION: Gate Window end angle.
**************************************************************************/
_eTPU_thread KNOCK::WINDOW_END(_eTPU_matches_disabled)
{
	/* IRQ & DMA at window end */
	if(irq_dma_options & KNOCK_IRQ_AT_WINDOW_END)
	{
		channel.CIRC = CIRC_INT_FROM_SERVICED;
	}
	if(irq_dma_options & KNOCK_DMA_AT_WINDOW_END)
	{
		channel.CIRC = CIRC_DATA_FROM_SERVICED;
	}

	/* Channel flag */
	channel.FLAG0 = KNOCK_FLAG0_WINDOW_NOT_ACTIVE;
	/* Output pin action control */
	channel.OPACB = OPAC_MATCH_HIGH;
	if(channel.FM0 == KNOCK_FM0_ACTIVE_LOW)
	{
		channel.OPACB = OPAC_MATCH_LOW;
	}
	/* Schedule next window start */
	ScheduleStartAngle_NoReturn();
}

/**************************************************************************
* THREAD NAME: TRIGGER
* DESCRIPTION: Trigger pulse active edge.
**************************************************************************/
_eTPU_thread KNOCK::TRIGGER(_eTPU_matches_disabled)
{
	/* IRQ & DMA at every trigger */
	if(irq_dma_options & KNOCK_IRQ_AT_EVERY_TRIGGER)
	{
		channel.CIRC = CIRC_INT_FROM_SERVICED;
	}
	if(irq_dma_options & KNOCK_DMA_AT_EVERY_TRIGGER)
	{
		channel.CIRC = CIRC_DATA_FROM_SERVICED;
	}
	
	/* Schedule end of this trigger pulse */
	erta = ertb + (trigger_period >> 1);
	channel.MRLB = MRL_CLEAR;
	channel.MRLA = MRL_CLEAR;
	channel.ERWA = ERW_WRITE_ERT_TO_MATCH;

	/* End of window? */
	if(tcr2 >= tcr2_window_end)
	{
		/* IRQ & DMA at window end */
		if(irq_dma_options & KNOCK_IRQ_AT_WINDOW_END)
		{
			channel.CIRC = CIRC_INT_FROM_SERVICED;
		}
		if(irq_dma_options & KNOCK_DMA_AT_WINDOW_END)
		{
			channel.CIRC = CIRC_DATA_FROM_SERVICED;
		}

		/* Channel flag */
		channel.FLAG0 = KNOCK_FLAG0_WINDOW_NOT_ACTIVE;
		/* Time base selection */
		channel.TBSB = TBS_M2C1GE;  /* match on angle and capture time */
		/* Schedule next WINDOW_START */
		ScheduleStartAngle_NoReturn();
	}
	else
	{
		/* Schedule next TRIGGER */
		ertb += trigger_period;
		channel.ERWB = ERW_WRITE_ERT_TO_MATCH;
	}
}


DEFINE_ENTRY_TABLE(KNOCK, KNOCK, alternate, outputpin, autocfsr)
{
	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR2(6,7,   x,  x, x, x,  x, x, INIT),
	ETPU_VECTOR1(0,     1,  0, 0, 0,  x, x, INIT),
	ETPU_VECTOR1(0,     1,  0, 0, 1,  x, x, INIT),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR3(1,4,5, x,  x, x, x,  x, x, STOP),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  0, 1, 0,  0, 0, WINDOW_START),
	ETPU_VECTOR1(0,     x,  0, 1, 0,  0, 1, WINDOW_START),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  0, 0, WINDOW_START),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  0, 1, WINDOW_START),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  0, 0, WINDOW_START),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  0, 1, WINDOW_START),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  0, 0, WINDOW_START),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  0, 1, WINDOW_START),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  0, 1, 0,  1, 0, WINDOW_END),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  1, 0, WINDOW_END),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  1, 0, WINDOW_END),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  1, 0, WINDOW_END),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  0, 1, 0,  1, 1, TRIGGER),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  1, 1, TRIGGER),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  1, 1, TRIGGER),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  1, 1, TRIGGER),

    // unused/invalid entries
	ETPU_VECTOR2(2,3,   x,  x, x, 0,  0, x, _Error_handler_unexpected_thread),
	ETPU_VECTOR2(2,3,   x,  x, x, 0,  1, x, _Error_handler_unexpected_thread),
	ETPU_VECTOR2(2,3,   x,  x, x, 1,  0, x, _Error_handler_unexpected_thread),
	ETPU_VECTOR2(2,3,   x,  x, x, 1,  1, x, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  1, 0, 0,  0, 0, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  1, 0, 0,  1, 0, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  1, 0, 0,  0, 1, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  1, 0, 0,  1, 1, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  0, 0, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  1, 0, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  0, 1, _Error_handler_unexpected_thread),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  1, 1, _Error_handler_unexpected_thread),
};


/*******************************************************************************
*  Export interface information to Host CPU program. 
*******************************************************************************/
/* [MISRA 2004 Rule 3.4] usage of #pragma write documented in the Pragma Write 
   Manual, see https://www.ashware.com/Manuals */
#pragma write h, (::ETPUfilename (cpu/etpu_knock_auto.h));
#pragma write h, (/**************************************************************** );
#pragma write h, (* WARNING: This file is automatically generated. DO NOT EDIT IT! );
#pragma write h, (*);
#pragma write h, (* COPYRIGHT (c) Freescale 2004-2014, All Rights Reserved );
#pragma write h, (*);
#pragma write h, (* FILE NAME: etpu_knock_auto.h  );
#if defined(__TARGET_ETPU1__) || defined(__ETPU__)
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
#pragma write h, (#ifndef _ETPU_KNOCK_AUTO_H_ );
#pragma write h, (#define _ETPU_KNOCK_AUTO_H_ );
#pragma write h, ( );
#pragma write h, (/* Function Configuration Information */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_FUNCTION_NUMBER) ::ETPUfunctionnumber(KNOCK) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_TABLE_SELECT) ::ETPUentrytype(KNOCK) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_NUM_PARMS) ::ETPUram(KNOCK) );
#pragma write h, ( );
#pragma write h, (/* Host Service Request Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_HSR_INIT)         KNOCK_HSR_INIT );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_HSR_STOP)         KNOCK_HSR_STOP );
#pragma write h, ( );
#pragma write h, (/* Function Mode Bit Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_FM0_ACTIVE_HIGH)  KNOCK_FM0_ACTIVE_HIGH );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_FM0_ACTIVE_LOW)   KNOCK_FM0_ACTIVE_LOW );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_FM1_MODE_GATE)    (KNOCK_FM1_MODE_GATE<<1) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_FM1_MODE_TRIGGER) (KNOCK_FM1_MODE_TRIGGER<<1) );
#pragma write h, ( );
#pragma write h, (/* Parameter Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_OFFSET_P_WINDOW_FIRST)    ::ETPUlocation (KNOCK, p_window_first) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_OFFSET_P_WINDOW)          ::ETPUlocation (KNOCK, p_window) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_OFFSET_TDC_ANGLE)         ::ETPUlocation (KNOCK, tdc_angle) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_OFFSET_TDC_ANGLE_ACTUAL)  ::ETPUlocation (KNOCK, tdc_angle_actual) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_OFFSET_TCR2_WINDOW_START) ::ETPUlocation (KNOCK, tcr2_window_start) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_OFFSET_TCR2_WINDOW_END)   ::ETPUlocation (KNOCK, tcr2_window_end) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_OFFSET_TRIGGER_PERIOD)    ::ETPUlocation (KNOCK, trigger_period) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_OFFSET_WINDOW_COUNT)      ::ETPUlocation (KNOCK, window_count) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_OFFSET_WINDOW_COUNTER)    ::ETPUlocation (KNOCK, window_counter) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_OFFSET_IRQ_DMA_OPTIONS)   ::ETPUlocation (KNOCK, irq_dma_options) );
#pragma write h, ( );
#pragma write h, (/* Window Structure Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_WINDOW_OFFSET_START)   0x01 );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_WINDOW_OFFSET_WIDTH)   0x05 );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_WINDOW_STRUCT_SIZE)    0x08 );
#pragma write h, ( );
#pragma write h, (/* IRQ & DMA Options */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_IRQ_AT_WINDOW_START)   KNOCK_IRQ_AT_WINDOW_START );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_IRQ_AT_WINDOW_END)     KNOCK_IRQ_AT_WINDOW_END );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_IRQ_AT_EVERY_TRIGGER)  KNOCK_IRQ_AT_EVERY_TRIGGER );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_DMA_AT_WINDOW_START)   KNOCK_DMA_AT_WINDOW_START );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_DMA_AT_WINDOW_END)     KNOCK_DMA_AT_WINDOW_END );
#pragma write h, (::ETPUliteral(#define FS_ETPU_KNOCK_DMA_AT_EVERY_TRIGGER)  KNOCK_DMA_AT_EVERY_TRIGGER );
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
*  Revision 0.1  2013/09/06  r54529
*  Initial version.
*
*******************************************************************************/
