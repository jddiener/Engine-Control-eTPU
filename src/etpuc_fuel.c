/*******************************************************************************
* This file has been modified by ASH WARE Inc. as allowed per the original 
* license (see bottom of file), to add features and fix issues.
*******************************************************************************/

/*******************************************************************************
*
*  FREESCALE SEMICONDUCTOR INC. 2008-2014
*  ALL RIGHTS RESERVED
*  COPYRIGHT (c)
*
********************************************************************************
*
*  FILE NAME:  etpuc_fuel.c
*
*  DESCRIPTION:
*    This eTPU function generates fuel injection pulses.
*
*******************************************************************************/

/*******************************************************************************
*  Includes
*******************************************************************************/
#include <etpu_std.h>
#include "etpuc_fuel.h"
#include "etpuc_crank.h"   /* global eng_cycle_tcr2_ticks */
#include "etpuc_set.h"

/*******************************************************************************
*  eTPU Function Parameters:
*  
*  error - error flags
*  tdc_angle - TCR2 angle relative to engine-cycle start
*  tdc_angle_actual - absolute TDC TCR2 angle
*  angle_normal_end - TDC-relative TCR2 normal end angle
*  angle_stop - TDC-relative TCR2 latest stop angle
*  angle_offset_recalc - TCR2 angle offset between the start angle to 
*    the recalculation point  
*  injection_time - requested TCR1 injection time
*  compensation_time - TCR1 time added to injection time to compensate the valve 
*    opening and closing time
*  off_time_minimum - minimum TCR1 time between 2 injection pulses 
*  injection_time_minimum - minimum TCR1 time of an injection pulse
*  injection_time_applied - applied TCR1 injection time
*  injection_time_applied_cpu - copy of injection_time_applied, updated 
*    every STOP_ANGLE 
*  injection_start_angle - calculated TCR2 start angle of the injection
*  injection_start_angle_cpu - TDC-relative TCR2 start angle of the last 
*    injection, updated every STOP_ANGLE
*  pulse_start_time - last pulse start TCR1 time
*  pulse_end_time - last pulse end TCR1 time 
*  generation_disable - disable/enable injection pulse generation. A value
*    change is applied from next recalculation angle, finishing the current
*    engine-cycle unaffected.
*
********************************************************************************
*
*  Channel Flag usage
*    Flag0 = FUEL_FLAG0_INJ_NOT_ACTIVE (0)  ... Injection is not active
*    Flag0 = FUEL_FLAG0_INJ_ACTIVE (1)      ... Injection is active
*    Flag1 = FUEL_FLAG1_RECALC_ANGLE (0)    ... RECALC_ANGLE is scheduled.
*    Flag1 = FUEL_FLAG1_STOP_ANGLE (1)      ... STOP_ANGLE is scheduled.
*
********************************************************************************
*
*  Channel Function Mode (FM) bits usage
*    FM0 is used to select the output polarity:
*      - FUEL_FM0_ACTIVE_HIGH
*      - FUEL_FM0_ACTIVE_LOW
*    FM1 is not used.
*
********************************************************************************
*
*  Channel Interrupt usage
*    The channel interrupt on the FUEL channel is set on each STOP_ANGLE. 
*
*******************************************************************************/

/*******************************************************************************
*  eTPU Class Methods/Fragments
*******************************************************************************/

/*******************************************************************************
*  FUNCTION NAME: OnRecalcAngle_NoReturn
*  DESCRIPTION: Recalculate start angle and schedule PULSE_START.
*               Schedule STOP_ANGLE. 
*******************************************************************************/
_eTPU_fragment FUEL::OnRecalcAngle_NoReturn(void)
{
	int24_t tmp;
	
	/* Schedule STOP_ANGLE */
	ertb = tdc_angle_actual - angle_stop;
	channel.MRLB = MRL_CLEAR;
	channel.ERWB = ERW_WRITE_ERT_TO_MATCH;
	/* Channel flags */
	channel.FLAG1 = FUEL_FLAG1_STOP_ANGLE;
	is_await_recalc = FALSE;

	/* Generate injection pulse only if injection time > minimum and generation is allowed */
	if((generation_disable == FUEL_GENERATION_ALLOWED) &&
	   (injection_time > injection_time_minimum))
	{
		/* Re-calculate start angle */
		tmp = injection_time + compensation_time;
		tmp = CRANK_Time_to_Angle_HighRes(tmp);
		injection_start_angle = tdc_angle_actual - angle_normal_end - tmp;
		erta = injection_start_angle;
		
		/* Output pin action control */
		if(cc.FM0 == FUEL_FM0_ACTIVE_HIGH)
		{
			channel.OPACA = OPAC_MATCH_HIGH;
		}
		else
		{
			channel.OPACA = OPAC_MATCH_LOW;
		}
		/* Schedule PULSE_START at start angle */
		channel.TBSA = TBS_M2C1GE;  /* match on angle */
		channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
	}
}

/*******************************************************************************
*  FUNCTION NAME: OnStopAngle_NoReturn
*  DESCRIPTION: Cancel any scheduled match (PULSE_START or PULSE_END).
*               Write applied injection time and start angle for CPU readings.
*               Update actual TDC angle.
*               Calculate next start angle, schedule next RECALC_ANGLE.
*               Set channel interrupt.
*******************************************************************************/
_eTPU_fragment FUEL::OnStopAngle_NoReturn(void)
{
	/* Disable match(A) recognition (cancel any PULSE_START or PULSE_END scheduled) */
	channel.MRLE = MRLE_DISABLE;

	/* Write applied injection time for CPU readings */
	injection_time_applied_cpu = injection_time_applied;
	/* Reset applied injection time */
	injection_time_applied = 0;

	/* Write injection start angle for CPU reading */
	injection_start_angle_cpu = tdc_angle_actual - injection_start_angle;

    /* record this stop angle */	
	angle_stop_actual_last = tdc_angle_actual - angle_stop;

	/* Update actual TDC angle for this cycle */
	tdc_angle_actual += eng_cycle_tcr2_ticks;
	
	/* Set channel interrupt */
	channel.CIRC = CIRC_INT_FROM_SERVICED;

    /* schedule the next start angle calculation */
    is_first_recalc = TRUE;
    angle_offset_recalc_working = angle_offset_recalc;
    ScheduleRecalc_NoReturn();
}

/*******************************************************************************
*  FUNCTION NAME: ScheduleRecalc_NoReturn
*  DESCRIPTION: Calculate next start angle, schedule next RECALC_ANGLE.
*******************************************************************************/
_eTPU_fragment FUEL::ScheduleRecalc_NoReturn(void)
{
	int24_t tmp;

	/* Calculate next start angle */
	tmp = injection_time + compensation_time;
	tmp = CRANK_Time_to_Angle_LowRes(tmp);
	injection_start_angle = tdc_angle_actual - angle_normal_end - tmp;
	/* ensure that minimum off time will be met */
	tmp = CRANK_Time_to_Angle_LowRes(off_time_minimum) + angle_stop_actual_last;
	if (tmp - injection_start_angle > 0)
	{
        /* need to enforce minimum off time, push recalc back */
        ertb = tmp;
	}
	else
	{
        /* expected case, next start angle is more than minimum time after stop angle */
        ertb = injection_start_angle - angle_offset_recalc_working;
	}
	
	/* Schedule next RECAL_ANGLE */
	channel.MRLB = MRL_CLEAR;
	channel.ERWB = ERW_WRITE_ERT_TO_MATCH;
	/* Channel flags */
	channel.FLAG1 = FUEL_FLAG1_RECALC_ANGLE;
	is_await_recalc = TRUE;
}

/*******************************************************************************
*  FUNCTION NAME: SchedulePulseEnd_NoReturn
*  DESCRIPTION: Record pulse start time.
*               Schedule PULSE_END, check minimum injection time.
*******************************************************************************/
_eTPU_fragment FUEL::SchedulePulseEnd_NoReturn(void)
{
	int24_t tmp;
	
	/* Record pulse start time */
	pulse_start_time = erta;

	/* Output pin action control */
	if(cc.FM0 == FUEL_FM0_ACTIVE_HIGH)
	{
		channel.OPACA = OPAC_MATCH_LOW;
	}
	else
	{
		channel.OPACA = OPAC_MATCH_HIGH;
	}
	/* Prevent the injection time to be shorter then the minimum injection time */
	tmp = injection_time - injection_time_applied;
	if(tmp < injection_time_minimum)
	{
		tmp = injection_time_minimum;
		/* set error flag */
		error |= FUEL_ERROR_MINIMUM_INJ_TIME_APPLIED;
	}
	/* Schedule PULSE_END */
	erta = pulse_start_time + tmp;
	erta += compensation_time;
	channel.TBSA = TBS_M1C1GE;  /* match on time */
	channel.MRLA = MRL_CLEAR;
	channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
	/* Channel flags */
	channel.FLAG0 = FUEL_FLAG0_INJ_ACTIVE;
}

/*******************************************************************************
*  FUNCTION NAME: ScheduleAdditionalPulse_NoReturn
*  DESCRIPTION: Schedule additional injection pulse with respect to
*               - minimum injection time and
*               - minimum off time 
*******************************************************************************/
_eTPU_fragment FUEL::ScheduleAdditionalPulse_NoReturn(void)
{
	/* Additional pulse needed? */
	if(injection_time - injection_time_applied > injection_time_minimum)
	{
		/* start the additional pulse at least minimum off time after the previous pulse */
		erta = pulse_end_time + off_time_minimum;
		/* Output pin action control */
		if(cc.FM0 == FUEL_FM0_ACTIVE_HIGH)
		{
			channel.OPACA = OPAC_MATCH_HIGH;
		}
		else
		{
			channel.OPACA = OPAC_MATCH_LOW;
		}
		/* Schedule PULSE_START */
		channel.TBSA = TBS_M1C1GE;  /* match on time */
		channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
	}
}

/*******************************************************************************
*  FUNCTION NAME: OnPulseEnd
*  DESCRIPTION: Record pulse end time.
*               Update applied injection time.
*******************************************************************************/
void FUEL::OnPulseEnd(void)
{
	/* Record pulse end time */
	pulse_end_time = erta;
	/* Add pulse width to applied injection time, remove compensation time */
	injection_time_applied += erta - pulse_start_time;
	injection_time_applied -= compensation_time;
	/* Clear latch */
	channel.MRLA = MRL_CLEAR;
	/* Channel flags */
	channel.FLAG0 = FUEL_FLAG0_INJ_NOT_ACTIVE;
}



/*******************************************************************************
*  eTPU Function
*******************************************************************************/

/**************************************************************************
* THREAD NAME: INIT
* DESCRIPTION: Initialize the channel to run the FUEL function.
**************************************************************************/
_eTPU_thread FUEL::INIT(_eTPU_matches_disabled)
{
	int24_t tmp;

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
	channel.TBSB = TBS_M2C1GE;
	/* Input pin action control */
	channel.IPACA = IPAC_NO_DETECT;
	channel.IPACB = IPAC_NO_DETECT;
	/* Output pin action control */
	if(cc.FM0 == FUEL_FM0_ACTIVE_HIGH)
	{
		channel.PIN = PIN_SET_LOW;
		channel.OPACB = OPAC_MATCH_LOW;
	}
	else
	{
		channel.PIN = PIN_SET_HIGH;
		channel.OPACB = OPAC_MATCH_HIGH;
	}
	/* Enable output pin buffer */
	channel.TBSA = TBSA_SET_OBE;
	/* Channel flags */
	channel.FLAG0 = FUEL_FLAG0_INJ_NOT_ACTIVE;
	channel.FLAG1 = FUEL_FLAG1_RECALC_ANGLE;
	is_await_recalc = TRUE;
	angle_offset_recalc_working = angle_offset_recalc;

    if (eng_pos_state != ENG_POS_FULL_SYNC)
    {
        /* only fully start the fuel processing once sync is achieved */
        return;
    }
	
	/* Initialize actual TDC angle */
	tdc_angle_actual = eng_cycle_tcr2_start + tdc_angle;
	angle_stop_actual_last = tdc_angle_actual - angle_stop;
	
	/* Calculate the first start angle */
	tmp = injection_time + compensation_time;
	tmp = CRANK_Time_to_Angle_LowRes(tmp);
	injection_start_angle = tdc_angle_actual - angle_normal_end - tmp;
	
	/* Schedule the first RECAL_ANGLE - double angle_offset_recalc */
	ertb = injection_start_angle - 2*angle_offset_recalc;
	channel.ERWB = ERW_WRITE_ERT_TO_MATCH;

	/* Enable event handling */
	channel.MTD = MTD_ENABLE;
}

/**************************************************************************
* THREAD NAME: STOP
* DESCRIPTION: Stop the running injection.
*              The next injection will start normally. 
**************************************************************************/
_eTPU_thread FUEL::STOP(_eTPU_matches_disabled)
{
	/* Stop - schedule an immediate match A to set pin inactive 
	   and rewrite any match already scheduled */
	/* Output pin action control */
	if(cc.FM0 == FUEL_FM0_ACTIVE_HIGH)
	{
		channel.OPACA = OPAC_MATCH_LOW;
	}
	else
	{
		channel.OPACA = OPAC_MATCH_HIGH;
	}
	/* Schedule match now */
	erta = tcr1;
	channel.MRLA = MRL_CLEAR;
	channel.TBSA = TBS_M1C1GE;  /* match on time */
	channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
	/* Channel flag */
	channel.FLAG0 = FUEL_FLAG0_INJ_NOT_ACTIVE;

	/* Clear the immediate match */
	channel.MRLA = MRL_CLEAR;
}

/**************************************************************************
* THREAD NAME: UPDATE_0
* DESCRIPTION: Update the injection time when the injection is NOT active.
**************************************************************************/
_eTPU_thread FUEL::UPDATE_INACTIVE(_eTPU_matches_disabled)
{
	/* Theoretically, the HSR_UPDATE can be serviced between PULSE_START edge 
	   and PULSE_START service - check match A latch. */
	if(cc.MRLA)
	{
		SchedulePulseEnd_NoReturn();
	}
	else
	{
	    if (eng_pos_state != ENG_POS_FULL_SYNC)
	    {
	        /* ignore updates until full sync achieved */
	        return;
	    }
	    else
	    {
			/* Does the HSR_UPDATE come before the main injection */
			if(injection_time_applied == 0)
			{
			    if (is_await_recalc)
			    {
			        /* pulse start not scheduled yet, reschedule the recalc match */
			        ScheduleRecalc_NoReturn();
			    }
			    else
			    {
			        /* pulse start has been scheduled, recalculate */
    				OnRecalcAngle_NoReturn();
    			}
			}
			else
			{
				ScheduleAdditionalPulse_NoReturn();
			}
		}
	}
}

/**************************************************************************
* THREAD NAME: UPDATE_1
* DESCRIPTION: Update the injection time when the injection is active.
**************************************************************************/
_eTPU_thread FUEL::UPDATE_ACTIVE(_eTPU_matches_disabled)
{
	/* Theoretically, the HSR_UPDATE can be serviced between PULSE_END edge 
	   and PULSE_END service - check match A latch. */
	if(cc.MRLA)
	{
		/* service PULSE_END first */
		OnPulseEnd();
		
		/* run UPDATE_0 */
		ScheduleAdditionalPulse_NoReturn();
	}
	else
	{
		erta = pulse_start_time;
		SchedulePulseEnd_NoReturn();
	}
}

/**************************************************************************
* THREAD NAME: PULSE_START
* DESCRIPTION: Start of main or additional injection pulse.
**************************************************************************/
_eTPU_thread FUEL::PULSE_START(_eTPU_matches_disabled)
{
	SchedulePulseEnd_NoReturn();
}

/**************************************************************************
* THREAD NAME: PULSE_END
* DESCRIPTION: End of main or additional injection pulse.
**************************************************************************/
_eTPU_thread FUEL::PULSE_END(_eTPU_matches_disabled)
{
	OnPulseEnd();
}

/**************************************************************************
* THREAD NAME: STOP_ANGLE_0
* DESCRIPTION: Stop angle when the injection is NOT active.
**************************************************************************/
_eTPU_thread FUEL::STOP_ANGLE_INACTIVE(_eTPU_matches_disabled)
{
	OnStopAngle_NoReturn();
}

/**************************************************************************
* THREAD NAME: STOP_ANGLE_1
* DESCRIPTION: Stop angle when the injection is active.
**************************************************************************/
_eTPU_thread FUEL::STOP_ANGLE_ACTIVE(_eTPU_matches_disabled)
{
	/* set error flag */
	error |= FUEL_ERROR_STOP_ANGLE_APPLIED;

	/* service PULSE_END first */
	erta = ertb; /* put pulse end time into erta where it is expected */
	OnPulseEnd();
	
	/* Process normal STOP_ANGLE */
	OnStopAngle_NoReturn();
}
/**************************************************************************
* THREAD NAME: RECALC_ANGLE
* DESCRIPTION: Recalculation angle.
**************************************************************************/
_eTPU_thread FUEL::RECALC_ANGLE(_eTPU_matches_disabled)
{
    if (is_first_recalc)
    {
        is_first_recalc = FALSE;
        angle_offset_recalc_working >>= 2;
        ScheduleRecalc_NoReturn();
    }
    
	OnRecalcAngle_NoReturn();
}


DEFINE_ENTRY_TABLE(FUEL, FUEL, alternate, outputpin, autocfsr)
{
	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR2(6,7,   x,  x, x, x,  x, x, INIT),
	ETPU_VECTOR1(0,     1,  0, 0, 0,  x, x, INIT),
	ETPU_VECTOR1(0,     1,  0, 0, 1,  x, x, INIT),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR3(1,4,5, x,  x, x, x,  x, x, STOP),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR2(2,3,   x,  x, x, 0,  0, x, UPDATE_INACTIVE),
	ETPU_VECTOR2(2,3,   x,  x, x, 1,  0, x, UPDATE_INACTIVE),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR2(2,3,   x,  x, x, 0,  1, x, UPDATE_ACTIVE),
	ETPU_VECTOR2(2,3,   x,  x, x, 1,  1, x, UPDATE_ACTIVE),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  1, 0, 0,  0, 0, PULSE_START),
	ETPU_VECTOR1(0,     x,  1, 0, 0,  0, 1, PULSE_START),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  0, 0, PULSE_START),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  0, 1, PULSE_START),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  0, 0, PULSE_START),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  0, 1, PULSE_START),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  0, 0, PULSE_START),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  0, 1, PULSE_START),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  1, 0, 0,  1, 0, PULSE_END),
	ETPU_VECTOR1(0,     x,  1, 0, 0,  1, 1, PULSE_END),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  1, 0, PULSE_END),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  1, 1, PULSE_END),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  1, 0, PULSE_END),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  1, 1, PULSE_END),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  1, 0, PULSE_END),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  1, 1, PULSE_END),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  0, 1, 0,  0, 1, STOP_ANGLE_INACTIVE),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  0, 1, STOP_ANGLE_INACTIVE),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  0, 1, 0,  1, 1, STOP_ANGLE_ACTIVE),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  1, 1, STOP_ANGLE_ACTIVE),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  0, 1, 0,  0, 0, RECALC_ANGLE),
	ETPU_VECTOR1(0,     x,  0, 1, 0,  1, 0, RECALC_ANGLE),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  0, 0, RECALC_ANGLE),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  1, 0, RECALC_ANGLE),

    // unused/invalid entries
};


/*******************************************************************************
*  Export interface information to Host CPU program.
*******************************************************************************/
/* [MISRA 2004 Rule 3.4] usage of #pragma write documented in the Pragma Write 
   Manual, see https://www.ashware.com/Manuals */
#pragma write h, (::ETPUfilename (cpu/etpu_fuel_auto.h));
#pragma write h, (/**************************************************************** );
#pragma write h, (* WARNING: This file is automatically generated. DO NOT EDIT IT! );
#pragma write h, (*);
#pragma write h, (* FILE NAME: etpu_fuel.h  );
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
#pragma write h, (#ifndef _ETPU_FUEL_AUTO_H_ );
#pragma write h, (#define _ETPU_FUEL_AUTO_H_ );
#pragma write h, ( );
#pragma write h, (/* Function Configuration Information */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_FUNCTION_NUMBER) ::ETPUfunctionnumber(FUEL) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_TABLE_SELECT) ::ETPUentrytype(FUEL) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_NUM_PARMS) ::ETPUram(FUEL) );
#pragma write h, ( );
#pragma write h, (/* Host Service Request Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_HSR_INIT)        FUEL_HSR_INIT );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_HSR_STOP)        FUEL_HSR_STOP );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_HSR_UPDATE)      FUEL_HSR_UPDATE );
#pragma write h, ( );
#pragma write h, (/* Function Mode Bit Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_FM0_ACTIVE_LOW)  FUEL_FM0_ACTIVE_LOW );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_FM0_ACTIVE_HIGH) FUEL_FM0_ACTIVE_HIGH );
#pragma write h, ( );
#pragma write h, (/* Parameter Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_OFFSET_ERROR                     ) ::ETPUlocation (FUEL, error ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_OFFSET_TDC_ANGLE                 ) ::ETPUlocation (FUEL, tdc_angle ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_OFFSET_TDC_ANGLE_ACTUAL          ) ::ETPUlocation (FUEL, tdc_angle_actual ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_OFFSET_ANGLE_NORMAL_END          ) ::ETPUlocation (FUEL, angle_normal_end ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_OFFSET_ANGLE_STOP                ) ::ETPUlocation (FUEL, angle_stop ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_OFFSET_ANGLE_OFFSET_RECALC       ) ::ETPUlocation (FUEL, angle_offset_recalc ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_OFFSET_INJECTION_TIME            ) ::ETPUlocation (FUEL, injection_time ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_OFFSET_COMPENSATION_TIME         ) ::ETPUlocation (FUEL, compensation_time ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_OFFSET_INJECTION_TIME_MINIMUM    ) ::ETPUlocation (FUEL, injection_time_minimum ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_OFFSET_OFF_TIME_MINIMUM          ) ::ETPUlocation (FUEL, off_time_minimum ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_OFFSET_INJECTION_TIME_APPLIED    ) ::ETPUlocation (FUEL, injection_time_applied ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_OFFSET_INJECTION_TIME_APPLIED_CPU) ::ETPUlocation (FUEL, injection_time_applied_cpu ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_OFFSET_INJECTION_START_ANGLE     ) ::ETPUlocation (FUEL, injection_start_angle ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_OFFSET_INJECTION_START_ANGLE_CPU ) ::ETPUlocation (FUEL, injection_start_angle_cpu ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_OFFSET_PULSE_START_TIME          ) ::ETPUlocation (FUEL, pulse_start_time ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_OFFSET_PULSE_END_TIME            ) ::ETPUlocation (FUEL, pulse_end_time ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_OFFSET_GENERATION_DISABLE        ) ::ETPUlocation (FUEL, generation_disable ) );
#pragma write h, ( );
#pragma write h, (/* Error Flags Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_ERROR_STOP_ANGLE_APPLIED)       FUEL_ERROR_STOP_ANGLE_APPLIED);
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_ERROR_MINIMUM_INJ_TIME_APPLIED) FUEL_ERROR_MINIMUM_INJ_TIME_APPLIED);
#pragma write h, ( );
#pragma write h, (/* Generation Disable Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_GENERATION_ALLOWED)             FUEL_GENERATION_ALLOWED);
#pragma write h, (::ETPUliteral(#define FS_ETPU_FUEL_GENERATION_DISABLED)            FUEL_GENERATION_DISABLED);
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
*  Revision 0.2  2013/09/05  r54529
*  Generation disable added + minor updates.
*
*  Revision 0.1  2013/08/27  r54529
*  Initial version.
*
*******************************************************************************/
