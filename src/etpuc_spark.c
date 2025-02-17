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
*  FILE NAME:  etpuc_spark.c
*
*  DESCRIPTION:
*    This eTPU function is intended to generate ignition spark pulses 
*    in an engine control system.
*    
*    A single spark consists of one or more pulses. 
*    The first pulse is the main spark pulse, which is defined by dwell_time
*    and end_angle. The main pulse start angle is estimated based on the current
*    engine speed (using TRR register) and recalculated one again at the 
*    angle_offset_recalc before the start angle. Whatever the acceleration or 
*    deceleration is, the spark main pulse end_angle is kept and 
*    dwell_time_applied varies, until the dwell_time_min or dwell_time_max must
*    limitation is applied. In that case the end_angle is not kept and an
*    error flag is set to report this.
*    Multi spark pulses can follow the main spark pulse. They are defined by
*    multi_on_time, multi_off_time and multi_pulse_count.
*    
*    The number of single sparks per engine cycle is configurable. The single
*    sparks are defined by an array of single spark structures, consisting of
*    end angle, dwell time and count of multi pulses.
*    
*    There is an SPARK parameter tdc_angle, relative to which all end angles are 
*    defined. Positive angles precede the tdc_angle, negative angles come after.
*    
*    The SPARK function generates channel interrupts at each Recalculation Angle
*    thread - angle_offset_recalc before the estimated start angle.
*
*******************************************************************************/

/*******************************************************************************
*  Includes
*******************************************************************************/
#include <etpu_std.h>
#include "etpuc_spark.h"
#include "etpuc_crank.h"   /* global eng_cycle_tcr2_ticks */
#include "etpuc_set.h"

/*******************************************************************************
*  eTPU Function Parameters:
*  
*  SPARK Channel Parameters (struct SPARK_CHAN_PARAMS)
*  ---------------------------------------------------
*  tdc_angle - TCR2 angle relative to engine-cycle start
*  tdc_angle_actual - absolute TDC TCR2 angle
*  angle_offset_recalc - TCR2 angle offset between the start angle to 
*    the recalculation point
*  dwell_time_min - TCR1 minimum spark dwell time  
*  dwell_time_max - TCR1 maximum spark dwell time
*  multi_on_time - TCR time of multi spark pulse active state  
*  multi_off_time - TCR time of multi spark pulse inactive state
*  *p_single_spark_first - pointer to the first single spark structure  
*  spark_count - count of single sparks per engine cycle      
*  *p_single_spark - pointer to the current single spark structure
*  spark_counter - counts single sparks
*  pulse_start_time - captured TCR1 time of last spark main pulse start
*  dwell_time_applied - applied TCR1 dwell time. It can slightly differ
*    from dwell_time due to acceleration or deceleration. At corner cases,
*    it is limited by dwell_time_min and dwell_time_max.
*  dwell_time - copy of current TCR1 dwell time from single spark structure      
*  end_angle - copy of current TCR2 end angle from single spark structure
*  multi_pulse_count - copy of current count of multi pulses from single 
*    spark structure     
*  multi_pulse_counter - counts multi pulses      
*  state - status, which angle/time event is scheduled
*  error - error flags
*  generation_disable - disable/enable injection pulse generation. A value
*    change is applied from next recalculation angle, finishing the current
*    engine-cycle unaffected.
*    
*  Single Spark Structure Parameters (struct SINGLE_SPARK)
*  -------------------------------------------------------
*  multi_pulse_count - count of multi pulses after the main spark pulse
*  end_angle - TCR2 angle of the spark main pulse end
*  dwell_time - TCR1 time of the spark dwell (spark main pulse width)
*
********************************************************************************
*
*  Channel Flag usage
*    Flag0 = SPARK_FLAG0_MAIN_PULSE (0)        ... Main pulse is active
*    Flag0 = SPARK_FLAG0_OUT_OF_MAIN_PULSE (1) ... Main Pulse is not active
*    Flag1 = SPARK_FLAG1_PRE_MIN_DWELL (0)     ... Prior to MIN_DWELL.
*    Flag1 = SPARK_FLAG1_POST_MIN_DWELL (1)    ... Post to MIN_DWELL.
*
********************************************************************************
*
*  Channel Function Mode (FM) bits usage
*    FM0 is used to select the output polarity:
*      - SPARK_FM0_ACTIVE_HIGH
*      - SPARK_FM0_ACTIVE_LOW
*    FM1 is not used.
*
********************************************************************************
*
*  Channel Interrupt usage
*    The channel interrupt on the SPARK channel is set on each RECALC_ANGLE. 
*
*******************************************************************************/

/*******************************************************************************
*  eTPU Class Methods/Fragments
*******************************************************************************/

/*******************************************************************************
*  FUNCTION NAME: Init_NoReturn
*  DESCRIPTION: Perform initialization actions for either startup, sync or stall
*******************************************************************************/
_eTPU_fragment SPARK::Init_NoReturn(void)
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
	/* Input pin action control */
	channel.IPACA = IPAC_NO_DETECT;
	channel.IPACB = IPAC_NO_DETECT;
	/* Enable output pin buffer */
	channel.TBSA = TBSA_SET_OBE;

	/* Set the pin to the inactive state */
	if(cc.FM0 == SPARK_FM0_ACTIVE_HIGH)
	{
		channel.PIN = PIN_SET_LOW;
	}
	else
	{
		channel.PIN = PIN_SET_HIGH;
	}				

	/* Initialize spark counter so that it is reset in ScheduleNextRecalAngle_NoReturn() */
	spark_counter = spark_count;
	/* Initialize actual TDC angle (eng_cycle_tcr2_ticks is added in ScheduleNextRecalAngle_NoReturn()) */
	tdc_angle_actual = eng_cycle_tcr2_start + tdc_angle - eng_cycle_tcr2_ticks;
	
	/* Enable event handling */
	channel.MTD = MTD_ENABLE;

    if (eng_pos_state != ENG_POS_FULL_SYNC)
    {
        /* only fully start the spark processing once sync is achieved */
        return;
    }

	/* Schedule the first RECALC_ANGLE */
	ScheduleNextRecalcAngle_NoReturn();
}

/*******************************************************************************
*  FUNCTION NAME: ScheduleNextRecalcAngle_NoReturn
*  DESCRIPTION: Schedule next RECALC_ANGLE
*******************************************************************************/
_eTPU_fragment SPARK::ScheduleNextRecalcAngle_NoReturn(void)
{
	/* Increment spark pointer and counter */
	p_single_spark++;
	spark_counter++;
	/* Engine-cycle finished? */
	if(spark_counter >= spark_count)
	{
		/* Reset spark pointer and counter */
		p_single_spark = p_single_spark_first;
		spark_counter = 0;
		/* Update actual TDC angle for next cycle */
		tdc_angle_actual += eng_cycle_tcr2_ticks;
	}

	/* Read next spark parameters */
	ReadSparkParams();

	/* Spark state */
	state = SPARK_STATE_RECALC;
	
	/* Channel flags */
	channel.FLAG0 = SPARK_FLAG0_OUT_OF_MAIN_PULSE;
	channel.FLAG1 = SPARK_FLAG1_POST_MIN_DWELL;
	
	/* schedule the recalc */
	is_first_recalc = TRUE;
	angle_offset_recalc_working = angle_offset_recalc;
	ScheduleRecalcAngle_NoReturn();
}

/*******************************************************************************
*  FUNCTION NAME: ScheduleRecalcAngle_NoReturn
*  DESCRIPTION: Schedule RECALC_ANGLE with current settings
*******************************************************************************/
_eTPU_fragment SPARK::ScheduleRecalcAngle_NoReturn(void)
{
	int24_t tmp;

	/* Calculate start angle */
	tmp = dwell_time;
	tmp = CRANK_Time_to_Angle_LowRes(tmp);
	ertb = tdc_angle_actual - end_angle - tmp;
	ertb = ertb - angle_offset_recalc_working;

	/* Configure action unit */
	channel.PDCM = PDCM_EM_NB_ST; /* either match non-blocking single transition */
	channel.TBSB = TBS_M2C1GE;
	channel.OPACB = OPAC_NO_CHANGE;
	/* Schedule next RECAL_ANGLE */
	channel.MRLA = MRL_CLEAR;
	channel.MRLB = MRL_CLEAR;
	channel.ERWB = ERW_WRITE_ERT_TO_MATCH;
}

/*******************************************************************************
*  FUNCTION NAME: ScheduleStartAngle_NoReturn
*  DESCRIPTION: Recalculate and schedule START_ANGLE.
*******************************************************************************/
_eTPU_fragment SPARK::ScheduleStartAngle_NoReturn(void)
{
	int24_t tmp;

	/* Re-Calculate start angle */
	tmp = dwell_time;
	tmp = CRANK_Time_to_Angle_HighRes(tmp);
	ertb = tdc_angle_actual - end_angle - tmp;

	/* Configure action unit */
	channel.TBSB = TBS_M2C1GE;
	if(cc.FM0 == SPARK_FM0_ACTIVE_HIGH)
	{
		channel.OPACB = OPAC_MATCH_HIGH;
	}
	else
	{
		channel.OPACB = OPAC_MATCH_LOW;
	}
	/* Schedule START_ANGLE */
	channel.MRLB = MRL_CLEAR;
	channel.ERWB = ERW_WRITE_ERT_TO_MATCH;
	
	/* Spark state */
	state = SPARK_STATE_START;

	/* Channel flags */
	channel.FLAG0 = SPARK_FLAG0_OUT_OF_MAIN_PULSE;
	channel.FLAG1 = SPARK_FLAG1_PRE_MIN_DWELL;
}

/*******************************************************************************
*  FUNCTION NAME: ScheduleMinDwellTime_NoReturn
*  DESCRIPTION: Schedule MIN_DWELL_TIME to match A.
*******************************************************************************/
_eTPU_fragment SPARK::ScheduleMinDwellTime_NoReturn(void)
{
	/* Configure action units */
	channel.PDCM = PDCM_SM_ST;
	channel.TBSA = TBS_M1C1GE;
	channel.TBSB = TBS_M2C2GE; /* capture TCR2 on action unit B in order to check for min dwell conditions */
	channel.OPACA = OPAC_NO_CHANGE;
	if(cc.FM0 == SPARK_FM0_ACTIVE_HIGH)
	{
		channel.OPACB = OPAC_MATCH_LOW;
	}
	else
	{
		channel.OPACB = OPAC_MATCH_HIGH;
	}
	/* Schedule MIN_DWELL */
	erta = pulse_start_time + dwell_time_min;
	channel.MRLA = MRL_CLEAR;
	channel.MRLB = MRL_CLEAR;
	channel.ERWA = ERW_WRITE_ERT_TO_MATCH;

	/* Spark state */
	state = SPARK_STATE_MIN_DWELL;

	/* Channel flags */
	channel.FLAG0 = SPARK_FLAG0_MAIN_PULSE;
	channel.FLAG1 = SPARK_FLAG1_PRE_MIN_DWELL;
}

/*******************************************************************************
*  FUNCTION NAME: ScheduleEndAngleAndMaxDwellTime_NoReturn
*  DESCRIPTION: Schedule MAX_DWELL_TIME to match A and END_ANGLE on B.
*******************************************************************************/
_eTPU_fragment SPARK::ScheduleEndAngleAndMaxDwellTime_NoReturn(void)
{
	/* Configure action unit */
	channel.PDCM = PDCM_EM_B_ST; /* either match blocking single transition */
	channel.TBSA = TBS_M1C1GE;   /* match on TCR1, capture TCR1, greater-equal */
	channel.TBSB = TBS_M2C1GE;
	if(cc.FM0 == SPARK_FM0_ACTIVE_HIGH)
	{
		channel.OPACA = OPAC_MATCH_LOW;
	}
	else
	{
		channel.OPACA = OPAC_MATCH_HIGH;
	}
	/* Schedule MAX_DWELL */
	erta = pulse_start_time + dwell_time_max;
	channel.MRLA = MRL_CLEAR;
	channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
	/* Schedule END_ANGLE */
	ertb = tdc_angle_actual - end_angle;
	channel.MRLB = MRL_CLEAR;
	channel.ERWB = ERW_WRITE_ERT_TO_MATCH;

	/* Spark state */
	state = SPARK_STATE_MAX_DWELL;

	/* Channel flags */
	channel.FLAG0 = SPARK_FLAG0_MAIN_PULSE;
	channel.FLAG1 = SPARK_FLAG1_POST_MIN_DWELL;
}

/*******************************************************************************
*  FUNCTION NAME: ScheduleMultiPulse_NoReturn
*  DESCRIPTION: Schedule MULTI_PULSE.
*               Note: End time of last pulse (either the main pulse or the 
*               previous multi pulse) must be in erta register.
*******************************************************************************/
_eTPU_fragment SPARK::ScheduleMultiPulse_NoReturn(void)
{
	/* Configure action unit */
	channel.PDCM = PDCM_BM_ST; /* both match single transition */
	channel.TBSA = TBS_M1C1GE;
	channel.TBSB = TBS_M1C1GE;
	if(cc.FM0 == SPARK_FM0_ACTIVE_HIGH)
	{
		channel.OPACA = OPAC_MATCH_LOW;
		channel.OPACB = OPAC_MATCH_HIGH;
	}
	else
	{
		channel.OPACA = OPAC_MATCH_HIGH;
		channel.OPACB = OPAC_MATCH_LOW;
	}
	/* Schedule MULTI_PULSE */
	ertb = erta + multi_off_time;
	erta = ertb + multi_on_time;
	channel.MRLB = MRL_CLEAR;
	channel.MRLA = MRL_CLEAR;
	channel.ERWB = ERW_WRITE_ERT_TO_MATCH;
	channel.ERWA = ERW_WRITE_ERT_TO_MATCH;

	/* Spark state */
	state = SPARK_STATE_MULTI_PULSE;

	/* Channel flags */
	channel.FLAG0 = SPARK_FLAG0_OUT_OF_MAIN_PULSE;
	channel.FLAG1 = SPARK_FLAG1_POST_MIN_DWELL;
}

/*******************************************************************************
*  FUNCTION NAME: ReadSparkParams
*  DESCRIPTION: Coherently read single spark parameters to internal variables.
*******************************************************************************/
void SPARK::ReadSparkParams(void)
{
	/* semi-assembly code for coherent read from spark array */
#ifdef __ETEC__
	register_p31_24 p31_24;
    register_diob diob = (int24_t)p_single_spark;
    NOP();
	/* [MISRA 2004 Rule 2.1] Assembly language shall be encapsulated and isolated */
#asm
    ram p31_0 <- by diob++.
    ram diob <- by diob; alu erta = p.
    alu ertb = diob.
#endasm
#else
	register_p31_24 int8_t p31_24;
	register_diob   int24_t diob;

	diob = (int24_t)p_single_spark;
	/* [MISRA 2004 Rule 2.1] Assembly language shall be encapsulated and isolated */
	asm{
		nop
		ld p31_0,*diob++              /* first coherent read - end_angle and multi_pulse_count*/ 
		move ert_a,p; ld diob,*diob   /* second coherent read - dwell_time */
		move ert_b,diob
	};
#endif
	multi_pulse_count = p31_24;
	end_angle = erta;
	dwell_time = ertb;
}


/*******************************************************************************
*  eTPU Function
*******************************************************************************/

/**************************************************************************
* THREAD NAME: INIT
* DESCRIPTION: Initialize the channel to run the SPARK function.
**************************************************************************/
_eTPU_thread SPARK::INIT(_eTPU_matches_disabled)
{
    Init_NoReturn();
}

/**************************************************************************
* THREAD NAME: UPDATE
* DESCRIPTION: Spark parameters are updated. Apply the update as possible.
**************************************************************************/
_eTPU_thread SPARK::UPDATE(_eTPU_matches_disabled)
{
	/* Read updated spark parameters */
	ReadSparkParams();

	/* Theoretically, the HSR_UPDATE can be serviced between a match and 
	 * the match service. In this case let the match be serviced. */
	if((cc.MRLA == 0) && (cc.MRLB == 0))
	{
		switch(state)
		{
		case SPARK_STATE_RECALC:
            /* Read next spark parameters to get (possibly) updated values */
            ReadSparkParams();

			/* Re-schedule RECALC_ANGLE */
			ScheduleRecalcAngle_NoReturn();
			break;

		case SPARK_STATE_START:
			/* Re-schedule START_ANGLE */
			ScheduleStartAngle_NoReturn();
			break;

		case SPARK_STATE_MIN_DWELL:
		case SPARK_STATE_MAX_DWELL:
			/* Re-schedule END_ANGLE */
			ertb = tdc_angle_actual - end_angle;
			channel.ERWB = ERW_WRITE_ERT_TO_MATCH;
			break;
		}
	}
}

/**************************************************************************
* THREAD NAME: RECALC_ANGLE
* DESCRIPTION: Set channel interrupt.
*              Check parameter values and schedule START_ANGLE.
**************************************************************************/
_eTPU_thread SPARK::RECALC_ANGLE(_eTPU_matches_disabled)
{
    if (channel.LSR == 1)
    {
        Init_NoReturn();
    }
    if (is_first_recalc)
    {
        /* channel interrupt */
        channel.CIRC = CIRC_INT_FROM_SERVICED;
        
        is_first_recalc = FALSE;
        angle_offset_recalc_working >>= 2;
        ScheduleRecalcAngle_NoReturn();
    }

    if((generation_disable == SPARK_GENERATION_ALLOWED) &&
       (dwell_time > 0))
    {
        /* Schedule START_ANGLE */
        ScheduleStartAngle_NoReturn();
    }
    else
    {
        /* Skip to next spark and schedule next RECALC_ANGLE */
        ScheduleNextRecalcAngle_NoReturn();
    }
}

/**************************************************************************
* THREAD NAME: START_ANGLE
* DESCRIPTION: Store pulse start time.
*              Schedule MIN_DWELL_TIME.
**************************************************************************/
_eTPU_thread SPARK::START_ANGLE(_eTPU_matches_disabled)
{
    if (channel.LSR == 1)
    {
        Init_NoReturn();
    }
    /* Store pulse start time */
    pulse_start_time = ertb;

    /* Schedule MIN_DWELL_TIME */
    ScheduleMinDwellTime_NoReturn();
}

/**************************************************************************
* THREAD NAME: MIN_DWELL_TIME
* DESCRIPTION: Processing at min dwell time.
**************************************************************************/
_eTPU_thread SPARK::MIN_DWELL_TIME(_eTPU_matches_disabled)
{
    if (channel.LSR == 1)
    {
        Init_NoReturn();
    }
    /* check for min dwell conditions - if end angle came before this
     * min dwell match, flag it */
    if ((tdc_angle_actual - end_angle) - ertb <= 0)
    {
		/* Set error */
		error |= SPARK_ERROR_MIN_DWELL_APPLIED;
    }

    /* Schedule END_ANGLE and MAX_DWELL_TIME */
    ScheduleEndAngleAndMaxDwellTime_NoReturn();
}

/**************************************************************************
* THREAD NAME: END_ANGLE
* DESCRIPTION: Store applied dwell time.
*              Start the multi pulse sequence or schedule next RECAL_ANGLE.
**************************************************************************/
_eTPU_thread SPARK::END_ANGLE(_eTPU_matches_disabled)
{
    if (channel.LSR == 1)
    {
        Init_NoReturn();
    }
    /* Store applied dwell time */
    dwell_time_applied = ertb - pulse_start_time;

    /* Multi-pulse sequence ? */
    if(multi_pulse_count > 0)
    {
        /* Reset multi-pulse counter */
        multi_pulse_counter = 0;
        /* Schedule the first MULTI_PULSE */
        erta = ertb;
        ScheduleMultiPulse_NoReturn();
    }
    else
    {
        /* Schedule next RECALC_ANGLE */
        ScheduleNextRecalcAngle_NoReturn();
    }
}

/**************************************************************************
* THREAD NAME: MAX_DWELL_TIME
* DESCRIPTION: Store applied dwell time.
*              Set error MAX_DWELL_APPLIED.
*              Start the multi pulse sequence or schedule next RECAL_ANGLE.
**************************************************************************/
_eTPU_thread SPARK::MAX_DWELL_TIME(_eTPU_matches_disabled)
{
    if (channel.LSR == 1)
    {
        Init_NoReturn();
    }
    /* Store applied dwell time */
    dwell_time_applied = erta - pulse_start_time;

    /* Set error */
    error |= SPARK_ERROR_MAX_DWELL_APPLIED;

    /* Multi-pulse sequence ? */
    if(multi_pulse_count > 0)
    {
        /* Reset multi-pulse counter */
        multi_pulse_counter = 0;
        /* Schedule the first MULTI_PULSE */
        ScheduleMultiPulse_NoReturn();
    }
    else
    {
        /* Schedule next RECALC_ANGLE */
        ScheduleNextRecalcAngle_NoReturn();
    }
}

/**************************************************************************
* THREAD NAME: MULTI_PULSE
* DESCRIPTION: Schedule next MULTI_PULSE or RECAL_ANGLE.
**************************************************************************/
_eTPU_thread SPARK::MULTI_PULSE(_eTPU_matches_disabled)
{
    if (channel.LSR == 1)
    {
        Init_NoReturn();
    }
    multi_pulse_counter++;
    if(multi_pulse_counter < multi_pulse_count)
    {
        /* Schedule next MULTI_PULSE */
        ScheduleMultiPulse_NoReturn();
    }
    else
    {
        /* Schedule next RECALC_ANGLE */
        ScheduleNextRecalcAngle_NoReturn();
    }
}

/**************************************************************************
* THREAD NAME: LINK_OR_ERROR
* DESCRIPTION: Handles a possible asynchronous link (stall), or processes an error
**************************************************************************/
_eTPU_thread SPARK::LINK_OR_ERROR(_eTPU_matches_disabled)
{
    if (channel.LSR == 1)
    {
        Init_NoReturn();
    }
    _Error_handler_unexpected_thread();
}


DEFINE_ENTRY_TABLE(SPARK, SPARK, alternate, outputpin, autocfsr)
{
	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR2(6,7,   x,  x, x, x,  x, x, INIT),
	ETPU_VECTOR1(0,     1,  0, 0, 0,  x, x, INIT),
	ETPU_VECTOR1(0,     1,  0, 0, 1,  x, x, INIT),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR2(2,3,   x,  x, x, 0,  0, x, UPDATE),
	ETPU_VECTOR2(2,3,   x,  x, x, 0,  1, x, UPDATE),
	ETPU_VECTOR2(2,3,   x,  x, x, 1,  0, x, UPDATE),
	ETPU_VECTOR2(2,3,   x,  x, x, 1,  1, x, UPDATE),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  0, 1, 0,  0, 0, RECALC_ANGLE),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  0, 0, RECALC_ANGLE),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  0, 1, 0,  0, 1, START_ANGLE),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  0, 1, START_ANGLE),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  0, 1, START_ANGLE),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  0, 1, START_ANGLE),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  1, 0, 0,  1, 1, MIN_DWELL_TIME),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  1, 1, MIN_DWELL_TIME),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  1, 1, MIN_DWELL_TIME),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  1, 1, MIN_DWELL_TIME),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  0, 1, 0,  1, 0, END_ANGLE),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  1, 0, END_ANGLE),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  1, 0, END_ANGLE),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  1, 0, END_ANGLE),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  1, 0, 0,  1, 0, MAX_DWELL_TIME),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  1, 0, MAX_DWELL_TIME),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  1, 1, 0,  0, 0, MULTI_PULSE),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  0, 0, MULTI_PULSE),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  1, 0, 0,  0, 0, LINK_OR_ERROR),
	ETPU_VECTOR1(0,     x,  1, 0, 0,  0, 1, LINK_OR_ERROR),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  0, 0, LINK_OR_ERROR),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  0, 1, LINK_OR_ERROR),
	ETPU_VECTOR1(0,     x,  0, 1, 0,  1, 1, LINK_OR_ERROR),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  1, 1, LINK_OR_ERROR),

    // unused/invalid entries
	ETPU_VECTOR3(1,4,5, x,  x, x, x,  x, x, _Error_handler_unexpected_thread),
};


/*******************************************************************************
*  Export interface information to Host CPU program.
*******************************************************************************/
/* [MISRA 2004 Rule 3.4] usage of #pragma write documented in the Pragma Write 
   Manual, see https://www.ashware.com/Manuals */
#pragma write h, (::ETPUfilename (cpu/etpu_spark_auto.h));
#pragma write h, (/**************************************************************** );
#pragma write h, (* WARNING: This file is automatically generated. DO NOT EDIT IT! );
#pragma write h, (*);
#pragma write h, (* FILE NAME: etpu_spark.h  );
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
#pragma write h, (#ifndef _ETPU_SPARK_AUTO_H_ );
#pragma write h, (#define _ETPU_SPARK_AUTO_H_ );
#pragma write h, ( );
#pragma write h, (/* Function Configuration Information */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_FUNCTION_NUMBER) ::ETPUfunctionnumber(SPARK) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_TABLE_SELECT) ::ETPUentrytype(SPARK) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_NUM_PARMS) ::ETPUram(SPARK) );
#pragma write h, ( );
#pragma write h, (/* Host Service Request Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_HSR_INIT)        SPARK_HSR_INIT );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_HSR_UPDATE)      SPARK_HSR_UPDATE );
#pragma write h, ( );
#pragma write h, (/* Function Mode Bit Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_FM0_ACTIVE_LOW)  SPARK_FM0_ACTIVE_LOW );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_FM0_ACTIVE_HIGH) SPARK_FM0_ACTIVE_HIGH );
#pragma write h, ( );
#pragma write h, (/* Parameter Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_TDC_ANGLE                 ) ::ETPUlocation (SPARK, tdc_angle ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_TDC_ANGLE_ACTUAL          ) ::ETPUlocation (SPARK, tdc_angle_actual ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_ANGLE_OFFSET_RECALC       ) ::ETPUlocation (SPARK, angle_offset_recalc ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_DWELL_TIME_MIN            ) ::ETPUlocation (SPARK, dwell_time_min ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_DWELL_TIME_MAX            ) ::ETPUlocation (SPARK, dwell_time_max ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_MULTI_ON_TIME             ) ::ETPUlocation (SPARK, multi_on_time ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_MULTI_OFF_TIME            ) ::ETPUlocation (SPARK, multi_off_time ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_P_SINGLE_SPARK_FIRST      ) ::ETPUlocation (SPARK, p_single_spark_first ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_P_SINGLE_SPARK            ) ::ETPUlocation (SPARK, p_single_spark ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_PULSE_START_TIME          ) ::ETPUlocation (SPARK, pulse_start_time ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_DWELL_TIME_APPLIED        ) ::ETPUlocation (SPARK, dwell_time_applied ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_DWELL_TIME                ) ::ETPUlocation (SPARK, dwell_time ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_END_ANGLE                 ) ::ETPUlocation (SPARK, end_angle ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_SPARK_COUNT               ) ::ETPUlocation (SPARK, spark_count ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_SPARK_COUNTER             ) ::ETPUlocation (SPARK, spark_counter ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_MULTI_PULSE_COUNT         ) ::ETPUlocation (SPARK, multi_pulse_count ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_MULTI_PULSE_COUNTER       ) ::ETPUlocation (SPARK, multi_pulse_counter ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_STATE                     ) ::ETPUlocation (SPARK, state ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_ERROR                     ) ::ETPUlocation (SPARK, error ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_OFFSET_GENERATION_DISABLE        ) ::ETPUlocation (SPARK, generation_disable ) );
#pragma write h, ( );
#pragma write h, (/* Error Flags Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_ERROR_MIN_DWELL_APPLIED)        SPARK_ERROR_MIN_DWELL_APPLIED);
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_ERROR_MAX_DWELL_APPLIED)        SPARK_ERROR_MAX_DWELL_APPLIED);
#pragma write h, ( );
#pragma write h, (/* Generation Disable Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_GENERATION_ALLOWED)             SPARK_GENERATION_ALLOWED);
#pragma write h, (::ETPUliteral(#define FS_ETPU_SPARK_GENERATION_DISABLED)            SPARK_GENERATION_DISABLED);
#pragma write h, ( );
#pragma write h, (/* Spark Structure Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_SINGLE_SPARK_OFFSET_MULTI_PULSE_COUNT) 0x00 );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SINGLE_SPARK_OFFSET_END_ANGLE)         0x01 );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SINGLE_SPARK_OFFSET_DWELL_TIME)        0x05 );
#pragma write h, (::ETPUliteral(#define FS_ETPU_SINGLE_SPARK_STRUCT_SIZE)              0x08 );
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
*  Revision 1.1  2020/08/14  nxa17216
*  Added initialization of pin state into an inactive state within INIT thread.
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
