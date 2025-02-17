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
*  FILE NAME:  etpuc_inj.c
*
*  DESCRIPTION:
*    This eTPU function enables to generate complex angle- and time-based output
*    patterns, primarily dedicated to direct injection timing control.
*    The INJ channel can control the outputs of up to 3 additional channels, 
*    called BANK channels (e.g. Boost, Batt, ...).
*    
*    An injection sequence consists of injections. Each injection starts at a
*    defined angle (angle_start) and consists of phases. Each phase is defined
*    by output states of the INJ and all the BANK channels, a phase duration, 
*    and options to generate DMA requests at the beginning of the phase.
*    The number of injections and the number of phases in each injection is 
*    configurable. More INJ channels (individual injectors) may use separate
*    injection sequences, or can share the same sequence. 
*    
*    There is an INJ parameter tdc_angle, relative to which all angles are 
*    defined. Positive angles precede the tdc_angle, negative angles come after.
*    INJ parameter angle_irq defines a tdc_angle-relative angle at which an IRQ
*    request is generated. The CPU may reconfigure the injection sequence
*    setting on this interrupt, but not later then the first injection 
*    angle_start is reached. If the CPU does not reconfigure, the actual 
*    injection sequence definition is used.
*    INJ parameter angle_stop defines the latest tdc_angle-relative angle when
*    the whole injection sequence must be finished. If it is not, all INJ and
*    BANK outputs are turned to inactive state, whatever injection phase is 
*    active.
*       
*    The CPU can monitor the INJ operation using INJ state variables
*    injection_counter, phase_counter and error. The reported error flags are:
*    INJ_ERROR_PREV_INJ_NOT_FINISHED - injection sequence can not start while 
*      another INJ channel occupies the BANK channels. The injection sequence 
*      is not generated. The global parameter active_bank_chans keeps
*      track of which BANK channels are in use.
*    INJ_ERROR_LATE_START_ANGLE_1ST - the 1st injection start-angle was about
*      to be scheduled in past, hence the whole injection sequence was skipped. 
*    INJ_ERROR_LATE_START_ANGLE_NTH - the 2nd or later injection start-angle 
*      was about to be scheduled in past, hence the rest of the injection 
*      sequence was skipped.
*    INJ_ERROR_STOPPED_BY_STOP_ANGLE - the injection sequence was not finished 
*      before the stop-angle and hence the injection was hard-stopped at the
*      stop-angle. 
*
*******************************************************************************/

/*******************************************************************************
*  Includes
*******************************************************************************/
#include <etpu_std.h>
#include "etpuc_inj.h"
#include "etpuc_crank.h"
#include "etpuc_set.h"

/*******************************************************************************
*  eTPU Function Parameters:
*
*  Global
*  ------
*  active_bank_chans - individual bits of this parameter determine 
*    which BANK channels are in use. It is used to prevent from running
*    2 injections, sharing the same BANK channels, in parallel.
*  
*  INJ Channel Parameters (struct INJ_CHAN_PARAMS)
*  -----------------------------------------------
*  injection_count - count of injections
*  injection_counter - counts injections from 1 to num_injection
*  phase_counter - counts injection phases from 1 to p_injection->phase_count
*  *p_injection_first - pointer to the first item in the array of injection 
*                       structures
*  *p_injection - pointer to the actual item in the array of injection 
*                 structures
*  *p_phase - pointer to the actual item in the array of phase structures
*  error - error flags
*  bank_chans - up to 3 BANK channel numbers in 3 bytes of uint24_t
*  bank_chan_count - count of BANK channels; 0 to 3
*  tdc_angle - TCR2 angle relative to engine-cycle start
*  tdc_angle_actual - absolute actual TDC TCR2 angle
*  angle_irq - TDC-relative TCR2 injector IRQ angle
*  angle_stop - TDC-relative TCR2 injector latest stop angle
*
*  Injection Structure Parameters (struct INJ_INJECTION)
*  -----------------------------------------------------
*  phase_count - number of injection phases
*  *p_phase_first - pointer to the first item in the array of phase structures
*  angle_start - TDC-relative TCR2 injection start angle
*
*  Phase Structure Parameters (struct INJ_PHASE)
*  -----------------------------------------------------
*  dmas_outputs - DMA requests and output pin states:
*    Setting of output states during this phase:
*     bit 0x01 - injector channel output state 
*     bit 0x02 - bank channel 1 output state (applies only if bank_chan_count > 0) 	    
*     bit 0x04 - bank channel 2 output state (applies only if bank_chan_count > 1) 	    
*     bit 0x08 - bank channel 3 output state (applies only if bank_chan_count > 2)
*    Setting of DMA or DMA&IRQ requests at the beginning of this phase:
*     bit 0x10 - DMA request from the injector channel  	    
*     bit 0x20 - DMA&IRQ request from the bank channel 1 (applies only on eTPU2 and if bank_chan_count > 0)	    
*     bit 0x40 - DMA&IRQ request from the bank channel 2 (applies only on eTPU2 and if bank_chan_count > 1)	    
*     bit 0x80 - DMA&IRQ request from the bank channel 3 (applies only on eTPU2 and if bank_chan_count > 2)	    
*  duration - TCR1 time duration of the injection phase
*  
********************************************************************************
*
*  Channel Flag usage
*    Flag0 = INJ_FLAG0_INJ_SEQ_NOT_ACTIVE (0)  ... Injection sequence is not active
*    Flag0 = INJ_FLAG0_INJ_SEQ_ACTIVE (1)      ... Injection sequence is active
*    Flag1 = INJ_FLAG1_IRQ_ANGLE (0)           ... IRQ_ANGLE is scheduled.
*    Flag1 = INJ_FLAG1_STOP_ANGLE (1)          ... STOP_ANGLE is scheduled.
*
*******************************************************************************/

/*******************************************************************************
*  Global Variables
*******************************************************************************/
volatile struct INJ_GLOBAL_PARAMS inj_global;


/*******************************************************************************
*  eTPU Class Methods/Fragments
*******************************************************************************/

/*******************************************************************************
*  FUNCTION NAME: ScheduleIRQAngle
*  DESCRIPTION: Schedule the IRQ_ANGLE, set flag.
*******************************************************************************/
void INJ::ScheduleIRQAngle(void)
{
	/* Schedule IRQ_ANGLE */
	ertb = tdc_angle_actual - angle_irq;
	channel.MRLB = MRL_CLEAR;
	channel.ERWB = ERW_WRITE_ERT_TO_MATCH;
	/* Channel flag */
	channel.FLAG1 = INJ_FLAG1_IRQ_ANGLE;
}

/*******************************************************************************
*  FUNCTION NAME: ScheduleStartAngle1st
*  DESCRIPTION: Schedule the START_ANGLE_1ST, check if it is not too late.
*******************************************************************************/
void INJ::ScheduleStartAngle1st(void)
{
	const struct INJ_INJECTION *p_inj;

	/* Channel flag */
	channel.FLAG0 = INJ_FLAG0_INJ_SEQ_NOT_ACTIVE;

	/* If there is at least one injection */
	if(injection_count > 0)
	{
		/* Schedule the START_ANGLE_1ST */
		p_inj = p_injection_first;
		erta = tdc_angle_actual - p_inj->angle_start;
		/* check if it is not late */
		if((int24_t)erta - (int24_t)tcr2 >= 0) 
		{
			channel.OPACA = OPAC_NO_CHANGE; /* all pin states will be set during START_ANGLE_1ST service */
			channel.TBSA = TBS_M2C1GE;  /* match on angle, capture time */
			channel.MRLA = MRL_CLEAR;
			channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
		}
		else
		{
			/* The start-angle is over, skip the rest of injections */
			/* Set error flag */
			error |= INJ_ERROR_LATE_START_ANGLE_1ST;
		}
	}
}

/*******************************************************************************
*  FUNCTION NAME: StopBankChannels_NoReturn
*  DESCRIPTION: On all BANK channels:
*               - disable matches
*               - turn output pin inactive
*******************************************************************************/
_eTPU_fragment INJ::StopBankChannels_NoReturn(void)
{
	uint24_t cnt;
	uint24_t chn;
	uint24_t out;

	cnt = bank_chan_count;
	chn = bank_chans;
	out = inactive_polarities;
	while(cnt > 0)
	{
		chan = (uint8_t)chn;
		cnt--;
		chn >>= 8;
		out >>= 1;
		/* Disable match detection */
		channel.MRLE = MRLE_DISABLE;
		/* Reset match latches */
		channel.MRLA = MRL_CLEAR;
		channel.MRLB = MRL_CLEAR;
		/* Output pin state */
		if(out & 0x01)
		{
			channel.PIN = PIN_SET_HIGH;
		}
		else
		{
			channel.PIN = PIN_SET_LOW;
		}
	}
}

/*******************************************************************************
*  FUNCTION NAME: Phase_NoReturn
*  DESCRIPTION: Processing a single injection phase.
*******************************************************************************/
_eTPU_fragment INJ::Phase_NoReturn(void)
{
	uint24_t cnt;
	uint24_t dma;
	uint24_t out;
	uint24_t chn;
	uint24_t end;

	struct INJ_INJECTION *p_inj;
	struct INJ_PHASE *p_ph;

	p_inj = p_injection;
	p_ph = p_phase;
	end = erta + p_ph->duration; /* this phase end time */
	dma = p_ph->dmas_outputs; /* this phase DMA requests */

	/* Increment phase counter */
	phase_counter++;
	/* Idle phase? */
	if(phase_counter > p_inj->phase_count)
	{
		/* This is the Idle phase */

		/* More injections in the sequence? */
		if(injection_counter < injection_count)
		{
			/* There are more injections to do */
			/* Increment injection counter and p_injection pointer*/
			injection_counter++;
			p_inj++;
			p_injection = p_inj;
			/* Reset phase counters and initialize p_phase pointer */
			phase_counter = 0;
			p_phase = p_inj->p_phase_first;

			/* Schedule the next injection start-angle PHASE */
			erta = tdc_angle_actual - p_inj->angle_start;
			/* check if it is not late */
			if((int24_t)erta - (int24_t)tcr2 >= 0)
			{
				/* Schedule next phase output on INJ channel */
				out = p_phase->dmas_outputs;
				if(out & 0x01)
				{
					channel.OPACA = OPAC_MATCH_HIGH;
				}
				else
				{
					channel.OPACA = OPAC_MATCH_LOW;
				}
				channel.MRLA = MRL_CLEAR;
				channel.TBSA = TBS_M2C1GE;  /* match on angle, capture time */
				channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
				/* Schedule next phase outputs on BANK channels */
				end = erta;
				cnt = bank_chan_count;
				chn = bank_chans;
				while(cnt > 0)
				{
					cnt--;
					chan = (uint8_t)chn;
					chn = chn >> 8;
					erta = end;
					out >>= 1;
					if(out & 0x01)
					{
						channel.OPACA = OPAC_MATCH_HIGH;
					}
					else
					{
						channel.OPACA = OPAC_MATCH_LOW;
					}
					channel.MRLA = MRL_CLEAR;
					channel.TBSA = TBS_M2C1GE;  /* match on angle, capture time */
					channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
				}
			}
			else
			{
				/* The start-angle is over, skip the rest of injections */
				/* Set error flag */
				error |= INJ_ERROR_LATE_START_ANGLE_NTH;
				/* Free BANK channels for other injectors */
				inj_global.active_bank_chans.parts.bits31_24 &= ~bank_chans_mask.parts.bits31_24;
				inj_global.active_bank_chans.parts.bits23_0  &= ~bank_chans_mask.parts.bits23_0;
				/* Channel flag */
				channel.FLAG0 = INJ_FLAG0_INJ_SEQ_NOT_ACTIVE;
				channel.MRLA = MRL_CLEAR;
			}
		}
		else
		{
			/* This is the last injection */
			/* Free BANK channels for other injectors */
			inj_global.active_bank_chans.parts.bits31_24 &= ~bank_chans_mask.parts.bits31_24;
			inj_global.active_bank_chans.parts.bits23_0  &= ~bank_chans_mask.parts.bits23_0;
			/* Channel flag */
			channel.FLAG0 = INJ_FLAG0_INJ_SEQ_NOT_ACTIVE;
			channel.MRLA = MRL_CLEAR;
			/* Reset injection counter */
			injection_counter = 0;
		}
	}
	else
	{
		/* Last injection phase? */
		if(phase_counter < p_inj->phase_count)
		{
			/* This is not the last phase */
			p_ph++;
			p_phase = p_ph; /* store pointer to the next phase */
			out = p_ph->dmas_outputs; /* next phase outputs */
		}
		else /* phase_counter == p_inj->phase_count */
		{
			/* This is the last phase */
			out = inactive_polarities;
		}
	
		/* Schedule next phase output on INJ channel */
		erta = end;
		if(out & 0x01)
		{
			channel.OPACA = OPAC_MATCH_HIGH;
		}
		else
		{
			channel.OPACA = OPAC_MATCH_LOW;
		}
		channel.MRLA = MRL_CLEAR;
		channel.TBSA = TBS_M1C1GE;  /* match on time, capture time */
		channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
		/* Generate DMA on INJ channel */
		if(dma & 0x10)
		{
			channel.CIRC = CIRC_DATA_FROM_SERVICED;
		}
		/* Schedule next phase outputs on BANK channels */
		cnt = bank_chan_count;
		chn = bank_chans;
		while(cnt > 0)
		{
			cnt--;
			chan = (uint8_t)chn;
			chn = chn >> 8;
			erta = end;
			out >>= 1;
			if(out & 0x01)
			{
				channel.OPACA = OPAC_MATCH_HIGH;
			}
			else
			{
				channel.OPACA = OPAC_MATCH_LOW;
			}
			channel.MRLA = MRL_CLEAR;
			channel.TBSA = TBS_M1C1GE;  /* match on time, capture time */
			channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
			/* Generate DMA&IRQ on BANK channels */
#if defined(__TARGET_ETPU2__)
			dma >>= 1;
			if(dma & 0x10)
			{
				channel.CIRC = CIRC_BOTH_FROM_SELECTED;
			}
#endif
		}
	}
}

/*******************************************************************************
*  FUNCTION NAME: Init_NoReturn
*  DESCRIPTION: Initialize the channel to run the INJ function.
*******************************************************************************/
_eTPU_fragment INJ::Init_NoReturn(void)
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
	channel.TBSB = TBS_M2C2GE;
	/* Input pin action control */
	channel.IPACA = IPAC_NO_DETECT;
	channel.IPACB = IPAC_NO_DETECT;
	/* Output pin state and Output pin action control */
	if(channel.FM0 == INJ_FM0_ACTIVE_HIGH)
	{
		channel.PIN = PIN_SET_LOW;
		channel.OPACA = OPAC_MATCH_LOW;
		channel.OPACB = OPAC_NO_CHANGE;
	}
	else
	{
		channel.PIN = PIN_SET_HIGH;
		channel.OPACA = OPAC_MATCH_HIGH;
		channel.OPACB = OPAC_NO_CHANGE;
	}
	/* Enable output pin buffer */
	channel.TBSA = TBSA_SET_OBE;
	
	/* Free BANK channels */
	inj_global.active_bank_chans.parts.bits31_24 &= ~bank_chans_mask.parts.bits31_24;
	inj_global.active_bank_chans.parts.bits23_0  &= ~bank_chans_mask.parts.bits23_0;

	/* On INJ channel schedule the first IRQ_ANGLE and START_ANGLE_1ST */
	if(channel.FM1 == INJ_FM1_CHANNEL_INJ)
	{
		/* Enable event handling */
		channel.MTD = MTD_ENABLE;
		
		/* Initialize actual TDC angle */
		tdc_angle_actual = eng_cycle_tcr2_start + tdc_angle;

        if (eng_pos_state != ENG_POS_FULL_SYNC)
        {
            /* only fully start the injection processing once sync is achieved */
            return;
        }

		/* Schedule IRQ_ANGLE */
		ScheduleIRQAngle();
		
		/* Schedule START_ANGLE_1ST */
		ScheduleStartAngle1st();
	}
}


/*******************************************************************************
*  eTPU Function
*******************************************************************************/

/**************************************************************************
* THREAD NAME: INIT
* DESCRIPTION: Initialize the channel to run the INJ function.
*              This HSR thread must be executed on all INJ and all BANK 
*              channels. 
**************************************************************************/
_eTPU_thread INJ::INIT(_eTPU_matches_disabled)
{
	Init_NoReturn();
}

/**************************************************************************
* THREAD NAME: STOP
* DESCRIPTION: Stop the running injection sequence.
*              The next injection sequence will start normally. 
**************************************************************************/
_eTPU_thread INJ::STOP(_eTPU_matches_disabled)
{
	/* Stop the INJ channel - schedule an immediate match A to set pin inactive
	   and rewrite any match already scheduled */
	/* Output pin action control */
	if(channel.FM0 == INJ_FM0_ACTIVE_HIGH)
	{
		channel.OPACA = OPAC_MATCH_LOW;
	}
	else
	{
		channel.OPACA = OPAC_MATCH_HIGH;
	}
	/* Time base selection */
	channel.TBSA = TBS_M1C1GE;
	/* Schedule match now */
	erta = tcr1;
	channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
	/* Channel flag */
	channel.FLAG0 = INJ_FLAG0_INJ_SEQ_NOT_ACTIVE;

	/* Free BANK channels for other injectors */
	inj_global.active_bank_chans.parts.bits31_24 &= ~bank_chans_mask.parts.bits31_24;
	inj_global.active_bank_chans.parts.bits23_0  &= ~bank_chans_mask.parts.bits23_0;

	/* Clear the immediate match */
	channel.MRLA = MRL_CLEAR;
	
	/* Stop the BANK channels */
	StopBankChannels_NoReturn();
}

/**************************************************************************
* THREAD NAME: UPDATE
* DESCRIPTION: The injection sequence parameters are updated. 
*              Check if it is not too late and reschedule START_ANGLE_1ST. 
**************************************************************************/
_eTPU_thread INJ::UPDATE(_eTPU_matches_disabled)
{
	ScheduleStartAngle1st();
}

/**************************************************************************
* THREAD NAME: START_ANGLE_1ST
* DESCRIPTION: The first injection first PHASE. Check if the previous 
*              injection sequence (driven by another INJ channel, but 
*              sharing the same BANK channels) is already finished.
**************************************************************************/
_eTPU_thread INJ::START_ANGLE_1ST(_eTPU_matches_disabled)
{
	uint24_t out;
	uint24_t cnt;
	uint24_t chn;
	uint8_t  inj_chan;
	
	/* prioritize Link service over Match service */
	if(channel.LSR == 1) 
	{
		Init_NoReturn();
	}
    else
    {
		/* Check if previous injection sequence is finished */
		if (((inj_global.active_bank_chans.parts.bits23_0 & bank_chans_mask.parts.bits23_0) == 0) &&
		    ((inj_global.active_bank_chans.parts.bits31_24 & bank_chans_mask.parts.bits31_24) == 0))
		{
			/* Occupy BANK channels */
			inj_global.active_bank_chans.parts.bits31_24 |= bank_chans_mask.parts.bits31_24;
			inj_global.active_bank_chans.parts.bits23_0  |= bank_chans_mask.parts.bits23_0;
			/* Channel flag */
			channel.FLAG0 = INJ_FLAG0_INJ_SEQ_ACTIVE;
			/* Init injection and phase counters */
			injection_counter = 1;
			phase_counter = 0;
			/* Initialize p_injection and p_phase pointers */
			p_injection = p_injection_first;
			p_phase = p_injection_first->p_phase_first;

			/* Set pin states */
			out = p_phase->dmas_outputs;
			cnt = bank_chan_count;
			chn = bank_chans;
			/* on INJ channel */
			if(out & 0x01)
			{
				channel.PIN = PIN_SET_HIGH;
			}
			else
			{
				channel.PIN = PIN_SET_LOW;
			}
			/* on BANK channels */
			inj_chan = chan;
			while(cnt > 0)
			{
				cnt--;
				chan = (uint8_t)chn;
				chn = chn >> 8;
				out >>= 1;
				if(out & 0x01)
				{
					channel.PIN = PIN_SET_HIGH;
				}
				else
				{
					channel.PIN = PIN_SET_LOW;
				}
			}
			chan = inj_chan;
			erta = tcr1;

			/* Process as a normal PHASE */
			Phase_NoReturn();
		}
		else
		{
			/* Set error flag */
			error |= INJ_ERROR_PREV_INJ_NOT_FINISHED;
		}
	}
}

/**************************************************************************
* THREAD NAME: PHASE
* DESCRIPTION: A single injection phase.
**************************************************************************/
_eTPU_thread INJ::PHASE(_eTPU_matches_disabled)
{
	/* prioritize Link service over Match service */
	if(channel.LSR == 1) 
	{
		Init_NoReturn();
	}
    else
    {
        Phase_NoReturn();
    }
}

/**************************************************************************
* THREAD NAME: STOP_ANGLE_WHILE_ACTIVE
* DESCRIPTION: Stop the active sequence of injections.
*              Update TDC for next engine cycle.
*              Schedule IRQ_ANGLE.
*              Schedule START_ANGLE_1ST.
**************************************************************************/
_eTPU_thread INJ::STOP_ANGLE_WHILE_ACTIVE(_eTPU_matches_disabled)
{
	/* prioritize Link service over Match service */
	if(channel.LSR == 1) 
	{
		Init_NoReturn();
	}
	else
	{
		/* Set error flag */
		error |= INJ_ERROR_STOPPED_BY_STOP_ANGLE;

		/* INJ channel */
		/* Disable match detection */
		channel.MRLE = MRLE_DISABLE;
		/* Reset match A latch */
		channel.MRLA = MRL_CLEAR;
		/* Output pin state */
		if(channel.FM0 == INJ_FM0_ACTIVE_HIGH)
		{
			channel.PIN = PIN_SET_LOW;
		}
		else
		{
			channel.PIN = PIN_SET_HIGH;
		}
		/* Channel flag */
		channel.FLAG0 = INJ_FLAG0_INJ_SEQ_NOT_ACTIVE;

		/* reset to the injection counter as would occur on a normal end of injection */
		injection_counter = 0;
		
		/* Free BANK channels for other injectors */
		inj_global.active_bank_chans.parts.bits31_24 &= ~bank_chans_mask.parts.bits31_24;
		inj_global.active_bank_chans.parts.bits23_0  &= ~bank_chans_mask.parts.bits23_0;
		
		/* Update actual TDC angle for this cycle */
		tdc_angle_actual += eng_cycle_tcr2_ticks;

		/* Schedule IRQ_ANGLE */
		ScheduleIRQAngle();

		/* Schedule START_ANGLE_1ST */
		ScheduleStartAngle1st();
		
		/* BANK channels */
		StopBankChannels_NoReturn();
	}
}

/**************************************************************************
* THREAD NAME: STOP_ANGLE_POST_ACTIVE
* DESCRIPTION: The injection sequence is already finished, no need to stop
*              it.
*              Update TDC for next engine cycle.
*              Schedule IRQ_ANGLE.
*              Schedule START_ANGLE_1ST.
**************************************************************************/
_eTPU_thread INJ::STOP_ANGLE_POST_ACTIVE(_eTPU_matches_disabled)
{
	/* prioritize Link service over Match service */
	if(channel.LSR == 1) 
	{
		Init_NoReturn();
	}
    else
    {
		/* Update actual TDC angle for this cycle */
		tdc_angle_actual += eng_cycle_tcr2_ticks;

		/* Schedule IRQ_ANGLE */
		ScheduleIRQAngle();

		/* Schedule START_ANGLE_1ST */
		ScheduleStartAngle1st();
	}
}

/**************************************************************************
* THREAD NAME: IRQ_ANGLE
* DESCRIPTION: Generate channel interrupt.
*              Schedule STOP_ANGLE.
**************************************************************************/
_eTPU_thread INJ::IRQ_ANGLE(_eTPU_matches_disabled)
{
	/* prioritize Link service over Match service */
	if(channel.LSR == 1) 
	{
		Init_NoReturn();
	}
    else
    {
		/* Generate channel interrupt */
		channel.CIRC = CIRC_INT_FROM_SERVICED;

		/* Schedule STOP_ANGLE */
		ertb = tdc_angle_actual - angle_stop;
		channel.MRLB = MRL_CLEAR;
		channel.ERWB = ERW_WRITE_ERT_TO_MATCH;
		/* Channel flag */
		channel.FLAG1 = INJ_FLAG1_STOP_ANGLE;
	}
}


DEFINE_ENTRY_TABLE(INJ, INJ, alternate, outputpin, autocfsr)
{
	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR2(6,7,   x,  x, x, x,  x, x, INIT),
	ETPU_VECTOR1(0,     1,  0, 0, 0,  x, x, INIT),
	ETPU_VECTOR1(0,     1,  0, 0, 1,  x, x, INIT),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR3(1,4,5, x,  x, x, x,  x, x, STOP),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR2(2,3,   x,  x, x, 0,  0, x, UPDATE),
	ETPU_VECTOR2(2,3,   x,  x, x, 0,  1, x, UPDATE),
	ETPU_VECTOR2(2,3,   x,  x, x, 1,  0, x, UPDATE),
	ETPU_VECTOR2(2,3,   x,  x, x, 1,  1, x, UPDATE),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  1, 0, 0,  0, 0, START_ANGLE_1ST),
	ETPU_VECTOR1(0,     x,  1, 0, 0,  0, 1, START_ANGLE_1ST),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  0, 0, START_ANGLE_1ST),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  0, 1, START_ANGLE_1ST),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  0, 0, START_ANGLE_1ST),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  0, 1, START_ANGLE_1ST),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  0, 0, START_ANGLE_1ST),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  0, 1, START_ANGLE_1ST),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  1, 0, 0,  1, 0, PHASE),
	ETPU_VECTOR1(0,     x,  1, 0, 0,  1, 1, PHASE),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  1, 0, PHASE),
	ETPU_VECTOR1(0,     x,  1, 0, 1,  1, 1, PHASE),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  1, 0, PHASE),
	ETPU_VECTOR1(0,     x,  1, 1, 0,  1, 1, PHASE),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  1, 0, PHASE),
	ETPU_VECTOR1(0,     x,  1, 1, 1,  1, 1, PHASE),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  0, 1, 0,  1, 1, STOP_ANGLE_WHILE_ACTIVE),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  1, 1, STOP_ANGLE_WHILE_ACTIVE),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  0, 1, 0,  0, 1, STOP_ANGLE_POST_ACTIVE),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  0, 1, STOP_ANGLE_POST_ACTIVE),

	//           HSR    LSR M1 M2 PIN F0 F1 vector
	ETPU_VECTOR1(0,     x,  0, 1, 0,  0, 0, IRQ_ANGLE),
	ETPU_VECTOR1(0,     x,  0, 1, 0,  1, 0, IRQ_ANGLE),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  0, 0, IRQ_ANGLE),
	ETPU_VECTOR1(0,     x,  0, 1, 1,  1, 0, IRQ_ANGLE),

    // unused/invalid entries
};


/*******************************************************************************
*  Export interface information to Host CPU program. 
*******************************************************************************/
/* [MISRA 2004 Rule 3.4] usage of #pragma write documented in the Pragma Write 
   Manual, see https://www.ashware.com/Manuals */
#pragma write h, (::ETPUfilename (cpu/etpu_inj_auto.h));
#pragma write h, (/**************************************************************** );
#pragma write h, (* WARNING: This file is automatically generated. DO NOT EDIT IT! );
#pragma write h, (*);
#pragma write h, (* FILE NAME: etpu_inj_auto.h  );
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
#pragma write h, (#ifndef _ETPU_INJ_AUTO_H_ );
#pragma write h, (#define _ETPU_INJ_AUTO_H_ );
#pragma write h, ( );
#pragma write h, (/* Function Configuration Information */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_FUNCTION_NUMBER) ::ETPUfunctionnumber(INJ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_TABLE_SELECT) ::ETPUentrytype(INJ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_NUM_PARMS) ::ETPUram(INJ) );
#pragma write h, ( );
#pragma write h, (/* Host Service Request Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_HSR_INIT)         INJ_HSR_INIT );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_HSR_UPDATE)       INJ_HSR_UPDATE );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_HSR_STOP)         INJ_HSR_STOP );
#pragma write h, ( );
#pragma write h, (/* Function Mode Bit Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_FM0_ACTIVE_HIGH)  (INJ_FM0_ACTIVE_HIGH) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_FM0_ACTIVE_LOW)   (INJ_FM0_ACTIVE_LOW) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_FM1_CHANNEL_INJ)  ((INJ_FM1_CHANNEL_INJ<<1)) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_FM1_CHANNEL_BANK) ((INJ_FM1_CHANNEL_BANK<<1)) );
#pragma write h, ( );
#pragma write h, (/* Parameter Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_INJECTION_COUNT)   ::ETPUlocation (INJ, injection_count) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_INJECTION_COUNTER) ::ETPUlocation (INJ, injection_counter) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_PHASE_COUNTER)     ::ETPUlocation (INJ, phase_counter) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_ERROR)             ::ETPUlocation (INJ, error) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_INACTIVE_POLARITIES) ::ETPUlocation (INJ, inactive_polarities) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_BANK_CHANS_COUNT)  ::ETPUlocation (INJ, bank_chan_count) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_BANK_CHANS)        ::ETPUlocation (INJ, bank_chans) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_BANK_CHANS_MASK)   ::ETPUlocation (INJ, bank_chans_mask) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_P_INJECTION_FIRST) ::ETPUlocation (INJ, p_injection_first) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_P_INJECTION)       ::ETPUlocation (INJ, p_injection) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_P_PHASE)           ::ETPUlocation (INJ, p_phase) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_ANGLE_IRQ)         ::ETPUlocation (INJ, angle_irq) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_ANGLE_STOP)        ::ETPUlocation (INJ, angle_stop) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_TDC_ANGLE)         ::ETPUlocation (INJ, tdc_angle) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_TDC_ANGLE_ACTUAL)  ::ETPUlocation (INJ, tdc_angle_actual) );
#pragma write h, ( );
#pragma write h, (/* Global Variable Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_ACTIVE_BANK_CHANS) ::ETPUlocation (inj_global.active_bank_chans) );
#pragma write h, ( );
#pragma write h, (/* Injection Structure Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_PHASE_COUNT)   0x00 );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_P_PHASE_FIRST) 0x01 );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_ANGLE_START)   0x05 );
#pragma write h, ( );
#pragma write h, (/* Phase Structure Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_DMAS_OUTPUTS)    0x00 );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_OFFSET_DURATION)        0x01 );
#pragma write h, ( );
#pragma write h, (/* Value Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_INJECTION_STRUCT_SIZE)  0x08 );
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_PHASE_STRUCT_SIZE)      0x04 );
#pragma write h, ( );
#pragma write h, (/* Error Flags Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_ERROR_PREV_INJ_NOT_FINISHED) INJ_ERROR_PREV_INJ_NOT_FINISHED);
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_ERROR_LATE_START_ANGLE_1ST)  INJ_ERROR_LATE_START_ANGLE_1ST);
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_ERROR_LATE_START_ANGLE_NTH)  INJ_ERROR_LATE_START_ANGLE_NTH);
#pragma write h, (::ETPUliteral(#define FS_ETPU_INJ_ERROR_STOPPED_BY_STOP_ANGLE) INJ_ERROR_STOPPED_BY_STOP_ANGLE);
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
*  Revision 1.2  2018/09/12  nxa17216
*  Bug fix: missing injection happening regularly with error 
*  FS_ETPU_INJ_ERROR_LATE_START_ANGLE_1ST. Condition code for checking
*  if it is not late for scheduling 1st injection at the start angle was fixed.
*  
*  Revision 1.1  2014/12/15  r54529
*  Union INJ_32_BIT added to enable bank_chans_mask functionality over all 
*  32 bits (thanks to AshWare).
*  Bug fix: after engine stall the INJ channels were not correctly reset and
*  not able to restart after resync. Handling link was fixed. 
*
*  Revision 1.0  2014/03/16  r54529
*  Minor comment and formating improvements. MISRA compliancy check.
*  Ready for eTPU Engine Control Library release 1.0.
*
*  Revision 0.2  2013/08/14  r54529
*  ERROR_UPDATE_NOT_APPLIED removed, asynchronous update enabled.
*  irq_angle can be anywhere before the stop_angle, even during the injection.
*  On link (stall condition from Crank), the INJ is re-initialized.
*  Workarounds removed. bank_chans_mask not functional for channels 24-31.
*
*  Revision 0.1  2012/11/12  r54529
*  Initial version. 
*  Includes compiler bug workarounds. eTPU2 version is not functional.
*  bank_chans_mask not functional for channels 24-31.
*
*******************************************************************************/
