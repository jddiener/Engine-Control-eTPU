/*******************************************************************************
*
*  FREESCALE SEMICONDUCTOR INC. 2012-2014
*  ALL RIGHTS RESERVED
*  COPYRIGHT (c)
*
********************************************************************************
*
*  FILE NAME:  etpuc_inj.h
*
*******************************************************************************/
#ifndef __ETPUC_INJ_H
#define __ETPUC_INJ_H

/*******************************************************************************
*  Definitions
*******************************************************************************/
/* Host Service Requests */
#define INJ_HSR_INIT               7
#define INJ_HSR_STOP               5
#define INJ_HSR_UPDATE             3

/* Function Modes */
#define INJ_FM0_ACTIVE_HIGH        0
#define INJ_FM0_ACTIVE_LOW         1
#define INJ_FM1_CHANNEL_INJ        0
#define INJ_FM1_CHANNEL_BANK       1

/* Channel Flags */
#define INJ_FLAG0_INJ_SEQ_NOT_ACTIVE   0
#define INJ_FLAG0_INJ_SEQ_ACTIVE       1
#define INJ_FLAG1_IRQ_ANGLE            0
#define INJ_FLAG1_STOP_ANGLE           1

/* Error Flags */
#define INJ_ERROR_PREV_INJ_NOT_FINISHED  0x01
#define INJ_ERROR_LATE_START_ANGLE_1ST   0x02
#define INJ_ERROR_LATE_START_ANGLE_NTH   0x04
#define INJ_ERROR_STOPPED_BY_STOP_ANGLE  0x08

/* Injection Phase Options */
#define INJ_PH_OPT_TRIGGER         0x80
#define INJ_PH_OPT_IRQ             0x40
#define INJ_PH_OPT_DMA             0x20


/* helper type to allow for arithmetic on 32-bit values */
union INJ_32_BIT
{
    struct {
        uint8_t bits31_24;
        uint24_t bits23_0;
    } parts;
    uint32_t all;
};

/* Data Structure Types */
typedef struct INJ_PHASE
{
	const   int8_t dmas_outputs; /* DMA requests; output pin states */
  /* Setting of output states during this phase:
     bit 0x01 - injector channel output state 
     bit 0x02 - bank channel 1 output state (applies only if num_bank_chans > 0) 	    
     bit 0x04 - bank channel 2 output state (applies only if num_bank_chans > 1) 	    
     bit 0x08 - bank channel 3 output state (applies only if num_bank_chans > 2)
    Setting of DMA or DMA&IRQ requests at the beginning of this phase:
     bit 0x10 - DMA request from the injector channel  	    
     bit 0x20 - DMA&IRQ request from the bank channel 1 (applies only on eTPU2 and if num_bank_chans > 0)	    
     bit 0x40 - DMA&IRQ request from the bank channel 2 (applies only on eTPU2 and if num_bank_chans > 1)	    
     bit 0x80 - DMA&IRQ request from the bank channel 3 (applies only on eTPU2 and if num_bank_chans > 2)	    
  */
	const uint24_t duration;     /* TCR1 duration of the injection phase */
};

typedef struct INJ_INJECTION
{
	const uint8_t phase_count;      /* number of injection phases */
	const struct INJ_PHASE *p_phase_first; /* pointer to the first phase structure */
	const int24_t angle_start;     /* TDC-relative TCR2 injection start angle */
};

/* Global Data Structure Type */
typedef struct INJ_GLOBAL_PARAMS
{
	      union INJ_32_BIT active_bank_chans; /* bits set corresponds to bank channels used in a running injection sequence */
};


/* INJ eTPU function class declaration */
_eTPU_class INJ
{
    /* channel frame */
public:
	const uint8_t  injection_count;  /* count of injections */
	const struct INJ_INJECTION *p_injection_first; /* pointer to the first injection structure */
	      uint8_t  injection_counter;/* counts injections from 1 to num_injection */
	      struct INJ_INJECTION *p_injection; /* pointer to the actual injection structure */
	      uint8_t  phase_counter;    /* counts injection phases from 1 to phase_count */
	      struct INJ_PHASE *p_phase; /* pointer to the actual phase structure */
	const uint8_t  bank_chan_count;  /* count of BANK channels; 0 to 3 */
	const uint24_t bank_chans;       /* up to 3 BANK channel numbers packed into 3 bytes */
	const union INJ_32_BIT bank_chans_mask;  /* bits corresponding to BANK channel numbers */
	      uint8_t  error;            /* error flags */
	const  int24_t angle_irq;        /* TDC-relative TCR2 angle */
	const uint8_t  inactive_polarities; /* inactive output polarities of INJ (bit0) and BANK (bits 1-3) channels */
	const  int24_t angle_stop;       /* TDC-relative TCR2 latest stop angle */
	const  int24_t tdc_angle;        /* TCR2 angle relative to engine-cycle start */
	       int24_t tdc_angle_actual; /* absolute TDC TCR2 angle */


    /************************************/
    
    /* threads */
    
    _eTPU_thread INIT(_eTPU_matches_disabled);
    _eTPU_thread STOP(_eTPU_matches_disabled);
    _eTPU_thread UPDATE(_eTPU_matches_disabled);
    _eTPU_thread START_ANGLE_1ST(_eTPU_matches_disabled);
    _eTPU_thread PHASE(_eTPU_matches_disabled);
    _eTPU_thread STOP_ANGLE_WHILE_ACTIVE(_eTPU_matches_disabled);
    _eTPU_thread STOP_ANGLE_POST_ACTIVE(_eTPU_matches_disabled);
    _eTPU_thread IRQ_ANGLE(_eTPU_matches_disabled);
    
    
    /************************************/
    
    /* methods and fragments */
    
    void ScheduleStartAngle1st(void);
    void ScheduleIRQAngle(void);
    _eTPU_fragment StopBankChannels_NoReturn(void);
    _eTPU_fragment Phase_NoReturn(void);
    _eTPU_fragment Init_NoReturn(void);
    
    
    /************************************/
    
    /* entry table */
    
    _eTPU_entry_table INJ;
};


#endif /* __ETPUC_INJ_H */

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
*  Revision 1.1  2014/12/15  r54529
*  Union INJ_32_BIT added to enable bank_chans_mask functionality over all 
*  32 bits (thanks to AshWare).
*
*  Revision 1.0  2014/03/06  r54529
*  Minor comment and formating improvements. MISRA compliancy check.
*  Ready for eTPU Engine Control Library release 1.0.
*
*  Revision 0.2  2013/07/30  r54529
*  ERROR_UPDATE_NOT_APPLIED removed.
*  
*  Revision 0.1  2012/10/25  r54529
*  Initial version.
*
*******************************************************************************/
