/*******************************************************************************
*
*  FREESCALE SEMICONDUCTOR INC. 2013-2014
*  ALL RIGHTS RESERVED
*  COPYRIGHT (c)
*
********************************************************************************
*
*  FILE NAME:  etpuc_knock.h
*
*******************************************************************************/
#ifndef __ETPUC_KNOCK_H
#define __ETPUC_KNOCK_H

/*******************************************************************************
*  Definitions
*******************************************************************************/
/* Host Service Requests */
#define KNOCK_HSR_INIT               7
#define KNOCK_HSR_STOP               5

/* Function Modes */
#define KNOCK_FM0_ACTIVE_HIGH        0
#define KNOCK_FM0_ACTIVE_LOW         1
#define KNOCK_FM1_MODE_GATE          0
#define KNOCK_FM1_MODE_TRIGGER       1

/* Channel Flags */
#define KNOCK_FLAG0_WINDOW_NOT_ACTIVE 0
#define KNOCK_FLAG0_WINDOW_ACTIVE     1
#define KNOCK_FLAG1_MODE_GATE         0
#define KNOCK_FLAG1_MODE_TRIGGER      1

/* IRQ and DMA options */
#define KNOCK_IRQ_AT_WINDOW_START    0x01
#define KNOCK_IRQ_AT_WINDOW_END      0x02
#define KNOCK_IRQ_AT_EVERY_TRIGGER   0x04
#define KNOCK_DMA_AT_WINDOW_START    0x10
#define KNOCK_DMA_AT_WINDOW_END      0x20
#define KNOCK_DMA_AT_EVERY_TRIGGER   0x40

/*******************************************************************************
*  Typedefs
*******************************************************************************/
/* Knock Window Data Structure Type */
typedef struct KNOCK_WINDOW
{
	const  int24_t start;     /* TDC-relative TCR2 angle start */
	const  int24_t width;     /* knock window TCR2 angle width */
};


/* KNOCK eTPU function class declaration */
_eTPU_class KNOCK
{
    /* channel frame */
public:
	const uint8_t  window_count;     /* count of knock windows */
	const struct KNOCK_WINDOW *p_window_first; /* pointer to the first knock window structure */
	      uint8_t  window_counter;   /* counts knock windows */
	      struct KNOCK_WINDOW *p_window; /* pointer to the current knock window structure */
	const  int24_t tdc_angle;        /* TCR2 angle relative to engine-cycle start */
	       int24_t tdc_angle_actual; /* absolute TDC TCR2 angle */
	       int24_t tcr2_window_start; /* absolute TCR2 window start angle  */
	       int24_t tcr2_window_end;   /* absolute TCR2 window end angle */
	const uint24_t trigger_period;   /* TCR1 trigger period */
	const uint8_t  irq_dma_options;  /* IRQ and DMA request selection */


    /************************************/
    
    /* threads */
    
    _eTPU_thread INIT(_eTPU_matches_disabled);
    _eTPU_thread STOP(_eTPU_matches_disabled);
    _eTPU_thread WINDOW_START(_eTPU_matches_disabled);
    _eTPU_thread WINDOW_END(_eTPU_matches_disabled);
    _eTPU_thread TRIGGER(_eTPU_matches_disabled);
    
    
    /************************************/
    
    /* methods and fragments */
    
    _eTPU_fragment ScheduleStartAngle_NoReturn(void);
    
    
    /************************************/
    
    /* entry table */
    
    _eTPU_entry_table KNOCK;
};

#endif /* __ETPUC_KNOCK_H */

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
*  Revision 1.0  2014/03/06  r54529
*  Minor comment and formating improvements. MISRA compliancy check.
*  Ready for eTPU Engine Control Library release 1.0.
*
*  Revision 0.1  2013/09/06  r54529
*  Initial version.
*
*******************************************************************************/
