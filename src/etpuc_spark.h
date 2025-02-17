/*******************************************************************************
*
*  FREESCALE SEMICONDUCTOR INC. 2008-2014
*  ALL RIGHTS RESERVED
*  COPYRIGHT (c)
*
********************************************************************************
*
*  FILE NAME:  etpuc_spark.h
*
*******************************************************************************/
#ifndef __ETPUC_SPARK_H
#define __ETPUC_SPARK_H

/*******************************************************************************
*  Definitions
*******************************************************************************/
/* Host Service Requests */
#define SPARK_HSR_INIT                  7
#define SPARK_HSR_UPDATE                3

/* Function Modes */
#define SPARK_FM0_ACTIVE_LOW            0
#define SPARK_FM0_ACTIVE_HIGH           1

/* Channel Flags */
#define SPARK_FLAG0_OUT_OF_MAIN_PULSE   0
#define SPARK_FLAG0_MAIN_PULSE          1
#define SPARK_FLAG1_POST_MIN_DWELL      0
#define SPARK_FLAG1_PRE_MIN_DWELL       1

/* Spark states - what is scheduled */
#define SPARK_STATE_RECALC              0
#define SPARK_STATE_START               1
#define SPARK_STATE_MIN_DWELL           2
#define SPARK_STATE_MAX_DWELL           3
#define SPARK_STATE_MULTI_PULSE         4

/* Error Flags */
#define SPARK_ERROR_MIN_DWELL_APPLIED   0x01
#define SPARK_ERROR_MAX_DWELL_APPLIED   0x02

/* Generation Disable flags */
#define SPARK_GENERATION_ALLOWED        0
#define SPARK_GENERATION_DISABLED       1

/*******************************************************************************
*  Typedefs
*******************************************************************************/
/* Spark Data Structure Type */
typedef struct SINGLE_SPARK
{
  const uint8_t  multi_pulse_count; /* count of multi-pulses */
  const  int24_t end_angle;         /* TDC-relative TCR2 end angle */
  const uint24_t dwell_time;        /* TCR1 dwell time */
};


/* SPARK eTPU function class declaration */
_eTPU_class SPARK
{
    /* channel frame */
public:
  const  int24_t tdc_angle;
         int24_t tdc_angle_actual;
  const  int24_t angle_offset_recalc; 
  const uint24_t dwell_time_min;
  const uint24_t dwell_time_max;
  const uint24_t multi_on_time;
  const uint24_t multi_off_time;
  const struct SINGLE_SPARK *p_single_spark_first;
  const uint8_t  spark_count;
        struct SINGLE_SPARK *p_single_spark;
        uint8_t  spark_counter;
        uint24_t pulse_start_time;
        uint24_t dwell_time_applied;
        uint24_t dwell_time;
         int24_t end_angle;
        uint8_t  multi_pulse_count;
        uint8_t  multi_pulse_counter;
        uint8_t  state; 
        uint8_t  error; 
  const uint8_t  generation_disable; 
         int24_t angle_offset_recalc_working;
         _Bool   is_first_recalc;


    /************************************/
    
    /* threads */
    
    _eTPU_thread INIT(_eTPU_matches_disabled);
    _eTPU_thread UPDATE(_eTPU_matches_disabled);
    _eTPU_thread RECALC_ANGLE(_eTPU_matches_disabled);
    _eTPU_thread START_ANGLE(_eTPU_matches_disabled);
    _eTPU_thread MIN_DWELL_TIME(_eTPU_matches_disabled);
    _eTPU_thread END_ANGLE(_eTPU_matches_disabled);
    _eTPU_thread MAX_DWELL_TIME(_eTPU_matches_disabled);
    _eTPU_thread MULTI_PULSE(_eTPU_matches_disabled);
    _eTPU_thread LINK_OR_ERROR(_eTPU_matches_disabled);

    
    /************************************/
    
    /* methods and fragments */
    
    _eTPU_fragment Init_NoReturn(void);
    _eTPU_fragment ScheduleNextRecalcAngle_NoReturn(void);
    _eTPU_fragment ScheduleRecalcAngle_NoReturn(void);
    _eTPU_fragment ScheduleStartAngle_NoReturn(void);
    _eTPU_fragment ScheduleMinDwellTime_NoReturn(void);
    _eTPU_fragment ScheduleEndAngleAndMaxDwellTime_NoReturn(void);
    _eTPU_fragment ScheduleMultiPulse_NoReturn(void);
    void ReadSparkParams(void);
    
    
    /************************************/
    
    /* entry table */
    
    _eTPU_entry_table SPARK;
};

#endif /* __ETPUC_SPARK_H */
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
*  Revision 0.1  2013/09/12  r54529
*  Initial version.
*
*******************************************************************************/
