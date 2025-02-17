/*******************************************************************************
*
*  FREESCALE SEMICONDUCTOR INC. 2008-2014
*  ALL RIGHTS RESERVED
*  COPYRIGHT (c)
*
********************************************************************************
*
*  FILE NAME:  etpuc_fuel.h
*
*******************************************************************************/
#ifndef __ETPUC_FUEL_H
#define __ETPUC_FUEL_H

/*******************************************************************************
*  Definitions
*******************************************************************************/
/* Host Service Requests */
#define FUEL_HSR_INIT                  7
#define FUEL_HSR_STOP                  5
#define FUEL_HSR_UPDATE                3

/* Function Modes */
#define FUEL_FM0_ACTIVE_LOW            0
#define FUEL_FM0_ACTIVE_HIGH           1

/* Channel Flags */
#define FUEL_FLAG0_INJ_NOT_ACTIVE      0
#define FUEL_FLAG0_INJ_ACTIVE          1
#define FUEL_FLAG1_RECALC_ANGLE        0
#define FUEL_FLAG1_STOP_ANGLE          1

/* Error Flags */
#define FUEL_ERROR_STOP_ANGLE_APPLIED        0x01
#define FUEL_ERROR_MINIMUM_INJ_TIME_APPLIED  0x02

/* Generation Disable flags */
#define FUEL_GENERATION_ALLOWED        0
#define FUEL_GENERATION_DISABLED       1


/* FUEL eTPU function class declaration */
_eTPU_class FUEL
{
    /* channel frame */
public:
  const  int24_t tdc_angle;
         int24_t tdc_angle_actual;
  const  int24_t angle_normal_end; 
  const  int24_t angle_stop;
  const  int24_t angle_offset_recalc; 
  const  int24_t injection_time; 
  const  int24_t compensation_time;
  const  int24_t injection_time_minimum;
  const  int24_t off_time_minimum;
         int24_t injection_time_applied;
         int24_t injection_time_applied_cpu;
         int24_t injection_start_angle;
         int24_t injection_start_angle_cpu;
         int24_t pulse_start_time;
         int24_t pulse_end_time;
  const  uint8_t generation_disable; 
         uint8_t error; 
         int24_t angle_stop_actual_last;
         int24_t angle_offset_recalc_working;
         _Bool   is_await_recalc;
         _Bool   is_first_recalc;


    /************************************/
    
    /* threads */
    
    _eTPU_thread INIT(_eTPU_matches_disabled);
    _eTPU_thread STOP(_eTPU_matches_disabled);
    _eTPU_thread UPDATE_INACTIVE(_eTPU_matches_disabled);
    _eTPU_thread UPDATE_ACTIVE(_eTPU_matches_disabled);
    _eTPU_thread PULSE_START(_eTPU_matches_disabled);
    _eTPU_thread PULSE_END(_eTPU_matches_disabled);
    _eTPU_thread STOP_ANGLE_INACTIVE(_eTPU_matches_disabled);
    _eTPU_thread STOP_ANGLE_ACTIVE(_eTPU_matches_disabled);
    _eTPU_thread RECALC_ANGLE(_eTPU_matches_disabled);

    
    /************************************/
    
    /* methods and fragments */
    
    _eTPU_fragment Init_NoReturn(void);
    _eTPU_fragment OnRecalcAngle_NoReturn(void);
    _eTPU_fragment OnStopAngle_NoReturn(void);
    _eTPU_fragment ScheduleRecalc_NoReturn(void);
    _eTPU_fragment SchedulePulseEnd_NoReturn(void);
    _eTPU_fragment ScheduleAdditionalPulse_NoReturn(void);
    void OnPulseEnd(void);
    
    
    /************************************/
    
    /* entry table */
    
    _eTPU_entry_table FUEL;
};


#endif /* __ETPUC_FUEL_H */
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
*  Revision 0.3  2013/10/30  r54529
*  All time variables are signed int24_t.
*
*  Revision 0.2  2013/09/05  r54529
*  Generation disable added + minor updates.
*
*  Revision 0.1  2013/08/27  r54529
*  Initial version.
*
*******************************************************************************/
