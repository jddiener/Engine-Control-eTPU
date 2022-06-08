/*******************************************************************************
* This file has been modified by ASH WARE Inc. to make it compatible with the
* ETEC eTPU C Compiler.
*******************************************************************************/

/*******************************************************************************
*
*  FREESCALE SEMICONDUCTOR INC. 2012-2014
*  ALL RIGHTS RESERVED
*  COPYRIGHT (c)
*
********************************************************************************
*
*  FILE NAME:  etpuc_crank.h
*
*******************************************************************************/
#ifndef __ETPUC_CRANK_H
#define __ETPUC_CRANK_H

/*******************************************************************************
*  Definitions
*******************************************************************************/
/* Need to workaround errata 2477 ? 
   - errata 2477 is present on MPC5500 devices only
   - uncomment the next line to compile code for MPC5500 devices */ 
/* #define ERRATA_2477 */

/* Host Service Requests */
#define CRANK_HSR_INIT                  7
#define CRANK_HSR_SET_SYNC              1
#define CRANK_HSR_SET_SPEED             3  /* CRANK_EMUL only */

/* Function Modes */
#define CRANK_FM0_USE_TRANS_RISING      1
#define CRANK_FM0_USE_TRANS_FALLING     0
#define CRANK_FM1_LOG_TOOTH_PERIODS     1

/* Channel Flags */
#define CRANK_FLAG0_GAP                 0
#define CRANK_FLAG0_ADDITIONAL_TOOTH    1
#define CRANK_FLAG1_NORMAL_MODE         0
#define CRANK_FLAG1_TOOTH_TCR2_SYNC     1

/* Errors */
#define CRANK_ERR_NO_ERROR              0
#define CRANK_ERR_INVALID_TRANS         1
#define CRANK_ERR_INVALID_MATCH         2
#define CRANK_ERR_TIMEOUT               4
#define CRANK_ERR_STALL                 8
#define CRANK_ERR_INTERNAL              16
#define CRANK_ERR_TIMEOUT_BEFORE_GAP    32
#define CRANK_ERR_TIMEOUT_AFTER_GAP     64
#define CRANK_ERR_TOOTH_IN_GAP          128
#define CRANK_ERR_ADD_TOOTH_NOT_FOUND   128

/* Crank Status values */
#define CRANK_SEEK                      0
#define CRANK_BLANK_TIME                1
#define CRANK_BLANK_TEETH               2
#define CRANK_FIRST_TRANS               3
#define CRANK_SECOND_TRANS              4
#define CRANK_TEST_POSSIBLE_GAP         5
#define CRANK_VERIFY_GAP                6
#define CRANK_COUNTING                  7
#define CRANK_COUNTING_TIMEOUT          8
#define CRANK_TOOTH_BEFORE_GAP          9
#define CRANK_TOOTH_BEFORE_GAP_NOT_HRM  10
#define CRANK_ADDITIONAL_TOOTH          10
#define CRANK_TOOTH_AFTER_GAP           11
#define CRANK_TOOTH_TCR2_SYNC           12

/* Global eng_pos_status values */
#define ENG_POS_SEEK                    0
#define ENG_POS_FIRST_HALF_SYNC         1
#define ENG_POS_PRE_FULL_SYNC           2
#define ENG_POS_FULL_SYNC               3


/* CRANK eTPU function class declaration */
_eTPU_class CRANK
{
    /* channel frame */
public:
    const uint24_t   blank_time; 
    const uint8_t    tcr1_clock_source_div1;
    const uint24_t   tcr2_ticks_per_tooth;
    const uint24_t   tcr2_ticks_per_add_tooth;
          uint24_t   last_tooth_tcr1_time; 
          uint24_t   last_tooth_period;
          uint24_t   last_tooth_period_norm;
          uint24_t   last_last_tooth_period_norm;
          uint24_t   additional_tooth_period;
           int24_t   tcr2_adjustment; 
    const ufract24_t gap_ratio;
    const ufract24_t win_ratio_normal;
    const ufract24_t win_ratio_across_gap;
    const ufract24_t win_ratio_after_gap;
    const ufract24_t win_ratio_after_timeout;
    const uint24_t   first_tooth_timeout; 
    const uint32_t   link_cam;
    const uint32_t   link_1;
    const uint32_t   link_2;
    const uint32_t   link_3;
    const uint32_t   link_4;
    const uint8_t    teeth_till_gap;
    const  int8_t    teeth_in_gap;
    const uint16_t   misscnt_mask;
    const uint8_t    teeth_per_cycle;
    const uint8_t    teeth_per_sync;
          uint8_t    tooth_counter_gap;
          uint8_t    tooth_counter_cycle;
          uint8_t    blank_teeth; 
          uint8_t    state;
          uint8_t    error;
    const uint24_t  *tooth_period_log;
          int24_t    tcr2_error_at_cycle_start;
#ifdef ERRATA_2477
           int24_t   err2477_tcr2_target;
#endif 


    /************************************/
    
    /* threads */
    
    /* CRANK */
    _eTPU_thread INIT(_eTPU_matches_disabled);
    _eTPU_thread ANGLE_ADJUST(_eTPU_matches_disabled);
    _eTPU_thread CRANK_WITH_GAP(_eTPU_matches_enabled);
    _eTPU_thread CRANK_WITH_ADDITIONAL_TOOTH(_eTPU_matches_enabled);
    _eTPU_thread CRANK_TOOTH_TCR2_SYNC_GAP(_eTPU_matches_enabled);
    _eTPU_thread CRANK_TOOTH_TCR2_SYNC_ADD(_eTPU_matches_enabled);
    
    /* CRANK_EMUL */
    _eTPU_thread INIT_EMUL(_eTPU_matches_disabled);
    _eTPU_thread ANGLE_ADJUST_EMUL(_eTPU_matches_disabled);
    _eTPU_thread SET_SPEED_EMUL(_eTPU_matches_disabled);
    _eTPU_thread CRANK_WITH_GAP_EMUL(_eTPU_matches_enabled);
    
    
    /************************************/
    
    /* methods and fragments */

    /* common */    
    void ToothArray_Log(register_a uint24_t tooth_period);
    void Set_TRR(register_a uint24_t tooth_period_norm);

    /* CRANK */
    _eTPU_fragment Window_NoReturn(
        register_a fract24_t win_ratio,
        register_d uint24_t tooth_period);
    _eTPU_fragment WindowAcrossGap_NoReturn(
        register_a uint24_t tooth_period);
    _eTPU_fragment WindowCloseAt_NoReturn(
        register_a uint24_t close_tcr1_time);
    _eTPU_fragment WindowClose_NoReturn(
        register_a fract24_t win_ratio,
        register_d uint24_t tooth_period);
    _eTPU_fragment Stall_NoReturn();
    _eTPU_fragment ToothTcr2Sync_NoReturn();

    
    /************************************/
    
    /* entry table */
    
    _eTPU_entry_table CRANK;
    _eTPU_entry_table CRANK_EMUL;
};


/*******************************************************************************
*  Function prototypes for global functions
*******************************************************************************/
int24_t CRANK_Time_to_Angle_HighRes( /* used by engine timing functions such as SPARK and FUEL */
    register_a uint24_t time);
int24_t CRANK_Time_to_Angle_LowRes( /* used by engine timing functions such as SPARK and FUEL */
    register_a uint24_t time);



/*******************************************************************************
*  Externs
*******************************************************************************/
/* enable usage of global variables handled by Crank */
extern       uint8_t   eng_pos_state;
extern const uint24_t  eng_cycle_tcr2_ticks;
extern       uint24_t  eng_cycle_tcr2_start;
extern       uint24_t  eng_trr_norm;


#endif /* __ETPUC_CRANK_H */
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
*  Revision 1.2  2015/09/01  r54529
*  Output parameter last_tooth_period_norm added.
*
*  Revision 1.1  2014/10/27  r54529
*  Parameter tcr2_ticks_per_add_tooth added.
*
*  Revision 1.0  2014/03/06  r54529
*  Minor comment and formating improvements. MISRA compliancy check.
*  Ready for eTPU Engine Control Library release 1.0.
*
*  Revision 0.1  2012/06/12  r54529
*  Initial version.
*
*******************************************************************************/
