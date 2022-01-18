/*******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2015 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
****************************************************************************//*!
*
* @file    etpu_crank.h
*
* @author  Milan Brejl [r54529]
*
* @version 1.2
*
* @date    01-Sep-2015
*
* @brief   This file contains useful macros and prototypes for CRANK API.
*
*******************************************************************************/
#ifndef _ETPU_CRANK_H_
#define _ETPU_CRANK_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include "etpu_util.h"        /* 24-bit types */
#include "etpu_crank_auto.h"  /* auto generated header file */

/*******************************************************************************
* Type Definitions
*******************************************************************************/
/** A structure to represent an instance of CRANK.
 *  It includes static CRANK initialization items. */
struct crank_instance_t
{
  const uint8_t  chan_num;  /**< Channel number of the CRANK channel. */
  const uint8_t  priority;  /**< Channel priority for the CRANK channel. */
  const uint8_t  polarity;  /**< CRANK transition polarity.
    It defines which type of transitions are captured. It can be one of:
    - @ref FS_ETPU_CRANK_FM0_USE_TRANS_RISING
    - @ref FS_ETPU_CRANK_FM0_USE_TRANS_FALLING */
  const uint8_t  teeth_till_gap; /**< A number of physical teeth between 2 gaps.
    This is the number of physical teeth on the crank wheel divided by
    the number of (equally spaced) gaps on the wheel. */
  const uint8_t  teeth_in_gap; /**< A number of missing teeth in one gap.
    This can be assigned a value of 1, 2, 3 (for eTPU and eTPU2), up to 7
    (for eTPU2+), or 0. The value 0 indicates there is an additional tooth
    instead of missing teeth. */
  const uint8_t  teeth_per_cycle; /**< A number of teeth (including missing
    teeth in gap) per an engine cycle (720 degrees). It must be a multiple of
    (teeth_till_gap + teeth_in_gap). */
  const uint32_t tcr1_clock_source; /**< The TCR1 clock source for this eTPU
    engine. This affects how the TRR register value needs to be calculated. Set to:
    - @ref FS_ETPU_TCR1CS_DIV1
    - @ref FS_ETPU_TCR1CS_DIV2 */
  const uint24_t tcr2_ticks_per_tooth; /**< A number of TCR2 angle ticks per one
    tooth. It can be any value between 1 and 1024. */
  const uint24_t tcr2_ticks_per_add_tooth; /**< A number of TCR2 angle ticks
    from the last tooth to the additional tooth. It can be any value between 
    1 and 1024. This parameter only applies for crank pattern with an additional
    tooth (teeth_in_gap=0). */
  const uint8_t  log_tooth_periods; /**< An option to record tooth_periods.
    It can be one of:
    - @ref FS_ETPU_CRANK_FM1_TOOTH_PERIODS_LOG_OFF
    - @ref FS_ETPU_CRANK_FM1_TOOTH_PERIODS_LOG_ON */
  const uint32_t link_cam;  /**< Set of 4 link numbers to send to reset the Cam
    log. Up to 4 Cam channel numbers can be used. In case of a single Cam on
    channel 1, use 0x01010101. */
  const uint32_t link_1;    /**< The first  set of 4 link numbers to send when
    stall conditions accure. */
  const uint32_t link_2;    /**< The second set of 4 link numbers to send when
    stall conditions accure. */
  const uint32_t link_3;    /**< The third  set of 4 link numbers to send when
    stall conditions accure. */
  const uint32_t link_4;    /**< The fourth set of 4 link numbers to send when
    stall conditions accure. */
        uint32_t *cpba;     /**< Channel parameter base address.
    Set cpba = 0 to use automatic allocation of eTPU DATA RAM for CRANK channel
    parameters using the eTPU utility function fs_etpu_malloc (recommanded),
    or assign the cpba manually by an address where the CRANK channel parameter
    space will start from, e.g. 0xC3FC8100. */
        uint32_t *cpba_tooth_period_log; /**< Base address of the CRANK tooth
    period log buffer in eTPU DATA RAM. Set cpba_tooth_period_log = 0 to use
    automatic allocation of eTPU DATA RAM for this buffer using the eTPU utility
    function fs_etpu_malloc (recommanded), or assign the p_tooth_period_log
    manually by an address where the CRANK buffer will start. The memory does
    not need to be allocated if FS_ETPU_CRANK_FM1_TOOTH_PERIODS_LOG_OFF is set.*/
};

/** A structure to represent a configuration of CRANK.
 *  It includes CRANK configuration items which can be changed in
 *  run-time. */
struct crank_config_t
{
        uint8_t  teeth_per_sync; /**< A number of teeth (including the
    missing teeth in gap) corresponding to a segment which is needed for the Cam
    to log enough Cam transitions so that it enables to recognize the correct
    engine half-cycle and achieve synchronozation. It must be a multiple of
    (teeth_till_gap + teeth_in_gap). */
        uint24_t blank_time; /**< A TCR1 time period after initialization during
    which teeth are ignored. */
        uint8_t  blank_teeth; /**< A number of teeth ignored after
    initialization. */
        ufract24_t gap_ratio; /**< A fraction used to perform the ABA gap test.
    For a crank wheel with a gap:
      gap_ratio * tooth_period_B > tooth_period_A.
    For a crank wheel with an additional tooth:
      gap_ratio * tooth_period_A > tooth_period_B. */
        ufract24_t win_ratio_normal; /**< A fraction used to derive
    the acceptance window for the next normal tooth. */
        ufract24_t win_ratio_across_gap; /**< A fraction used to derive
    the acceptance window for the first tooth after the gap. */
        ufract24_t win_ratio_after_gap; /**< A fraction used to derive
    the acceptance window for the second tooth after the gap. */
        ufract24_t win_ratio_after_timeout; /**< A fraction used to derive
    the acceptance window for the tooth following a timeout condition. */
        uint24_t first_tooth_timeout; /**< A TCR1 time period after the first
    tooth (after blank_teeth) when a timeout will be deemed to have happened. */
};

/** A structure to represent internal states of CRANK. */
struct crank_states_t
{
        uint8_t error;      /**< This is the error status of CRANK. It includes
    the following error flags:
    - @ref FS_ETPU_CRANK_ERR_INVALID_TRANS
    - @ref FS_ETPU_CRANK_ERR_INVALID_MATCH
    - @ref FS_ETPU_CRANK_ERR_TIMEOUT
    - @ref FS_ETPU_CRANK_ERR_STALL
    - @ref FS_ETPU_CRANK_ERR_INTERNAL
    - @ref FS_ETPU_CRANK_ERR_TIMEOUT_BEFORE_GAP
    - @ref FS_ETPU_CRANK_ERR_TIMEOUT_AFTER_GAP
    - @ref FS_ETPU_CRANK_ERR_TOOTH_IN_GAP
    The eTPU sets the error flags, the CPU clears them after reading. */
        uint8_t state; /**< This is the state of CRANK. It can be one of:
    - @ref FS_ETPU_CRANK_SEEK
    - @ref FS_ETPU_CRANK_BLANK_TIME
    - @ref FS_ETPU_CRANK_BLANK_TEETH
    - @ref FS_ETPU_CRANK_FIRST_TRANS
    - @ref FS_ETPU_CRANK_SECOND_TRANS
    - @ref FS_ETPU_CRANK_TEST_POSSIBLE_GAP
    - @ref FS_ETPU_CRANK_VERIFY_GAP
    - @ref FS_ETPU_CRANK_COUNTING
    - @ref FS_ETPU_CRANK_COUNTING_TIMEOUT
    - @ref FS_ETPU_CRANK_TOOTH_BEFORE_GAP
    - @ref FS_ETPU_CRANK_TOOTH_BEFORE_GAP_NOT_HRM
      @ref FS_ETPU_CRANK_ADDITIONAL_TOOTH
    - @ref FS_ETPU_CRANK_TOOTH_AFTER_GAP  */
        uint8_t eng_pos_state; /**< This is the global engine position state.
    It can be one of:
    - @ref FS_ETPU_ENG_POS_SEEK
    - @ref FS_ETPU_ENG_POS_FIRST_HALF_SYNC
    - @ref FS_ETPU_ENG_POS_PRE_FULL_SYNC
    - @ref FS_ETPU_ENG_POS_FULL_SYNC  */
        uint8_t tooth_counter_gap; /**< The actual number of the tooth counter
    which counts from 1 to teeth_till_gap and resets on every gap. */
        uint8_t tooth_counter_cycle; /**< The actual number of the tooth counter
    which counts from 1 to teeth_per_cycle and resets every engine cycle. */
       uint24_t last_tooth_period; /**< The last tooth period as a number
    of TCR1 ticks. */
       uint24_t last_tooth_period_norm; /**< The last tooth period normalized
    over the gap or over the additional tooth as a number of TCR1 ticks. */
};

/*******************************************************************************
* Function prototypes
*******************************************************************************/
/* Initialize */
uint32_t fs_etpu_crank_init(
  struct crank_instance_t *p_crank_instance,
  struct crank_config_t   *p_crank_config);

/* Change configuration */
uint32_t fs_etpu_crank_config(
  struct crank_instance_t *p_crank_instance,
  struct crank_config_t   *p_crank_config);

/* Get states */
uint32_t fs_etpu_crank_get_states(
  struct crank_instance_t *p_crank_instance,
  struct crank_states_t   *p_crank_states);

/* Set synchronization */
uint32_t fs_etpu_crank_set_sync(
  struct crank_instance_t *p_crank_instance,
                 uint24_t tcr2_adjustment);

/* Copy tooth_period_log */
uint24_t *fs_etpu_crank_copy_tooth_period_log(
  struct crank_instance_t *p_crank_instance,
                 uint24_t *p_tooth_period_log);

/* Get resetting engine angle */
uint32_t fs_etpu_crank_get_angle_reseting(void);


#endif /* _ETPU_CRANK_H_ */
/*******************************************************************************
 *
 * Copyright:
 *  Freescale Semiconductor, INC. All Rights Reserved.
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
 ******************************************************************************/
/*******************************************************************************
 *
 * REVISION HISTORY:
 *
 * FILE OWNER: Milan Brejl [r54529]
 *
 * Revision 1.2  2015/09/01  r54529
 * Parameter crank_states_t.last_tooth_period_norm added.
 * 
 * Revision 1.1  2015/06/29  r54529
 * Parameter crank_instance_t.tcr2_ticks_per_add_tooth added.
 * Function fs_etpu_crank_get_angle_reseting added.
 *
 * Revision 1.0  2014/03/25  r54529
 * Minor comment and formating improvements.
 * Ready for eTPU Engine Control Library release 1.0.
 *
 * Revision 0.3  2013/11/28  r54529
 * Support of up to 7 missing teeth on eTPU2+.
 *
 * Revision 0.2  2012/11/28  r54529
 * Revision of instance and config structures.
 *
 * Revision 0.1  2012/06/13  r54529
 * Initial version of file.
 ******************************************************************************/
