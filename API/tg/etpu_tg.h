/*******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2014 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
****************************************************************************//*!
*
* @file    etpu_tg.h
*
* @author  Milan Brejl [r54529]
*
* @version 1.0
*
* @date    17-Mar-2014
*
* @brief   This file contains useful macros and prototypes for TG API.
*
*******************************************************************************/
#ifndef _ETPU_TG_H_
#define _ETPU_TG_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include "etpu_util.h"        /* 24-bit types */
#include "etpu_tg_auto.h"     /* auto generated header file */

/*******************************************************************************
* Definitions
*******************************************************************************/

/*******************************************************************************
* Type Definitions
*******************************************************************************/
/** A structure to represent an instance of TG.
 *  It includes static TG initialization items. */
struct tg_instance_t
{
  const uint8_t  chan_num_crank; /**< Channel number of the TG Crank channel. */
  const uint8_t  chan_num_cam;   /**< Channel number of the TG Cam channel. */
  const uint8_t  priority;  /**< Channel priority for the TG channel. */
  const uint8_t  polarity_crank;  /**< Initial Crank output polarity.
    It can be one of:
    - @ref FS_ETPU_TG_FM0_POLARITY_LOW
    - @ref FS_ETPU_TG_FM0_POLARITY_HIGH */
  const uint8_t  polarity_cam;  /**< Initial cam output polarity.
    It can be one of:
    - @ref FS_ETPU_TG_FM0_POLARITY_LOW
    - @ref FS_ETPU_TG_FM0_POLARITY_HIGH */
  const uint8_t  teeth_till_gap; /**< A number of physical teeth between 2 gaps.
    This is the number of physical teeth on the crank wheel divided by
    the number of (equally spaced) gaps on the wheel. */
  const uint8_t  teeth_in_gap; /**< A number of missing teeth in one gap.
    This should be assigned a value of 1, 2 or 3. */
  const uint8_t  teeth_per_cycle; /**< A number of teeth (including missing
    teeth in gap) per an engine cycle (720 degrees). It must be a multiple of
    (teeth_till_gap + teeth_in_gap). */
  const uint8_t  cam_edge_count;  /**< The count of Cam edges within an engine
    cycle.*/
  const uint8_t  *p_cam_edge_tooth; /**< Pointer to the first item of an array
    of tooth numbers where the cam signal toggles */
        uint32_t *cpba;     /**< Channel parameter base address.
    Set cpba = 0 to use automatic allocation of eTPU DATA RAM for TG channel
    parameters using the eTPU utility function fs_etpu_malloc (recommanded),
    or assign the cpba manually by an address where the TG channel parameter
    space will start from, e.g. 0xC3FC8100. */
        uint8_t  *cpba8_cam_edge_tooth; /**< Base address of the array of tooth
    numbers where the cam signal toggles in eTPU DATA RAM.
    Set cpba_cam_edge_tooth = 0 to use automatic allocation of the
    eTPU DATA RAM space corresponding to the cam_edge_count value,
    using the eTPU utility function fs_etpu_malloc (recommanded),
    or assign the cpba_injections manually by an address, e.g. 0xC3FC8100. */
};

/** A structure to represent a configuration of TG.
 *  It includes TG configuration items which can be changed in
 *  run-time. */
struct tg_config_t
{
    int24_t tooth_period_target;  /**< TG target Crank tooth period
    as a number of TCR1 ticks. */
  fract24_t accel_ratio;  /**< A signed fractional value
    (values 0 to 0x7FFFFF corresponds to 0 - 1.0) defining an exponencial
    acceleration/deceleration profile. On each tooth, the tooth_period_actual
    is updated by:
        accel_ratio * (tooth_period_target - tooth_period_actual)  */
   uint8_t  generation_disable;     /**< This parameter disables/enables
    the generation of crank output signal. It can be assigned one of the values:
    - @ref FS_ETPU_TG_GENERATION_ALLOWED
    - @ref FS_ETPU_TG_GENERATION_DISABLED */
};

/** A structure to represent internal states of TG. */
struct tg_states_t
{
  uint8_t  tooth_counter_cycle;  /**< This is the actual value of the tooth
    counter which counts from 1 to teeth_per_cycle. */
   int24_t tooth_period_actual; /**< TG actual Crank tooth period
    as a number of TCR1 ticks. */
};

/*******************************************************************************
* Function prototypes
*******************************************************************************/
/* Initialize */
uint32_t fs_etpu_tg_init(
  struct tg_instance_t *p_tg_instance,
  struct tg_config_t   *p_tg_config);

/* Change configuration */
uint32_t fs_etpu_tg_config(
  struct tg_instance_t *p_tg_instance,
  struct tg_config_t   *p_tg_config);

/* Get states */
uint32_t fs_etpu_tg_get_states(
  struct tg_instance_t *p_tg_instance,
  struct tg_states_t   *p_tg_states);


#endif /* _ETPU_TG_H_ */
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
 * Revision 1.0  2014/03/17  r54529
 * Minor comment and formating improvements.
 * Ready for eTPU Engine Control Library release 1.0.
 *
 * Revision 0.4  2013/10/22  r54529
 * generation_disable switch added.
 *
 * Revision 0.3  2013/07/24  r54529
 * Separate initial polarity for Crank and Cam.
 *
 * Revision 0.2  2013/06/20  r54529
 * Acceleration/deceleration added.
 *
 * Revision 0.1  2012/12/28  r54529
 * Initial version of file.
 ******************************************************************************/