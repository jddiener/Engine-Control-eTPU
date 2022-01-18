/*******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2014 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
****************************************************************************//*!
*
* @file    etpu_spark.h
*
* @author  Milan Brejl [r54529]
*
* @version 1.0
*
* @date    17-Mar-2014
*
* @brief   This file contains useful macros and prototypes for SPARK API.
*
*******************************************************************************/
#ifndef _ETPU_SPARK_H_
#define _ETPU_SPARK_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include "etpu_util.h"        /* 24-bit types */
#include "etpu_spark_auto.h"  /* auto generated header file */

/*******************************************************************************
* Definitions
*******************************************************************************/

/*******************************************************************************
* Type Definitions
*******************************************************************************/
/** A structure to represent an instance of SPARK.
 *  It includes static SPARK initialization items. */
struct spark_instance_t
{
  const uint8_t  chan_num;  /**< Channel number of the SPARK channel. */
  const uint8_t  priority;  /**< Channel priority for the SPARK channel. */
  const uint8_t  polarity;  /**< SPARK polarity. It can one any of:
    - @ref FS_ETPU_SPARK_FM0_ACTIVE_HIGH - active output signal state is high
    - @ref FS_ETPU_SPARK_FM0_ACTIVE_LOW - active output signal state is low */
  const uint24_t  tdc_angle; /**< The cylinder Top Dead Center as a number
    of TCR2 angle ticks relative to zero engine angle, in a range corresponding
    to 0-720 degrees. */
        uint32_t *cpba;     /**< Channel parameter base address.
    Set cpba = 0 to use automatic allocation of eTPU DATA RAM for SPARK channel
    parameters using the eTPU utility function fs_etpu_malloc (recommanded),
    or assign the cpba manually by an address where the SPARK channel parameter
    space will start from, e.g. 0xC3FC8100. */
        uint32_t *cpba_single_spark; /**< Base address of the single spark array
    in eTPU DATA RAM. Set cpba_single_spark = 0 to use automatic allocation of
    the eTPU DATA RAM space corresponding to the spark_count value,
    using the eTPU utility function fs_etpu_malloc (recommanded),
    or assign the cpba_sparks manually by an address, e.g. 0xC3FC8100. */
};

/** A structure to represent a configuration of SPARK.
 *  It includes SPARK configuration items which can be changed in run-time. */
struct spark_config_t
{
   int24_t angle_offset_recalc;  /**< The recalculation offset angle as a
    number of TCR2 ticks.
    At the end of one spark the start angle of the next spark is calculated
    according to the actual engine speed and end_angle. The calculation is
    repeated once again in order to adjust to a speed change at a defined
    angular position before the originaly calculated start_angle. This position
    is given by angle_offset_recalc. */
  uint24_t dwell_time_min;  /**< The minimum spark dwell time as a number of
    TCR1 ticks. */
  uint24_t dwell_time_max;  /**< The maximum spark dwell time as a number of
    TCR1 ticks. */
  uint24_t multi_on_time;  /**< The multi-pulse ON-time as a number of TCR1
    ticks. */
  uint24_t multi_off_time;  /**< The multi-pulse OFF-time as a number of TCR1
    ticks. */
  uint8_t  spark_count;  /**< The count of single sparks per engine-cycle. */
  struct single_spark_config_t *p_single_spark_config; /**< Pointer to the first
    item of an array of the single spark configuration structures. */
  uint8_t  generation_disable;     /**< This parameter disables/enables
    the generation of output pulses. It can be assigned one of the values:
    - @ref FS_ETPU_SPARK_GENERATION_ALLOWED
    - @ref FS_ETPU_SPARK_GENERATION_DISABLED */
};

/** A structure to represent a single spark configuration. */
struct single_spark_config_t
{
   int24_t end_angle;  /**< The tdc_angle-relative stark main pulse end angle
    as a number of TCR2 ticks.
    Positive values precede the TDC, negative go after.
    The spark start angle is calculated, based on the currecnt engine speed,
    so that the spark main pulse ends at the end_angle. */
  uint24_t dwell_time;  /**< The spark dwell time as a number of TCR1 ticks. */
  uint8_t  multi_pulse_count;  /**< The count of multi-pulses following the
    main spark pulse. */
};

/** A structure to represent states of SPARK. */
struct spark_states_t
{
        uint8_t  error;     /**< This is the error status of SPARK. It may
    include the following error flags:
    - @ref FS_ETPU_SPARK_ERROR_MIN_DWELL_APPLIED
    - @ref FS_ETPU_SPARK_ERROR_MAX_DWELL_APPLIED
    The eTPU sets the error flags, the CPU clears them after reading. */
        uint24_t dwell_time_applied;  /**< This is the last spark dwell-time
    actually generated. The value corresponds to commanded dwell_time, but
    it may slightly differ in case of rapid acceleration or deceleration. */
};

/*******************************************************************************
* Function prototypes
*******************************************************************************/
/* Initialize */
uint32_t fs_etpu_spark_init(
  struct spark_instance_t *p_spark_instance,
  struct spark_config_t   *p_spark_config);

/* Change configuration */
uint32_t fs_etpu_spark_config(
  struct spark_instance_t *p_spark_instance,
  struct spark_config_t   *p_spark_config);

  /* Get states */
uint32_t fs_etpu_spark_get_states(
  struct spark_instance_t *p_spark_instance,
  struct spark_states_t   *p_spark_states);

#endif /* _ETPU_SPARK_H_ */
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
 * Revision 0.1  2013/09/17  r54529
 * Initial version of file.
 ******************************************************************************/