/*******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2014 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
****************************************************************************//*!
*
* @file    etpu_fuel.h
*
* @author  Milan Brejl [r54529]
*
* @version 1.0
*
* @date    17-Mar-2014
*
* @brief   This file contains useful macros and prototypes for FUEL API.
*
*******************************************************************************/
#ifndef _ETPU_FUEL_H_
#define _ETPU_FUEL_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include "etpu_util.h"        /* 24-bit types */
#include "etpu_fuel_auto.h"   /* auto generated header file */

/*******************************************************************************
* Definitions
*******************************************************************************/

/*******************************************************************************
* Type Definitions
*******************************************************************************/
/** A structure to represent an instance of FUEL.
 *  It includes static FUEL initialization items. */
struct fuel_instance_t
{
  const uint8_t  chan_num;  /**< Channel number of the FUEL channel. */
  const uint8_t  priority;  /**< Channel priority for the FUEL channel. */
  const uint8_t  polarity;  /**< FUEL polarity. It can one any of:
    - @ref FS_ETPU_FUEL_FM0_ACTIVE_HIGH - active output signal state is high
    - @ref FS_ETPU_FUEL_FM0_ACTIVE_LOW - active output signal state is low */
  const uint24_t  tdc_angle; /**< The cylinder Top Dead Center as a number
    of TCR2 angle ticks relative to zero engine angle, in a range corresponding
    to 0-720 degrees. */
        uint32_t *cpba;     /**< Channel parameter base address.
    Set cpba = 0 to use automatic allocation of eTPU DATA RAM for FUEL channel
    parameters using the eTPU utility function fs_etpu_malloc (recommanded),
    or assign the cpba manually by an address where the FUEL channel parameter
    space will start from, e.g. 0xC3FC8100. */
};

/** A structure to represent a configuration of FUEL.
 *  It includes FUEL configuration items which can be changed in run-time. */
struct fuel_config_t
{
   int24_t angle_normal_end;  /**< The tdc_angle-relative normal end angle as a
    number of TCR2 ticks. Positive values precede the TDC, negative go after.
    The injection start angle is calculated, based on the currecnt engine speed,
    so that the injection ends at the normal_end_angle. */
   int24_t angle_stop;      /**< The tdc_angle-relative injection latest stop
    angle, or drop-dead-angle, as a number of TCR2 ticks. Positive values
    precede the TDC, negative go after.
    If the engine accelerates, the need is to keep the injection_time, so the
    injectind ends later than at the normal_end_angle, but not later than the
    stop_angle. If na injection is stopped by the stop_angle, the error flag
    @ref FS_ETPU_FUEL_ERROR_STOP_ANGLE_APPLIED is set. */
   int24_t angle_offset_recalc;  /**< The recalculation offset angle as a
    number of TCR2 ticks.
    At the stop_angle of one injection the start_angle of the next injection is
    calculated according to the actual engine speed. The calculation is repeated
    once again in order to adjust to a speed change at a defined angular
    position before the originaly calculated start_angle. This position is given
    by angle_offset_recalc. */
  uint24_t injection_time; /**< A TCR1 time determining the fuel injection
    pulse width, corresponding to the amount of fuel injected by one fuel
    injector in each engine cycle. */
  uint24_t compensation_time;  /**< A TCR1 time which is added to each fuel
    injection pulse width in order to compensate the valve openning and closing
    time. */
  uint24_t injection_time_minimum;  /**< A TCR1 minimum fuel injection pulse
    width. Pulses shorter than injection_time_minimum are not generated. */
  uint24_t off_time_minimum;  /**< A TCR1 minimum time between fuel injection
    pulses. In case there is an additional injection pulse generated because of
    injection_time update after the main pulse ended, the additional pulse does
    not start earlier than off_time_minimum after the main pulse end. This also
    applies to next additional pulses. */
  uint8_t  generation_disable;     /**< This parameter disables/enables
    the generation of output pulses. It can be assigned one of the values:
    - @ref FS_ETPU_FUEL_GENERATION_ALLOWED
    - @ref FS_ETPU_FUEL_GENERATION_DISABLED
    FS_ETPU_FUEL_GENERATION_DISABLED switches the injection pulse generation
    off. Switching off can also be done by setting the injection_time = 0, but
    that would shorten an injection pulse which is currently in progress.
    Instead, FS_ETPU_FUEL_GENERATION_DISABLED can switch off at any time, but
    a pulse which has already been started will be correctly finished.
    FS_ETPU_FUEL_GENERATION_ALLOWED switches the injection pulse generation
    on. */
};

/** A structure to represent states of FUEL. */
struct fuel_states_t
{
  uint8_t  error;     /**< This is the error status of FUEL. It may
    include the following error flags:
    - @ref FS_ETPU_FUEL_ERROR_STOP_ANGLE_APPLIED
    - @ref FS_ETPU_FUEL_ERROR_MINIMUM_INJ_TIME_APPLIED
    The eTPU sets the error flags, the CPU clears them after reading. */
  uint24_t injection_time_applied;  /**< This is the applied injection
    time of the last injection. The value corresponds to commanded
    injection_time, but it may slightly differ in case of rapid acceleration or
    late call of @ref fs_etpu_fuel_update_injection_time(). */
  int24_t injection_start_angle;  /**< This is the last injection
    tdc_angle-relative start angle as a number of TCR2 ticks. */
};

/*******************************************************************************
* Function prototypes
*******************************************************************************/
/* Initialize */
uint32_t fs_etpu_fuel_init(
  struct fuel_instance_t   *p_fuel_instance,
  struct fuel_config_t     *p_fuel_config);

/* Change configuration */
uint32_t fs_etpu_fuel_config(
  struct fuel_instance_t *p_fuel_instance,
  struct fuel_config_t   *p_fuel_config);

/* Update injection time */
uint32_t fs_etpu_fuel_update_injection_time(
  struct fuel_instance_t *p_fuel_instance,
  struct fuel_config_t   *p_fuel_config);

/* Get states */
uint32_t fs_etpu_fuel_get_states(
  struct fuel_instance_t *p_fuel_instance,
  struct fuel_states_t   *p_fuel_states);

#endif /* _ETPU_FUEL_H_ */
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
 * Revision 0.2  2013/09/05  r54529
 * Addition of generation_disable.
 *
 * Revision 0.1  2013/08/28  r54529
 * Initial version of file.
 *
 ******************************************************************************/