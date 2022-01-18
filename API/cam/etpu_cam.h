/*******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2014 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
****************************************************************************//*!
*
* @file    etpu_cam.h
*
* @author  Milan Brejl [r54529]
*
* @version 1.0
*
* @date    16-Mar-2014
*
* @brief   This file contains useful macros and prototypes for CAM API.
*
*******************************************************************************/
#ifndef _ETPU_CAM_H_
#define _ETPU_CAM_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include "etpu_util.h"        /* 24-bit types */
#include "etpu_cam_auto.h"    /* auto generated header file */

/*******************************************************************************
* Definitions
*******************************************************************************/
/** CAM Mode */
#define FS_ETPU_CAM_LOG_FALLING   FS_ETPU_CAM_FM0_LOG_FALLING
#define FS_ETPU_CAM_LOG_RISING    FS_ETPU_CAM_FM1_LOG_RISING
#define FS_ETPU_CAM_LOG_BOTH     (FS_ETPU_CAM_FM0_LOG_FALLING + FS_ETPU_CAM_FM1_LOG_RISING)

/*******************************************************************************
* Type Definitions
*******************************************************************************/
/** A structure to represent an instance of CAM.
 *  It includes static CAM initialization items. */
struct cam_instance_t
{
  const uint8_t  chan_num;  /**< Channel number of the CAM channel. */
  const uint8_t  priority;  /**< Channel priority for the CAM channel. */
  const uint8_t  log_size;  /**< CAM log array size as a number of logged
                                 items. */
        uint32_t *cpba;     /**< Channel parameter base address.
    Set cpba = 0 to use automatic allocation of eTPU DATA RAM for CAM channel
    parameters using the eTPU utility function fs_etpu_malloc (recommanded),
    or assign the cpba manually by an address where the CAM channel parameter
    space will start from, e.g. 0xC3FC8100. */
        uint32_t *cpba_log; /**< Pointer to the CAM log buffer in eTPU DATA RAM.
    Set cpba_log = 0 to use automatic allocation of eTPU DATA RAM for the CAM
    buffer using the eTPU utility function fs_etpu_malloc (recommanded), or
    assign the cpba_log manually by an address where the CAM buffer will start.*/
};

/** A structure to represent a configuration of CAM.
 *  It includes CAM configuration items which can be changed in
 *  run-time. */
struct cam_config_t
{
        uint8_t  mode;      /**< CAM mode.
    It defines which type of transitions are logged. It can be one of:
    - @ref FS_ETPU_CAM_LOG_FALLING
    - @ref FS_ETPU_CAM_LOG_RISING
    - @ref FS_ETPU_CAM_LOG_BOTH */
};

/** A structure to represent internal states of CAM. */
struct cam_states_t
{
        uint8_t error;      /**< This is the error status of CAM. It includes
    the following error flags:
    - @ref FS_ETPU_CAM_ERROR_ZERO_TRANS
    - @ref FS_ETPU_CAM_ERROR_LOG_OVERFLOW
    The eTPU sets the error flags, the CPU clears them after reading. */
        uint8_t log_count; /**< This is a count of transitions logged during
    the last completed engine cycle. */
        uint8_t log_idx; /**< This is an index of the first free position
    in the log buffer, starting at 0. log_idx is copied to log_count before
    resetting. */
};

/*******************************************************************************
* Function prototypes
*******************************************************************************/
/* Initialize */
uint32_t fs_etpu_cam_init(
  struct cam_instance_t *p_cam_instance,
  struct cam_config_t   *p_cam_config);

/* Change configuration */
uint32_t fs_etpu_cam_config(
  struct cam_instance_t *p_cam_instance,
  struct cam_config_t   *p_cam_config);

/* Get states */
uint32_t fs_etpu_cam_get_states(
  struct cam_instance_t *p_cam_instance,
  struct cam_states_t   *p_cam_states);

/* Copy log */
uint32_t *fs_etpu_cam_copy_log(
  struct cam_instance_t *p_cam_instance,
               uint32_t *p_cam_log);

/* Reset log */
uint32_t fs_etpu_cam_reset_log(
  struct cam_instance_t *p_cam_instance);


#endif /* _ETPU_CAM_H_ */
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
 * Revision 1.0  2014/03/16  r54529
 * Minor comment and formating improvements.
 * Ready for eTPU Engine Control Library release 1.0.
 *
 * Revision 0.2  2014/01/23  r54529
 * fs_etpu_cam_reset_log() added.
 *
 * Revision 0.1  2012/06/13  r54529
 * Initial version of file.
 ******************************************************************************/