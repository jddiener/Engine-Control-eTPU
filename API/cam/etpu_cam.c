/*******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2014 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
****************************************************************************//*!
*
* @file    etpu_cam.c
*
* @author  Milan Brejl [r54529]
*
* @version 1.0
*
* @date    16-Mar-2014
*
* @brief   This file contains API for using the eTPU function
*          Cam (CAM).
*
****************************************************************************//*!
*
* @mainpage
*
* The eTPU CAM APIs @ref etpu_cam.c/.h includes API functions for eTPU
* function CAM, typically used together with CRANK in an engine position system.
*
* The CAM eTPU function uses 1 eTPU channel to log input signal transitions.
* More instances of CAM can be initialized to independently log several inputs.
*
* Features:
* - Based on the selected mode, either raising, falling or both signal
*   transitions are detected and logged.
* - The log buffer size is configurable.
* - The log can be reset automatically by a link from Crank eTPU function or
*   manually by the CPU application. Reseting the log means setting the buffer
*   index to the buffer start position so that the next transition overwrites
*   the first value in the buffer.
* - Number of transitions logged during the last engine cycle (between last
*   two log resets) and the actual position in the log buffer are available
*   to read.
* - 2 error conditions are reported:
*   - @ref FS_ETPU_CAM_ERROR_ZERO_TRANS – no input transition was logged during
*     the last engine cycle (between last two leg resets).
*     It might mean the cam signal is lost.
*   - @ref FS_ETPU_CAM_ERROR_LOG_OVERFLOW – the log buffer size is not big
*     enought to log all input transitions. The last transition was not logged.
* - Channel interrupt is generated when an error condition is detected.
*
* A single item in the log buffer is a 32-bit word which includes:
* - transition TCR2 angle in the lower 24 bits,
* - transition polarity (0-falling, 1-rising) in upper 8 bits.
*
*******************************************************************************/
/*******************************************************************************
* Includes
*******************************************************************************/
#include "etpu_cam.h"     /* private header file */
#include "etpu_util.h"    /* utility routines for working with the eTPU */

/*******************************************************************************
* Global variables
*******************************************************************************/
extern uint32_t fs_etpu_data_ram_start;
extern uint32_t fs_etpu_data_ram_ext;

/*******************************************************************************
* FUNCTION: fs_etpu_cam_init
****************************************************************************//*!
* @brief   This function initializes eTPU channels to run CAM function.
*
* @note    The following actions are performed in order:
*          -# Use user-defined CPBA or allocate new eTPU DATA RAM
*          -# Write chan config registers and FM bits
*          -# Write channel parameters
*          -# Write HSR
*          -# Set channel priority
*
* @param   *p_cam_instance - This is a pointer to the instance structure
*            @ref cam_instance_t.
* @param   *p_cam_config - This is a pointer to the structure of configuration
*            parameters @ref cam_config_t.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_MALLOC - eTPU DATA RAM memory allocation error
*          - @ref FS_ETPU_ERROR_NONE - No error
*
* @warning This function does not configure the pins, only the eTPU channels.
*******************************************************************************/
uint32_t fs_etpu_cam_init(
  struct cam_instance_t *p_cam_instance,
  struct cam_config_t   *p_cam_config)
{
  uint8_t  chan_num;
  uint8_t  priority;
  uint8_t  log_size;
  uint32_t *cpba_log;
  uint32_t *cpba;

  chan_num = p_cam_instance->chan_num;
  priority = p_cam_instance->priority;
  log_size = p_cam_instance->log_size;
  cpba_log = p_cam_instance->cpba_log;
  cpba     = p_cam_instance->cpba;

  /* Use user-defined CPBA or allocate new eTPU DATA RAM */
  if(cpba == 0)
  {
    cpba = fs_etpu_malloc(FS_ETPU_CAM_NUM_PARMS);
    if(cpba == 0)
    {
      return(FS_ETPU_ERROR_MALLOC);
    }
    else
    {
      p_cam_instance->cpba = cpba;
    }
  }

  /* Use user-defined log or allocate new eTPU DATA RAM */
  if(cpba_log == 0)
  {
    cpba_log = fs_etpu_malloc(log_size<<2);
    if(cpba_log == 0)
    {
      return(FS_ETPU_ERROR_MALLOC);
    }
    else
    {
      p_cam_instance->cpba_log = cpba_log;
    }
  }

  /* Write chan config registers and FM bits */
  eTPU->CHAN[chan_num].CR.R =
       (FS_ETPU_CAM_TABLE_SELECT << 24) +
       (FS_ETPU_CAM_FUNCTION_NUMBER << 16) +
       (((uint32_t)cpba - fs_etpu_data_ram_start) >> 3);
  eTPU->CHAN[chan_num].SCR.R = (uint32_t)p_cam_config->mode;

  /* Write channel parameters */
  *(cpba + ((FS_ETPU_CAM_OFFSET_LOG_SIZE  - 1)>>2)) = log_size;
  *(cpba + ((FS_ETPU_CAM_OFFSET_LOG_IDX   - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_CAM_OFFSET_LOG_COUNT - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_CAM_OFFSET_LOG       - 1)>>2)) = (uint32_t)cpba_log - fs_etpu_data_ram_start;
  *((uint8_t*)cpba + FS_ETPU_CAM_OFFSET_ERROR     ) = FS_ETPU_CAM_ERROR_NO;

  /* Write HSR */
  eTPU->CHAN[chan_num].HSRR.R = FS_ETPU_CAM_HSR_INIT;

  /* Set channel priority */
  fs_etpu_enable(chan_num, priority);

  return(FS_ETPU_ERROR_NONE);
}

/*******************************************************************************
* FUNCTION: fs_etpu_cam_config
****************************************************************************//*!
* @brief   This function changes the CAM configuration.
*
* @note    The following actions are performed in order:
*          -# Write FM bits
*
* @param   *p_cam_instance - This is a pointer to the instance structure
*            @ref cam_instance_t.
* @param   *p_cam_config - This is a pointer to the structure of configuration
*            parameters @ref cam_config_t.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_NONE - No error
*
*******************************************************************************/
uint32_t fs_etpu_cam_config(
  struct cam_instance_t *p_cam_instance,
  struct cam_config_t   *p_cam_config)
{
  /* Write FM bits */
  eTPU->CHAN[p_cam_instance->chan_num].SCR.R = (uint32_t)p_cam_config->mode;

  return(FS_ETPU_ERROR_NONE);
}

/*******************************************************************************
* FUNCTION: fs_etpu_cam_get_states
****************************************************************************//*!
* @brief   This function reads state parameter values of the CAM function.
*
* @note    The following actions are performed in order:
*          -# Read output parameter values from eTPU DATA RAM
*          -# Clear CAM error flags in eTPU DATA RAM
*
* @param   *p_cam_instance - This is a pointer to the instance structure
*            @ref cam_instance_t.
* @param   *p_cam_states - This is a pointer to the structure of states
*            @ref cam_states_t.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_NONE - No error
*
*******************************************************************************/
uint32_t fs_etpu_cam_get_states(
  struct cam_instance_t *p_cam_instance,
  struct cam_states_t   *p_cam_states)
{
  uint32_t *cpba;

  cpba = p_cam_instance->cpba;

  /* Read channel parameters */
  p_cam_states->log_count = *(cpba + ((FS_ETPU_CAM_OFFSET_LOG_COUNT - 1)>>2));
  p_cam_states->log_idx   = *(cpba + ((FS_ETPU_CAM_OFFSET_LOG_IDX   - 1)>>2));
  p_cam_states->error     = *((uint8_t*)cpba + FS_ETPU_CAM_OFFSET_ERROR);

  /* Clear CAM error flags */
  *((uint8_t*)cpba + FS_ETPU_CAM_OFFSET_ERROR) = 0;

  return(FS_ETPU_ERROR_NONE);
}

/*******************************************************************************
* FUNCTION: fs_etpu_cam_copy_log
****************************************************************************//*!
* @brief   This function copies the CAM log into another array in RAM.
*
* @note    The following actions are performed in order:
*          -# Copy out data from eTPU DATA RAM.
*
* @param   *p_cam_instance - This is a pointer to the instance structure
*            @ref cam_instance_t.
* @param   *p_cam_log - This is a pointer where the Cam log will be copied to.
*            The Cam log is an array of input transitions captured. Each item is
*            a 32-bit word which includes:
*            - transition polarity (0-falling, 1-rising) in upper 8 bits
*            - transition TCR2 angle in lower 24 bits.
*            The amount of data copied in bytes is 4 * p_cam_instance->log_size.
*
* @return  A pointer to a memory location just after the copied data.
*
*******************************************************************************/
uint32_t *fs_etpu_cam_copy_log(
  struct cam_instance_t *p_cam_instance,
               uint32_t *p_cam_log)
{
  uint32_t *dest;
  uint32_t *source;
  uint8_t  size;

  dest   = p_cam_log;
  source = p_cam_instance->cpba_log;
  size   = p_cam_instance->log_size;

  while(size--)
  {
    *dest++ = *source++;
  }

  return(dest);
}

/*******************************************************************************
* FUNCTION: fs_etpu_cam_reset_log
****************************************************************************//*!
* @brief   This function resets the index to the CAM log so that the next
*          detected transition is logged to the first position.
*          Before that, the internal log_count variable is set to the number
*          of logged transitions before the reset.
*          If this number is 0 the channel interrupt is raised together with
*          setting the error FS_ETPU_CAM_ERROR_ZERO_TRANS.
*
* @note    The following actions are performed in order:
*          -# Check HSR is 0
*          -# Write HSR
*
* @param   *p_cam_instance - This is a pointer to the instance structure
*            @ref cam_instance_t.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_NONE - No error.
*          - @ref FS_ETPU_ERROR_TIMING - The reset was not done because there is
*              a HSR pending on the eTPU channel.
*              It can be caused by a call of another API function which writes
*              the HSR to the same channel shortly before this function call.
*              Try to repeat the function call several microseconds later.
*
*******************************************************************************/
uint32_t fs_etpu_cam_reset_log(
  struct cam_instance_t *p_cam_instance)
{
  /* Check there is no pending HSR */
  if(eTPU->CHAN[p_cam_instance->chan_num].HSRR.R != 0)
  {
    return(FS_ETPU_ERROR_TIMING);
  }
  else
  {
    /* Write HSR to run RESET on eTPU */
    eTPU->CHAN[p_cam_instance->chan_num].HSRR.R = FS_ETPU_CAM_HSR_RESET;

    return(FS_ETPU_ERROR_NONE);
  }
}


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
 * Revision 0.1  2012/05/17  r54529
 * Initial version of file.
 ******************************************************************************/