/*******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2014 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
****************************************************************************//*!
*
* @file    etpu_crank_emul.c
*
* @author  Milan Brejl [r54529]
*
* @version 1.0
*
* @date    16-Mar-2014
*
* @brief   This file contains API for using the eTPU function
*          Crank Emulator (CRANK_EMUL).
*
****************************************************************************//*!
*
* @mainpage
*
* The eTPU CRANK_EMUL APIs @ref etpu_crank_emul.c/.h includes API functions for
* eTPU function Crank Emulator.
* The CRANK_EMUL eTPU function uses eTPU channel 0 (on eTPU2 optionally 
* channel 1 or 2) to generate internal TCR2 angle-base without processing any 
* input crank signal. 
* For testing porpuses in conditions when the crank signal is not available 
* the CRANK_EMUL eTPU function can be used to replace the CRANK function. 
* Crank Emulator drives the internal angle-base at a given speed and 
* consequently enables the injection, ignition and other output functions 
* to generate outputs.
*
* The API function prototypes and data structures are similar to the CRANK API,
* so that the CRANK API can be easily replaced by CRANK_EMUL API
* in an application just by replacing the header file inclusion.
*
*******************************************************************************/
/*******************************************************************************
* Includes
*******************************************************************************/
#include "etpu_crank_emul.h"   /* private header file */
#include "etpu_util.h"         /* utility routines for working with the eTPU */

/*******************************************************************************
* Global variables
*******************************************************************************/
extern uint32_t fs_etpu_data_ram_start;
extern uint32_t fs_etpu_data_ram_ext;

/*******************************************************************************
* FUNCTION: fs_etpu_crank_init
****************************************************************************//*!
* @brief   This function initializes eTPU channels to run CRANK_EMUL function.
*
* @note    The following actions are performed in order:
*          -# Use user-defined CPBA or allocate new eTPU DATA RAM
*          -# Write chan config registers and FM bits
*          -# Write channel and global parameters
*          -# Write HSR
*          -# Set channel priority
*
* @param   *p_crank_instance - This is a pointer to the instance structure
*            @ref crank_instance_t.
* @param   *p_crank_config - This is a pointer to the structure of configuration
*            parameters @ref crank_config_t.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_MALLOC - eTPU DATA RAM memory allocation error
*          - @ref FS_ETPU_ERROR_NONE - No error
*
* @warning This function does not configure the pins, only the eTPU channels.
*******************************************************************************/
uint32_t fs_etpu_crank_init(
  struct crank_instance_t *p_crank_instance,
  struct crank_config_t   *p_crank_config)
{
  uint8_t  chan_num;
  uint8_t  priority;
  uint32_t *cpba_log;
  uint32_t *cpba;

  chan_num = p_crank_instance->chan_num;
  priority = p_crank_instance->priority;
  cpba_log = p_crank_instance->cpba_tooth_period_log;
  cpba     = p_crank_instance->cpba;

  /* Use user-defined CPBA or allocate new eTPU DATA RAM */
  if(cpba == 0)
  {
    cpba = fs_etpu_malloc(FS_ETPU_CRANK_EMUL_NUM_PARMS);
    if(cpba == 0)
    {
      return(FS_ETPU_ERROR_MALLOC);
    }
    else
    {
      p_crank_instance->cpba = cpba;
    }
  }

  /* Use user-defined log or allocate new eTPU DATA RAM */
  if(cpba_log == 0
     && p_crank_instance->log_tooth_periods ==
        FS_ETPU_CRANK_FM1_TOOTH_PERIODS_LOG_ON)
  {
    cpba_log = fs_etpu_malloc(p_crank_instance->teeth_per_cycle<<2);
    if(cpba_log == 0)
    {
      return(FS_ETPU_ERROR_MALLOC);
    }
    else
    {
      p_crank_instance->cpba_tooth_period_log = cpba_log;
    }
  }

  /* Write chan config registers and FM bits */
  eTPU->CHAN[chan_num].CR.R =
       (FS_ETPU_CRANK_EMUL_TABLE_SELECT << 24) +
       (FS_ETPU_CRANK_EMUL_FUNCTION_NUMBER << 16) +
       (((uint32_t)cpba - fs_etpu_data_ram_start) >> 3);
  eTPU->CHAN[chan_num].SCR.R = (uint32_t)p_crank_instance->polarity
                               + (uint32_t)p_crank_instance->log_tooth_periods;

  /* Write channel parameters */
  /* 24-bit */
  *(cpba + ((FS_ETPU_CRANK_OFFSET_BLANK_TIME              - 1)>>2)) = p_crank_config->blank_time;
  *(cpba + ((FS_ETPU_CRANK_OFFSET_TCR2_TICKS_PER_TOOTH    - 1)>>2)) = p_crank_instance->tcr2_ticks_per_tooth;
  *(cpba + ((FS_ETPU_CRANK_OFFSET_LAST_TOOTH_TCR1_TIME    - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_CRANK_OFFSET_LAST_TOOTH_PERIOD       - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_CRANK_OFFSET_TCR2_ADJUSTMENT         - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_CRANK_OFFSET_GAP_RATIO               - 1)>>2)) = p_crank_config->gap_ratio;
  *(cpba + ((FS_ETPU_CRANK_OFFSET_WIN_RATIO_NORMAL        - 1)>>2)) = p_crank_config->win_ratio_normal;
  *(cpba + ((FS_ETPU_CRANK_OFFSET_WIN_RATIO_ACROSS_GAP    - 1)>>2)) = p_crank_config->win_ratio_across_gap;
  *(cpba + ((FS_ETPU_CRANK_OFFSET_WIN_RATIO_AFTER_GAP     - 1)>>2)) = p_crank_config->win_ratio_after_gap;
  *(cpba + ((FS_ETPU_CRANK_OFFSET_WIN_RATIO_AFTER_TIMEOUT - 1)>>2)) = p_crank_config->win_ratio_after_timeout;
  *(cpba + ((FS_ETPU_CRANK_OFFSET_FIRST_TOOTH_TIMEOUT     - 1)>>2)) = p_crank_config->first_tooth_timeout;
  *(cpba + ((FS_ETPU_CRANK_OFFSET_TOOTH_PERIOD_LOG        - 1)>>2)) = (uint32_t)cpba_log - fs_etpu_data_ram_start;
  /* 8-bit */
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_TEETH_TILL_GAP     ) = p_crank_instance->teeth_till_gap;
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_TEETH_IN_GAP       ) = p_crank_instance->teeth_in_gap;
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_TEETH_PER_CYCLE    ) = p_crank_instance->teeth_per_cycle;
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_TEETH_PER_SYNC     ) = p_crank_config->teeth_per_sync;
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_TOOTH_COUNTER_GAP  ) = 0;
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_TOOTH_COUNTER_CYCLE) = 0;
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_BLANK_TEETH        ) = p_crank_config->blank_teeth;
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_STATE              ) = FS_ETPU_CRANK_SEEK;
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_ERROR              ) = FS_ETPU_CRANK_ERR_NO_ERROR;
  /* 32-bit */
  *(cpba + (FS_ETPU_CRANK_OFFSET_LINK_CAM >>2)) = p_crank_instance->link_cam;
  *(cpba + (FS_ETPU_CRANK_OFFSET_LINK_1   >>2)) = p_crank_instance->link_1;
  *(cpba + (FS_ETPU_CRANK_OFFSET_LINK_2   >>2)) = p_crank_instance->link_2;
  *(cpba + (FS_ETPU_CRANK_OFFSET_LINK_3   >>2)) = p_crank_instance->link_3;
  *(cpba + (FS_ETPU_CRANK_OFFSET_LINK_4   >>2)) = p_crank_instance->link_4;

  /* Write global parameters */
  *((uint32_t*)fs_etpu_data_ram_start + ((FS_ETPU_OFFSET_ENG_CYCLE_TCR2_TICKS -1)>>2)) =
    p_crank_instance->tcr2_ticks_per_tooth * p_crank_instance->teeth_per_cycle;
  *((uint32_t*)fs_etpu_data_ram_start + ((FS_ETPU_OFFSET_ENG_CYCLE_TCR2_START -1)>>2)) = 0;
  *((uint8_t*)fs_etpu_data_ram_start + FS_ETPU_OFFSET_ENG_POS_STATE) = FS_ETPU_ENG_POS_SEEK;

  /* Write HSR */
  eTPU->CHAN[chan_num].HSRR.R = FS_ETPU_CRANK_HSR_INIT;

  /* Set channel priority */
  fs_etpu_enable(chan_num, priority);

  return(FS_ETPU_ERROR_NONE);
}

/*******************************************************************************
* FUNCTION: fs_etpu_crank_config
****************************************************************************//*!
* @brief   This function changes the CRANK configuration.
*          In EMUL version, this has no effect!
*
* @note    The following actions are performed in order:
*          -# Write configuration parameter values to eTPU DATA RAM
*
* @param   *p_crank_instance - This is a pointer to the instance structure
*            @ref crank_instance_t.
* @param   *p_crank_config - This is a pointer to the structure of configuration
*            parameters @ref crank_config_t.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_NONE - No error
*
*******************************************************************************/
uint32_t fs_etpu_crank_config(
  struct crank_instance_t *p_crank_instance,
  struct crank_config_t   *p_crank_config)
{
  uint32_t *cpba;
  uint32_t *cpbae;

  cpba = p_crank_instance->cpba;
  cpbae = cpba + (0x4000 >> 2); /* sign-extended memory area */

  /* Write channel parameters */
  /* 24-bit - use cpbae to prevent from overwriting bits 31:24 */
  *(cpbae + ((FS_ETPU_CRANK_OFFSET_BLANK_TIME              - 1)>>2)) = p_crank_config->blank_time;
  *(cpbae + ((FS_ETPU_CRANK_OFFSET_GAP_RATIO               - 1)>>2)) = p_crank_config->gap_ratio;
  *(cpbae + ((FS_ETPU_CRANK_OFFSET_WIN_RATIO_NORMAL        - 1)>>2)) = p_crank_config->win_ratio_normal;
  *(cpbae + ((FS_ETPU_CRANK_OFFSET_WIN_RATIO_ACROSS_GAP    - 1)>>2)) = p_crank_config->win_ratio_across_gap;
  *(cpbae + ((FS_ETPU_CRANK_OFFSET_WIN_RATIO_AFTER_GAP     - 1)>>2)) = p_crank_config->win_ratio_after_gap;
  *(cpbae + ((FS_ETPU_CRANK_OFFSET_WIN_RATIO_AFTER_TIMEOUT - 1)>>2)) = p_crank_config->win_ratio_after_timeout;
  *(cpbae + ((FS_ETPU_CRANK_OFFSET_FIRST_TOOTH_TIMEOUT     - 1)>>2)) = p_crank_config->first_tooth_timeout;
  /* 8-bit */
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_TEETH_PER_SYNC) = p_crank_config->teeth_per_sync;
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_BLANK_TEETH   ) = p_crank_config->blank_teeth;

  return(FS_ETPU_ERROR_NONE);
}

/*******************************************************************************
* FUNCTION: fs_etpu_crank_get_states
****************************************************************************//*!
* @brief   This function reads state parameter values of the CRANK function.
*
* @note    The following actions are performed in order:
*          -# Read parameter values from eTPU DATA RAM
*          -# Clear Crank error
*
* @param   *p_crank_instance - This is a pointer to the instance structure
*            @ref crank_instance_t.
* @param   *p_crank_states - This is a pointer to the structure of states
*            @ref crank_states_t which is updated.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_NONE - No error
*
*******************************************************************************/
uint32_t fs_etpu_crank_get_states(
  struct crank_instance_t *p_crank_instance,
  struct crank_states_t   *p_crank_states)
{
  uint32_t *cpba;
  uint32_t *cpbae;

  cpba = p_crank_instance->cpba;
  cpbae = cpba + (0x4000 >> 2); /* sign-extended memory area */

  /* Read channel and global parameters */
  p_crank_states->state               = *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_STATE);
  p_crank_states->eng_pos_state       = *((uint8_t*)fs_etpu_data_ram_start + FS_ETPU_OFFSET_ENG_POS_STATE);
  p_crank_states->tooth_counter_gap   = *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_TOOTH_COUNTER_GAP);
  p_crank_states->tooth_counter_cycle = *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_TOOTH_COUNTER_CYCLE);
  p_crank_states->last_tooth_period   = *(cpbae + ((FS_ETPU_CRANK_OFFSET_LAST_TOOTH_PERIOD - 1)>>2));
  p_crank_states->error               = *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_ERROR);
  /* Clear Crank error */
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_ERROR) = 0;

  return(FS_ETPU_ERROR_NONE);
}


/*******************************************************************************
* FUNCTION: fs_etpu_crank_set_sync
****************************************************************************//*!
* @brief   In EMUL version, this function adjusts the TCR2 angle counter.
*
* @note    The following actions are performed in order:
*          -# Write channel parameter tcr2_adjustment
*          -# Write HSR FS_ETPU_CRANK_HSR_SET_SYNC
*
* @param   *p_crank_instance - This is a pointer to the instance structure
*            @ref crank_instance_t.
* @param   tcr2_adjustment - This is the TCR2 angle value corresponding to the
*            fisrt tooth after the gap at which the recognized Cam log ends.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_NONE - No error
*
*******************************************************************************/
uint32_t fs_etpu_crank_set_sync(
  struct crank_instance_t *p_crank_instance,
                 uint24_t tcr2_adjustment)
{
  uint32_t *cpba;
  uint32_t *cpbae;

  cpba = p_crank_instance->cpba;
  cpbae = cpba + (0x4000 >> 2); /* sign-extended memory area */

  /* Write channel parameter - use cpbae to prevent from overwriting bits 31:24 */
  *(cpbae + ((FS_ETPU_CRANK_OFFSET_TCR2_ADJUSTMENT - 1)>>2)) = tcr2_adjustment;

  /* Write HSR */
  eTPU->CHAN[p_crank_instance->chan_num].HSRR.R = FS_ETPU_CRANK_HSR_SET_SYNC;

  return(FS_ETPU_ERROR_NONE);
}


/*******************************************************************************
* FUNCTION: fs_etpu_crank_set_speed - EMUL version only
****************************************************************************//*!
* @brief   This function sets the CRANK tooth_period and starts the CRANK angle
*          counter if not started yet.
*
* @note    The following actions are performed in order:
*          -# Write channel parameter last_tooth_period
*          -# Write HSR FS_ETPU_CRANK_HSR_SET_SPEED
*
* @param   *p_crank_instance - This is a pointer to the instance structure
*            @ref crank_instance_t.
* @param   tooth_period - This is the TCR1 time value of a single tooth period.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_NONE - No error
*
*******************************************************************************/
uint32_t fs_etpu_crank_set_speed(
  struct crank_instance_t *p_crank_instance,
                 uint24_t tooth_period)
{
  uint32_t *cpba;
  uint32_t *cpbae;

  cpba = p_crank_instance->cpba;
  cpbae = cpba + (0x4000 >> 2); /* sign-extended memory area */

  /* Write channel parameter - use cpbae to prevent from overwriting bits 31:24 */
  *(cpbae + ((FS_ETPU_CRANK_OFFSET_LAST_TOOTH_PERIOD - 1)>>2)) = tooth_period;

  /* Write HSR */
  eTPU->CHAN[p_crank_instance->chan_num].HSRR.R = FS_ETPU_CRANK_HSR_SET_SPEED;

  return(FS_ETPU_ERROR_NONE);
}


/*******************************************************************************
* FUNCTION: fs_etpu_crank_copy_tooth_period_log
****************************************************************************//*!
* @brief   This function copies the CRANK tooth_period_log into another array
*          in RAM.
*
* @param   *p_crank_instance - This is a pointer to the instance structure
*            @ref crank_instance_t.
* @param   *p_tooth_period_log - This is a pointer where the CRANK
*            tooth_period_log will be copied to. The amount of data copied
*            in bytes is 4 * p_crank_instance->teeth_per_cycle.
*
* @return  A pointer to a memory location just after the copied data.
*
*******************************************************************************/
uint24_t *fs_etpu_crank_copy_tooth_period_log(
  struct crank_instance_t *p_crank_instance,
                 uint24_t *p_tooth_period_log)
{
  uint24_t *dest;
  uint24_t *source;
  uint8_t  size;

  dest   = p_tooth_period_log;
  source = p_crank_instance->cpba_tooth_period_log;
  size   = p_crank_instance->teeth_per_cycle;

  while(size--)
  {
    *dest++ = *source++;
  }

  return(dest);
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
 * Revision 0.2  2014/01/15  r54529
 * Improvment of DoxyGen comments.
 *
 * Revision 0.1  2013/03/29  r54529
 * Initial version based on standard etpu_crank.c v0.2.
 ******************************************************************************/