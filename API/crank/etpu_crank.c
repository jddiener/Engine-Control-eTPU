/*******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2015 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
****************************************************************************//*!
*
* @file    etpu_crank.c
*
* @author  Milan Brejl [r54529]
*
* @version 1.2
*
* @date    01-Sep-2015
*
* @brief   This file contains API for using the eTPU function
*          Crank (CRANK).
*
****************************************************************************//*!
*
* @mainpage
*
* The eTPU CRANK APIs @ref etpu_crank.c/.h includes API functions for eTPU
* function CRANK, typically used together with CAM in an engine position system.
*
* The CRANK eTPU function uses 1 eTPU channel to process the tooth signal from a
* crankshaft sensor and generate the eTPU-internal angle-base (TCR2).
* The CRANK eTPU function uses the Enhanced Angle Counter (EAC) eTPU hardware.
* The CRANK eTPU function can be assigned to one of:
* - eTPU channel 0, input signal connected to TCRCLK
* - eTPU channel 1, input signal connected to channel 1 input (eTPU2 only)
* - eTPU channel 2, input signal connected to channel 2 input (eTPU2 only)
* Remember to set the TBCR.AM field correspondingly.
*
* Features:
* - Based on the selected polarity, either raising or falling signal transitions
*   are detected.
* - There are various tooth patterns supported:
*   - single gap
*   - multiple equally spaced gaps
*   - an additional tooth instead of a gap
* - The tooth pattern is described by
*   - number of teeth between two gaps (@ref teeth_till_gap),
*   - number of missing teeth in one gap (@ref teeth_in_gap) and
*   - number of teeth per one engine cycle (@ref teeth_per_cycle).
* - An additional tooth instead of a gap is characterized by teeth_in_gap = 0.
* - The number of angle-base counts per one tooth is configurable
*   (@ref ticks_per_tooth).
* - The gap is recognized using an ABA test, see @ref gap_ratio.
* - A noise imunity and check of unexpected acceleration/deceleration are
*   achieved using tooth windows (@ref win_ratio_normal,
*   @ref win_ratio_across_gap, @ref win_ratio_after_gap,
*   @ref win_ratio_after_timeout)
* - The measured tooth periods can optionally be logged to an array.
* - The CRANK state and the global engine position state are handled.
*   The CRANK state can be one of:
*   - @ref FS_ETPU_CRANK_SEEK
*   - @ref FS_ETPU_CRANK_BLANK_TIME
*   - @ref FS_ETPU_CRANK_BLANK_TEETH
*   - @ref FS_ETPU_CRANK_FIRST_TRANS
*   - @ref FS_ETPU_CRANK_SECOND_TRANS
*   - @ref FS_ETPU_CRANK_TEST_POSSIBLE_GAP
*   - @ref FS_ETPU_CRANK_VERIFY_GAP
*   - @ref FS_ETPU_CRANK_COUNTING
*   - @ref FS_ETPU_CRANK_COUNTING_TIMEOUT
*   - @ref FS_ETPU_CRANK_TOOTH_BEFORE_GAP
*   - @ref FS_ETPU_CRANK_TOOTH_BEFORE_GAP_NOT_HRM (only when ERRATTA_2477 is defined)
*     @ref FS_ETPU_CRANK_ADDITIONAL_TOOTH
*   - @ref FS_ETPU_CRANK_TOOTH_AFTER_GAP
*   The global global engine position state can be one of:
    - @ref FS_ETPU_ENG_POS_SEEK
    - @ref FS_ETPU_ENG_POS_FIRST_HALF_SYNC
    - @ref FS_ETPU_ENG_POS_PRE_FULL_SYNC
    - @ref FS_ETPU_ENG_POS_FULL_SYNC
* - 8 error conditions are reported:
    - @ref FS_ETPU_CRANK_ERR_INVALID_TRANS - an internal error.
    - @ref FS_ETPU_CRANK_ERR_INVALID_MATCH - an internal error.
    - @ref FS_ETPU_CRANK_ERR_TIMEOUT - a transition was not detected in the
      expectation window.
    - @ref FS_ETPU_CRANK_ERR_STALL - the engine position cannot be handled any
      more. The synchronization algorithm will be restarted.
    - @ref FS_ETPU_CRANK_ERR_INTERNAL - an internal error.
    - @ref FS_ETPU_CRANK_ERR_TIMEOUT_BEFORE_GAP - timeout on the last tooth
      before gap detected.
    - @ref FS_ETPU_CRANK_ERR_TIMEOUT_AFTER_GAP - timeout on the first tooth
      after gap detected.
    - @ref FS_ETPU_CRANK_ERR_TOOTH_IN_GAP - a tooth was detected where the gap
      was expected.
* - Channel interrupt is generated when:
*   - the global engine position state has been changed.
*   - once per engine cycle, on the first tooth, in full synchronization state
*    (FS_ETPU_ENG_POS_FULL_SYNC).
*   - during synchronization, when the CAM log buffer is ready.
*
* The Synchronization of eTPU TCR2 angle counter to the physical rotation of
* the crank wheel is, from the software point of view, result of a sequence of
* processing on both the eTPU and the CPU site:
* -# eTPU function CRANK recognizes the gap or the additional tooth on the crank
*    wheel. eng_pos_state is set to HALF_SYNC.
* -# eTPU function CAM starts to log cam signal transitions. It last for for
*    teeth_per_sync crank teeth. After that CRANK sets channel interrupt flag.
*    eng_pos_state is set to PRE_FULL_SYNC.
* -# On the CRANK interrupt, the CPU can use the Cam Log to decode the engine
*    position at the interrupt (at the first tooth after gap).
*    The CPU writes the decoded position (TCR2 engine angle), using
*    @ref fs_etpu_crank_set_sync.
* -# eTPU function CRANK adjusts the engine angle TCR2 value and sets
*    eng_pos_state to FULL_SYNC. The full synchronization is achieved.
*
*******************************************************************************/
/*******************************************************************************
* Includes
*******************************************************************************/
#include "etpu_crank.h"   /* private header file */
#include "etpu_util.h"    /* utility routines for working with the eTPU */

/*******************************************************************************
* Global variables
*******************************************************************************/
extern uint32_t fs_etpu_data_ram_start;
extern uint32_t fs_etpu_data_ram_ext;

/*******************************************************************************
* FUNCTION: fs_etpu_crank_init
****************************************************************************//*!
* @brief   This function initializes eTPU channels to run CRANK function.
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
  uint16_t misscnt_mask;

  chan_num = p_crank_instance->chan_num;
  priority = p_crank_instance->priority;
  cpba_log = p_crank_instance->cpba_tooth_period_log;
  cpba     = p_crank_instance->cpba;

  /* Use user-defined CPBA or allocate new eTPU DATA RAM */
  if(cpba == 0)
  {
    cpba = fs_etpu_malloc(FS_ETPU_CRANK_NUM_PARMS);
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
       (FS_ETPU_CRANK_TABLE_SELECT << 24) +
       (FS_ETPU_CRANK_FUNCTION_NUMBER << 16) +
       (((uint32_t)cpba - fs_etpu_data_ram_start) >> 3);
  eTPU->CHAN[chan_num].SCR.R = (uint32_t)p_crank_instance->polarity
	                             + (uint32_t)p_crank_instance->log_tooth_periods;

  /* Write channel parameters */
  /* 24-bit */
  *(cpba + ((FS_ETPU_CRANK_OFFSET_BLANK_TIME              - 1)>>2)) = p_crank_config->blank_time;
  *(cpba + ((FS_ETPU_CRANK_OFFSET_TCR2_TICKS_PER_TOOTH    - 1)>>2)) = p_crank_instance->tcr2_ticks_per_tooth;
  *(cpba + ((FS_ETPU_CRANK_OFFSET_TCR2_TICKS_PER_ADD_TOOTH- 1)>>2)) = p_crank_instance->tcr2_ticks_per_add_tooth;
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
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_TCR1_CLOCK_SOURCE_DIV1) = (p_crank_instance->tcr1_clock_source == FS_ETPU_TCR1CS_DIV1);
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_TEETH_TILL_GAP     ) = p_crank_instance->teeth_till_gap;
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_TEETH_IN_GAP       ) = p_crank_instance->teeth_in_gap;
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_TEETH_PER_CYCLE    ) = p_crank_instance->teeth_per_cycle;
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_TEETH_PER_SYNC     ) = p_crank_config->teeth_per_sync;
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_TOOTH_COUNTER_GAP  ) = 0;
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_TOOTH_COUNTER_CYCLE) = 0;
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_BLANK_TEETH        ) = p_crank_config->blank_teeth;
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_STATE              ) = FS_ETPU_CRANK_SEEK;
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_ERROR              ) = FS_ETPU_CRANK_ERR_NO_ERROR;
  /* 16-bit */
  misscnt_mask = p_crank_instance->teeth_in_gap << 13;
  misscnt_mask = (misscnt_mask & 0x6000) | ((misscnt_mask & 0x8000)>>5);
  *((uint16_t*)cpba + (FS_ETPU_CRANK_OFFSET_MISSCNT_MASK>>1)) = misscnt_mask;
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
  p_crank_states->last_tooth_period_norm = *(cpbae + ((FS_ETPU_CRANK_OFFSET_LAST_TOOTH_PERIOD_NORM - 1)>>2));
  p_crank_states->error              |= *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_ERROR);
  /* Clear Crank error */
  *((uint8_t*)cpba + FS_ETPU_CRANK_OFFSET_ERROR) = 0;

  return(FS_ETPU_ERROR_NONE);
}


/*******************************************************************************
* FUNCTION: fs_etpu_crank_set_sync
****************************************************************************//*!
* @brief   This function adjusts the TCR2 angle counter and asks the eTPU to
*          set the global engine position status to FULL_SYNC.
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
* FUNCTION: fs_etpu_crank_get_angle_reseting
****************************************************************************//*!
* @brief   This function returns the engine angle as a number of TCR2 ticks
*          in a range 0 to (tcr2_ticks_per_tooth*teeth_per_cycle - 1), 
*          corresponding to 0-720 degrees.
*
* @return  A number of TCR2 ticks which determines the actual engine angle.
*          The value 0 corresponds to the first tooth after gap. A maximum
*          number is (tcr2_ticks_per_tooth*teeth_per_cycle - 1).
*
*******************************************************************************/
uint32_t fs_etpu_crank_get_angle_reseting(void)
{
  uint32_t tcr2_ticks;
  uint32_t tcr2_start;
  uint32_t tcr2;

  tcr2_ticks = fs_etpu_get_global_24(FS_ETPU_OFFSET_ENG_CYCLE_TCR2_TICKS);
  tcr2_start = fs_etpu_get_global_24(FS_ETPU_OFFSET_ENG_CYCLE_TCR2_START);
  tcr2 = eTPU->TB2R_A.R;
  return((0x00FFFFFF & (tcr2 + tcr2_ticks - tcr2_start)) % tcr2_ticks);
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
 * Revision 0.1  2012/07/02  r54529
 * Initial version of file.
 ******************************************************************************/
