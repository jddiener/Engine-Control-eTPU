/*******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2014 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
****************************************************************************//*!
*
* @file    etpu_inj.c
*
* @author  Milan Brejl [r54529]
*
* @version 1.0
*
* @date    17-Mar-2014
*
* @brief   This file contains API for using the eTPU function
*          Direct Injection (INJ).
*
****************************************************************************//*!
*
* @mainpage
*
* The eTPU INJ APIs @ref etpu_inj.c/.h includes API functions for eTPU
* function Direct Injection.
*
* Each instance of the INJ eTPU function controls a single INJ channel together
* with up to 3 BANK channels. The BANK channels can be common for more INJ
* instances.
*
* This eTPU function enables to generate complex angle- and time-based output
* patterns, primarily dedicated to direct injection timing control.
* The INJ channel can control the outputs of up to 3 additional channels,
* called BANK channels (e.g. Boost, Batt, ...).
*
* An injection sequence consists of injections. Each injection starts at a
* defined angle (angle_start) and consists of phases. Each phase is defined
* by output states of the INJ and all the BANK channels, a phase duration,
* and options to generate DMA requests at the beginning of the phase.
* The number of injections and the number of phases in each injection is
* configurable. More INJ channels (individual injectors) may use separate
* injection sequences, or can share the same sequence.
*
* There is an INJ parameter tdc_angle, relative to which all angles are
* defined. Positive angles precede the tdc_angle, negative angles come after.
* INJ parameter angle_irq which defines a tdc_angle-relative angle at which
* an IRQ request is generated. The CPU may reconfigure the injection sequence
* setting on this interrupt, but not later then the first injection
* angle_start is reached. If the CPU does not reconfigure, the actual
* injection sequence definition is used.
* INJ parameter angle_stop defines the latest tdc_angle-relative angle when
* the whole injection sequence must be finished. If it is not, all INJ and
* BANK outputs are turned to inactive state, whatever injection phase is
* active.
*
* The CPU can monitor the INJ operation using INJ state variables
* injection_counter, phase_counter and error. The reported error flags are:
* @ref INJ_ERROR_PREV_INJ_NOT_FINISHED - injection sequence can not start while
*   another INJ channel occupies the BANK channels. The injection sequence
*   is not generated. The global parameter INJ_active_bank_chans keeps
*   track of which BANK channels are in use.
* @ref INJ_ERROR_LATE_START_ANGLE_1ST - the 1st injection start-angle was about
*   to be scheduled in past, hence the whole injection sequence was skipped.
* @ref INJ_ERROR_LATE_START_ANGLE_NTH - the 2nd or later injection start-angle
*   was about to be scheduled in past, hence the rest of the injection
*   sequence was skipped.
* @ref INJ_ERROR_STOPPED_BY_STOP_ANGLE - the injection sequence was not finished
*   before the stop-angle and hence the injection was hard-stopped at the
*   stop-angle.
*
*******************************************************************************/
/*******************************************************************************
* Includes
*******************************************************************************/
#include "etpu_inj.h"     /* private header file */
#include "etpu_util.h"    /* utility routines for working with the eTPU */

/*******************************************************************************
* Global variables
*******************************************************************************/
extern uint32_t fs_etpu_data_ram_start;
extern uint32_t fs_etpu_data_ram_ext;

/*******************************************************************************
* FUNCTION: fs_etpu_inj_init
****************************************************************************//*!
* @brief   This function initializes eTPU channels to run INJ function.
*
* @note    The following actions are performed in order:
*          -# Use user-defined CPBA or allocate new eTPU DATA RAM
*          -# Write chan config registers and FM bits
*          -# Write channel parameters
*          -# Write HSR
*          -# Set channel priority
*
* @param   *p_inj_instance - This is a pointer to the instance structure
*            @ref inj_instance_t.
* @param   *p_inj_config - This is a pointer to the structure of configuration
*            parameters @ref inj_config_t.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_MALLOC - eTPU DATA RAM memory allocation error
*          - @ref FS_ETPU_ERROR_NONE - No error
*
* @warning This function does not configure the pins, only the eTPU channels.
*******************************************************************************/
uint32_t fs_etpu_inj_init(
  struct inj_instance_t   *p_inj_instance,
  struct inj_config_t     *p_inj_config)
{
  uint8_t  chan_num_inj;
  uint8_t  chan_num_bank_1;
  uint8_t  chan_num_bank_2;
  uint8_t  chan_num_bank_3;
  uint8_t  priority;
  uint32_t *cpba;
  uint32_t *cpba_injections;
  uint32_t *cpba_phases;
  uint8_t  injection_count;
  uint8_t  phase_count = 0;
  struct inj_injection_config_t *p_injection_config;
  uint32_t *p_phase_config;
  uint8_t  bank_chan_count = 0;
  uint8_t  inactive_polarities = 0;
  uint24_t bank_chans = 0;
  uint32_t bank_chans_mask = 0;
  uint32_t cr;
  uint8_t  i,j;

  chan_num_inj    = p_inj_instance->chan_num_inj;
  chan_num_bank_1 = p_inj_instance->chan_num_bank_1;
  chan_num_bank_2 = p_inj_instance->chan_num_bank_2;
  chan_num_bank_3 = p_inj_instance->chan_num_bank_3;
  priority        = p_inj_instance->priority;
  cpba            = p_inj_instance->cpba;
  cpba_injections = p_inj_instance->cpba_injections;
  cpba_phases     = p_inj_instance->cpba_phases;

  /* Use user-defined CPBA or allocate new eTPU DATA RAM for chan. parameters */
  if(cpba == 0)
  {
    cpba = fs_etpu_malloc(FS_ETPU_INJ_NUM_PARMS);
    if(cpba == 0)
    {
      return(FS_ETPU_ERROR_MALLOC);
    }
    else
    {
      p_inj_instance->cpba = cpba;
    }
  }
  /* Use user-defined CPBA or allocate new eTPU DATA RAM for injections */
  injection_count = p_inj_config->injection_count;
  if(cpba_injections == 0)
  {
    cpba_injections =
      fs_etpu_malloc(FS_ETPU_INJ_INJECTION_STRUCT_SIZE * injection_count);
    if(cpba_injections == 0)
    {
      return(FS_ETPU_ERROR_MALLOC);
    }
    else
    {
      p_inj_instance->cpba_injections = cpba_injections;
    }
  }
  /* Use user-defined CPBA or allocate new eTPU DATA RAM for inj. phases */
  if(cpba_phases == 0)
  {
    /* Sum number of all injection phases */
    p_injection_config = p_inj_config->p_injection_config;
    for(i=0; i<injection_count; i++)
    {
      phase_count += p_injection_config->phase_count;
      p_injection_config++;
    }

    cpba_phases = fs_etpu_malloc(FS_ETPU_INJ_PHASE_STRUCT_SIZE * phase_count);
    if(cpba_phases == 0)
    {
      return(FS_ETPU_ERROR_MALLOC);
    }
    else
    {
      p_inj_instance->cpba_phases = cpba_phases;
    }
  }

  /* Write chan config registers and FM bits */
  cr = (FS_ETPU_INJ_TABLE_SELECT << 24) +
       (FS_ETPU_INJ_FUNCTION_NUMBER << 16) +
       (((uint32_t)cpba - fs_etpu_data_ram_start) >> 3);
  eTPU->CHAN[chan_num_inj].CR.R = cr;
  eTPU->CHAN[chan_num_inj].SCR.R = (uint32_t)p_inj_instance->polarity_inj
                                   + FS_ETPU_INJ_FM1_CHANNEL_INJ;
  if(chan_num_bank_3 != FS_ETPU_INJ_BANK_CHAN_NOT_USED)
  {
    bank_chan_count = 1;
    bank_chans = chan_num_bank_3;
    bank_chans_mask = 1<<chan_num_bank_3;
    inactive_polarities = p_inj_instance->polarity_bank;
    eTPU->CHAN[chan_num_bank_3].CR.R = cr;
    eTPU->CHAN[chan_num_bank_3].SCR.R = (uint32_t)p_inj_instance->polarity_bank
                                        + FS_ETPU_INJ_FM1_CHANNEL_BANK;
  }
  if(chan_num_bank_2 != FS_ETPU_INJ_BANK_CHAN_NOT_USED)
  {
    bank_chan_count++;
    bank_chans = (bank_chans << 8) | chan_num_bank_2;
    bank_chans_mask |= 1<<chan_num_bank_2;
    inactive_polarities = (inactive_polarities << 1) | p_inj_instance->polarity_bank;
    eTPU->CHAN[chan_num_bank_2].CR.R = cr;
    eTPU->CHAN[chan_num_bank_2].SCR.R = (uint32_t)p_inj_instance->polarity_bank
                                        + FS_ETPU_INJ_FM1_CHANNEL_BANK;
  }
  if(chan_num_bank_1 != FS_ETPU_INJ_BANK_CHAN_NOT_USED)
  {
    bank_chan_count++;
    bank_chans = (bank_chans << 8) | chan_num_bank_1;
    bank_chans_mask |= 1<<chan_num_bank_1;
    inactive_polarities = (inactive_polarities << 1) | p_inj_instance->polarity_bank;
    eTPU->CHAN[chan_num_bank_1].CR.R = cr;
    eTPU->CHAN[chan_num_bank_1].SCR.R = (uint32_t)p_inj_instance->polarity_bank
                                        + FS_ETPU_INJ_FM1_CHANNEL_BANK;
  }
  inactive_polarities = (inactive_polarities << 1) | p_inj_instance->polarity_inj;

  /* Write channel parameters */
  /* 24-bit */
  *(cpba + ((FS_ETPU_INJ_OFFSET_P_INJECTION_FIRST - 1)>>2)) = (uint32_t)cpba_injections - fs_etpu_data_ram_start;
  *(cpba + ((FS_ETPU_INJ_OFFSET_P_INJECTION       - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_INJ_OFFSET_P_PHASE           - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_INJ_OFFSET_BANK_CHANS        - 1)>>2)) = bank_chans;
  *(cpba + ((FS_ETPU_INJ_OFFSET_ANGLE_IRQ         - 1)>>2)) = p_inj_config->angle_irq;
  *(cpba + ((FS_ETPU_INJ_OFFSET_ANGLE_STOP        - 1)>>2)) = p_inj_config->angle_stop;
  *(cpba + ((FS_ETPU_INJ_OFFSET_TDC_ANGLE         - 1)>>2)) = p_inj_instance->tdc_angle;
  *(cpba + ((FS_ETPU_INJ_OFFSET_TDC_ANGLE_ACTUAL  - 1)>>2)) = p_inj_instance->tdc_angle;

  /* 8-bit */
  *((uint8_t*)cpba + FS_ETPU_INJ_OFFSET_INJECTION_COUNT  ) = injection_count;
  *((uint8_t*)cpba + FS_ETPU_INJ_OFFSET_INJECTION_COUNTER) = 0;
  *((uint8_t*)cpba + FS_ETPU_INJ_OFFSET_PHASE_COUNTER    ) = 0;
  *((uint8_t*)cpba + FS_ETPU_INJ_OFFSET_BANK_CHANS_COUNT ) = bank_chan_count;
  *((uint8_t*)cpba + FS_ETPU_INJ_OFFSET_ERROR            ) = 0;
  *((uint8_t*)cpba + FS_ETPU_INJ_OFFSET_INACTIVE_POLARITIES) = inactive_polarities;

  /* 32-bit */
  *(cpba + (FS_ETPU_INJ_OFFSET_BANK_CHANS_MASK >>2)) = bank_chans_mask;

  /* Write array of injection parameters */
  p_injection_config = p_inj_config->p_injection_config;

  for(i=0; i<injection_count; i++)
  {
    phase_count = p_injection_config->phase_count;
    p_phase_config = p_injection_config->p_phase_config;

    /* 24-bit */
    *(cpba_injections + ((FS_ETPU_INJ_OFFSET_P_PHASE_FIRST - 1)>>2)) = (uint32_t)cpba_phases - fs_etpu_data_ram_start;
    *(cpba_injections + ((FS_ETPU_INJ_OFFSET_ANGLE_START   - 1)>>2)) = p_injection_config->angle_start;
    /* 8-bit */
    *((uint8_t*)cpba_injections + FS_ETPU_INJ_OFFSET_PHASE_COUNT   ) = phase_count;

    for(j=0; j<phase_count; j++)
    {
      /* 32-bit */
      *cpba_phases = *p_phase_config;
      p_phase_config++;
      cpba_phases += FS_ETPU_INJ_PHASE_STRUCT_SIZE >> 2;
    }

    p_injection_config++;
    cpba_injections += FS_ETPU_INJ_INJECTION_STRUCT_SIZE >> 2;
  }

  /* Write HSR and Set channel priority*/
  eTPU->CHAN[chan_num_inj].HSRR.R = FS_ETPU_INJ_HSR_INIT;
  fs_etpu_enable(chan_num_inj, priority);
  if(chan_num_bank_1 != FS_ETPU_INJ_BANK_CHAN_NOT_USED)
  {
    eTPU->CHAN[chan_num_bank_1].HSRR.R = FS_ETPU_INJ_HSR_INIT;
    fs_etpu_enable(chan_num_bank_1, priority);
  }
  if(chan_num_bank_2 != FS_ETPU_INJ_BANK_CHAN_NOT_USED)
  {
    eTPU->CHAN[chan_num_bank_2].HSRR.R = FS_ETPU_INJ_HSR_INIT;
    fs_etpu_enable(chan_num_bank_2, priority);
  }
  if(chan_num_bank_3 != FS_ETPU_INJ_BANK_CHAN_NOT_USED)
  {
    eTPU->CHAN[chan_num_bank_3].HSRR.R = FS_ETPU_INJ_HSR_INIT;
    fs_etpu_enable(chan_num_bank_3, priority);
  }

  return(FS_ETPU_ERROR_NONE);
}

/*******************************************************************************
* FUNCTION: fs_etpu_inj_config
****************************************************************************//*!
* @brief   This function changes the INJ configuration.
*
* @note    The following actions are performed in order:
*          -# Read channel parameter from eTPU DATA RAM to check no injection
*             sequence is active on this injector.
*          -# Write configuration parameter values to eTPU DATA RAM
*          -# Write HSR
*
* @warning The new injection sequence definition (array of injections and array
*          of phases of each injection) must fit into the eTPU DATA RAM
*          already allocated.
*
* @param   *p_inj_instance - This is a pointer to the instance structure
*            @ref inj_instance_t.
* @param   *p_inj_config - This is a pointer to the structure of configuration
*            parameters @ref inj_config_t.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_NONE - No error, injection parameters were set.
*          - @ref FS_ETPU_ERROR_TIMING - The injection parameters were not set
*              because the injection sequence is active. During this time
*              the injection parameters cannot be updated.
*
*******************************************************************************/
uint32_t fs_etpu_inj_config(
  struct inj_instance_t *p_inj_instance,
  struct inj_config_t   *p_inj_config)
{
  uint32_t *cpba;
  uint32_t *cpbae;
  uint32_t *cpba_injections;
  uint32_t *cpba_phases;
  uint8_t  injection_idx;
  uint8_t  injection_count;
  uint8_t  phase_count;
  struct inj_injection_config_t *p_injection_config;
  uint32_t *p_phase_config;
  uint8_t  i,j;

  cpba            = p_inj_instance->cpba;
  cpba_injections = p_inj_instance->cpba_injections;
  cpba_phases     = p_inj_instance->cpba_phases;

  /* Check if the injection sequence is not active */
  injection_idx = *((uint8_t*)cpba + FS_ETPU_INJ_OFFSET_INJECTION_COUNTER);
  if(injection_idx != 0)
  {
    /* An injection sequence is active on this injector */
    return(FS_ETPU_ERROR_TIMING);
  }
  else
  {
    /* Write channel parameters */
    /* 24-bit - use cpbae to prevent from overwriting bits 31:24 */
    cpbae = cpba + (0x4000 >> 2); /* sign-extended memory area */
    *(cpbae + ((FS_ETPU_INJ_OFFSET_ANGLE_IRQ  - 1)>>2)) = p_inj_config->angle_irq;
    *(cpbae + ((FS_ETPU_INJ_OFFSET_ANGLE_STOP - 1)>>2)) = p_inj_config->angle_stop;

    /* 8-bit */
    injection_count = p_inj_config->injection_count;
    *((uint8_t*)cpba + FS_ETPU_INJ_OFFSET_INJECTION_COUNT) = injection_count;

    /* Write array of injection parameters */
    p_injection_config = p_inj_config->p_injection_config;

    for(i=0; i<injection_count; i++)
    {
      phase_count = p_injection_config->phase_count;
      p_phase_config = p_injection_config->p_phase_config;

      /* 24-bit */
      *(cpba_injections + ((FS_ETPU_INJ_OFFSET_P_PHASE_FIRST - 1)>>2)) = (uint32_t)cpba_phases - fs_etpu_data_ram_start;
      *(cpba_injections + ((FS_ETPU_INJ_OFFSET_ANGLE_START   - 1)>>2)) = p_injection_config->angle_start;
      /* 8-bit */
      *((uint8_t*)cpba_injections + FS_ETPU_INJ_OFFSET_PHASE_COUNT   ) = phase_count;

      for(j=0; j<phase_count; j++)
      {
        /* 32-bit */
        *cpba_phases = *p_phase_config;
        p_phase_config++;
        cpba_phases += FS_ETPU_INJ_PHASE_STRUCT_SIZE >> 2;
      }

      p_injection_config++;
      cpba_injections += FS_ETPU_INJ_INJECTION_STRUCT_SIZE >> 2;
    }
    /* Write HSR to run UPDATE on eTPU, which reschedules the new start_angle[0] */
    eTPU->CHAN[p_inj_instance->chan_num_inj].HSRR.R = FS_ETPU_INJ_HSR_UPDATE;

    return(FS_ETPU_ERROR_NONE);
  }
}

/*******************************************************************************
* FUNCTION: fs_etpu_inj_get_states
****************************************************************************//*!
* @brief   This function reads INJ state variables, including error flags,
*          and clears the eTPU error after reading.
*
* @note    The following actions are performed in order:
*          -# Read state parameter values from eTPU DATA RAM
*          -# Clear INJ error
*
* @param   *p_inj_instance - This is a pointer to the instance structure
*            @ref inj_instance_t.
* @param   *p_inj_states - This is a pointer to the return structure of states
*            @ref inj_states_t.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_NONE - No error
*
*******************************************************************************/
uint32_t fs_etpu_inj_get_states(
  struct inj_instance_t *p_inj_instance,
  struct inj_states_t   *p_inj_states)
{
  uint32_t *cpba;

  cpba = p_inj_instance->cpba;

  /* Read INJ channel parameters */
  p_inj_states->injection_idx = *((uint8_t*)cpba + FS_ETPU_INJ_OFFSET_INJECTION_COUNTER);
  p_inj_states->phase_idx     = *((uint8_t*)cpba + FS_ETPU_INJ_OFFSET_PHASE_COUNTER);
  p_inj_states->error         = *((uint8_t*)cpba + FS_ETPU_INJ_OFFSET_ERROR);
  /* Clear INJ error */
  *((uint8_t*)cpba + FS_ETPU_INJ_OFFSET_ERROR) = 0;

  return(FS_ETPU_ERROR_NONE);
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
 * Revision 1.0  2014/03/17  r54529
 * Minor comment and formating improvements.
 * Ready for eTPU Engine Control Library release 1.0.
 *
 * Revision 0.2  2013/08/02  r54529
 * fs_etpu_inj_update() removed, fs_etpu_inj_config() modified so that it can be
 * used anytime to update the injection parameters. During the injection
 * sequence activity the injection parameters cannot be updated and
 * fs_etpu_inj_config() returns FS_ETPU_ERROR_TIMING.
 * Separate polarity for Inj and Bank channels.
 *
 * Revision 0.1  2012/05/17  r54529
 * Initial version of file.
 ******************************************************************************/