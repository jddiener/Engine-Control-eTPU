/*******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2014 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
****************************************************************************//*!
*
* @file    etpu_fuel.c
*
* @author  Milan Brejl [r54529]
*
* @version 1.0
*
* @date    17-Mar-2014
*
* @brief   This file contains API for using the eTPU function
*          Fuel Port Injection (FUEL).
*
****************************************************************************//*!
*
* @mainpage
*
* The eTPU FUEL APIs @ref etpu_fuel.c/.h includes API functions for eTPU
* function Fuel Port Injection.
*
* Each instance of the FUEL eTPU function controls a single FUEL channel.
*
* There is a FUEL parameter tdc_angle, relative to which all angles are
* defined. Positive angles precede the tdc_angle, negative angles come after.
*
* The CPU can control the FUEL operation using FUEL config variables
* angle_normal_end, angle_stop, angle_offset_recalc,
* injection_time, compensation_time, injection_time_minimum and
* off_time_minimum.
*
* The CPU can update the amount of injected fuel during the running injection
* using the function @ref fs_etpu_fuel_update_injection_time() which not only
* sets the injection_time value for the next engine cycles, but also updates the
* current injection - shorts the pulse, extends the pulse or generates an
* additional pulse.
*
* In order to
* - immediatelly disable injection generation, set injection_time to 0
* - disable the injection generation from the next cycle, but finish the running
*   injection pulse, use generation_diable configuration flag.
*
* The CPU can monitor the FUEL operation using FUEL state variables
* injection_time_applied, injection_start_angle and error.
* The reported error flags are:
*   @ref FS_ETPU_FUEL_ERROR_STOP_ANGLE_APPLIED - a fuel injection pulse has
*     been stopped and shorted by the stop_angle. Hence, the commanded
*     injection_time and the injection_time_applied may differ.
*   @ref FS_ETPU_FUEL_ERROR_MINIMUM_INJ_TIME_APPLIED - a fuel injection pulse,
*     the main or an additional one, is shorter than the injection_time_minimum
*     and hence not generated, skipped. The commanded injection_time and
*     the injection_time_applied may differ.
*
* Channel interrupt is generated once every engine cycle, on the angle_stop.
*
*******************************************************************************/
/*******************************************************************************
* Includes
*******************************************************************************/
#include "etpu_fuel.h"     /* private header file */
#include "etpu_util.h"    /* utility routines for working with the eTPU */

/*******************************************************************************
* Global variables
*******************************************************************************/
extern uint32_t fs_etpu_data_ram_start;
extern uint32_t fs_etpu_data_ram_ext;

/*******************************************************************************
* FUNCTION: fs_etpu_fuel_init
****************************************************************************//*!
* @brief   This function initializes eTPU channels to run FUEL function.
*
* @note    The following actions are performed in order:
*          -# Use user-defined CPBA or allocate new eTPU DATA RAM
*          -# Write chan config registers and FM bits
*          -# Write channel parameters
*          -# Write HSR
*          -# Set channel priority
*
* @param   *p_fuel_instance - This is a pointer to the instance structure
*            @ref fuel_instance_t.
* @param   *p_fuel_config - This is a pointer to the structure of configuration
*            parameters @ref fuel_config_t.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_MALLOC - eTPU DATA RAM memory allocation error
*          - @ref FS_ETPU_ERROR_NONE - No error
*
* @warning This function does not configure the pins, only the eTPU channels.
*******************************************************************************/
uint32_t fs_etpu_fuel_init(
  struct fuel_instance_t   *p_fuel_instance,
  struct fuel_config_t     *p_fuel_config)
{
  uint8_t  chan_num;
  uint8_t  priority;
  uint32_t *cpba;

  chan_num = p_fuel_instance->chan_num;
  priority = p_fuel_instance->priority;
  cpba     = p_fuel_instance->cpba;

  /* Use user-defined CPBA or allocate new eTPU DATA RAM for chan. parameters */
  if(cpba == 0)
  {
    cpba = fs_etpu_malloc(FS_ETPU_FUEL_NUM_PARMS);
    if(cpba == 0)
    {
      return(FS_ETPU_ERROR_MALLOC);
    }
    else
    {
      p_fuel_instance->cpba = cpba;
    }
  }

  /* Write chan config registers and FM bits */
  eTPU->CHAN[chan_num].CR.R =
       (FS_ETPU_FUEL_TABLE_SELECT << 24) +
       (FS_ETPU_FUEL_FUNCTION_NUMBER << 16) +
       (((uint32_t)cpba - fs_etpu_data_ram_start) >> 3);
  eTPU->CHAN[chan_num].SCR.R = (uint32_t)p_fuel_instance->polarity;

  /* Write channel parameters */
  /* 24-bit */
  *(cpba + ((FS_ETPU_FUEL_OFFSET_TDC_ANGLE                 - 1)>>2)) = p_fuel_instance->tdc_angle;
  *(cpba + ((FS_ETPU_FUEL_OFFSET_TDC_ANGLE_ACTUAL          - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_FUEL_OFFSET_ANGLE_NORMAL_END          - 1)>>2)) = p_fuel_config->angle_normal_end;
  *(cpba + ((FS_ETPU_FUEL_OFFSET_ANGLE_STOP                - 1)>>2)) = p_fuel_config->angle_stop;
  *(cpba + ((FS_ETPU_FUEL_OFFSET_ANGLE_OFFSET_RECALC       - 1)>>2)) = p_fuel_config->angle_offset_recalc;
  *(cpba + ((FS_ETPU_FUEL_OFFSET_INJECTION_TIME            - 1)>>2)) = p_fuel_config->injection_time;
  *(cpba + ((FS_ETPU_FUEL_OFFSET_COMPENSATION_TIME         - 1)>>2)) = p_fuel_config->compensation_time;
  *(cpba + ((FS_ETPU_FUEL_OFFSET_INJECTION_TIME_MINIMUM    - 1)>>2)) = p_fuel_config->injection_time_minimum;
  *(cpba + ((FS_ETPU_FUEL_OFFSET_OFF_TIME_MINIMUM          - 1)>>2)) = p_fuel_config->off_time_minimum;
  *(cpba + ((FS_ETPU_FUEL_OFFSET_INJECTION_TIME_APPLIED    - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_FUEL_OFFSET_INJECTION_TIME_APPLIED_CPU- 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_FUEL_OFFSET_INJECTION_START_ANGLE     - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_FUEL_OFFSET_INJECTION_START_ANGLE_CPU - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_FUEL_OFFSET_PULSE_START_TIME          - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_FUEL_OFFSET_PULSE_END_TIME            - 1)>>2)) = 0;
  /* 8-bit */
  *((uint8_t*)cpba + FS_ETPU_FUEL_OFFSET_ERROR) = 0;
  *((uint8_t*)cpba + FS_ETPU_FUEL_OFFSET_GENERATION_DISABLE ) = p_fuel_config->generation_disable;

  /* Write HSR */
  eTPU->CHAN[chan_num].HSRR.R = FS_ETPU_FUEL_HSR_INIT;

  /* Set channel priority */
  fs_etpu_enable(chan_num, priority);

  return(FS_ETPU_ERROR_NONE);
}

/*******************************************************************************
* FUNCTION: fs_etpu_fuel_config
****************************************************************************//*!
* @brief   This function changes the FUEL configuration.
*
* @warning The new injection_time is applied from the next fuel injection.
*          Use @ref fs_etpu_fuel_update_injection_time() in order to update
*          the current injection.
*
* @note    The following actions are performed in order:
*          -# Write configuration parameter values to eTPU DATA RAM
*
* @param   *p_fuel_instance - This is a pointer to the instance structure
*            @ref inj_instance_t.
* @param   *p_fuel_config - This is a pointer to the structure of configuration
*            parameters @ref inj_config_t.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_NONE - No error.
*
*******************************************************************************/
uint32_t fs_etpu_fuel_config(
  struct fuel_instance_t *p_fuel_instance,
  struct fuel_config_t   *p_fuel_config)
{
  uint32_t *cpba;
  uint32_t *cpbae;

  cpba  = p_fuel_instance->cpba;
  cpbae = cpba + (0x4000 >> 2); /* sign-extended memory area */

  /* Write channel parameters */
  /* 24-bit - use cpbae to prevent from overwriting bits 31:24 */
  *(cpbae + ((FS_ETPU_FUEL_OFFSET_ANGLE_NORMAL_END       - 1)>>2)) = (uint24_t)p_fuel_config->angle_normal_end;
  *(cpbae + ((FS_ETPU_FUEL_OFFSET_ANGLE_STOP             - 1)>>2)) = (uint24_t)p_fuel_config->angle_stop;
  *(cpbae + ((FS_ETPU_FUEL_OFFSET_ANGLE_OFFSET_RECALC    - 1)>>2)) = (uint24_t)p_fuel_config->angle_offset_recalc;
  *(cpbae + ((FS_ETPU_FUEL_OFFSET_INJECTION_TIME         - 1)>>2)) = p_fuel_config->injection_time;
  *(cpbae + ((FS_ETPU_FUEL_OFFSET_COMPENSATION_TIME      - 1)>>2)) = p_fuel_config->compensation_time;
  *(cpbae + ((FS_ETPU_FUEL_OFFSET_INJECTION_TIME_MINIMUM - 1)>>2)) = p_fuel_config->injection_time_minimum;
  *(cpbae + ((FS_ETPU_FUEL_OFFSET_OFF_TIME_MINIMUM       - 1)>>2)) = p_fuel_config->off_time_minimum;
  /* 8-bit */
  *((uint8_t*)cpba + FS_ETPU_FUEL_OFFSET_GENERATION_DISABLE) = p_fuel_config->generation_disable;

  return(FS_ETPU_ERROR_NONE);
}

/*******************************************************************************
* FUNCTION: fs_etpu_fuel_update_injection_time
****************************************************************************//*!
* @brief   This function updates the FUEL injection_time.
*
* @note    The following actions are performed in order:
*          -# Check there is no pending HSR
*          -# Write parameter value to eTPU DATA RAM
*          -# Write HSR
*
* @param   *p_fuel_instance - This is a pointer to the instance structure
*            @ref inj_instance_t.
* @param   *p_fuel_config - This is a pointer to the structure of configuration
*            parameters @ref inj_config_t.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_NONE - No error.
*          - @ref FS_ETPU_ERROR_TIMING - The mode was not set because there is
*              a HSR pending on the eTPU channel.
*              It can be caused by a call of another API function which writes
*              the HSR to the same channel shortly before this function call.
*              Try to repeat the function call several microseconds later.
*
*******************************************************************************/
uint32_t fs_etpu_fuel_update_injection_time(
  struct fuel_instance_t *p_fuel_instance,
  struct fuel_config_t   *p_fuel_config)
{
  uint32_t *cpba;
  uint32_t *cpbae;

  /* Check there is no pending HSR */
  if(eTPU->CHAN[p_fuel_instance->chan_num].HSRR.R != 0)
  {
    return(FS_ETPU_ERROR_TIMING);
  }
  else
  {
    cpba  = p_fuel_instance->cpba;
    cpbae = cpba + (0x4000 >> 2); /* sign-extended memory area */

    /* Write channel parameter */
    /* 24-bit - use cpbae to prevent from overwriting bits 31:24 */
    *(cpbae + ((FS_ETPU_FUEL_OFFSET_INJECTION_TIME - 1)>>2)) = p_fuel_config->injection_time;

    /* Write HSR to run UPDATE on eTPU */
    eTPU->CHAN[p_fuel_instance->chan_num].HSRR.R = FS_ETPU_FUEL_HSR_UPDATE;

    return(FS_ETPU_ERROR_NONE);
  }
}

/*******************************************************************************
* FUNCTION: fs_etpu_fuel_get_states
****************************************************************************//*!
* @brief   This function reads FUEL state variables, including error flags,
*          and clears the eTPU error after reading.
*
* @note    The following actions are performed in order:
*          -# Read state parameter values from eTPU DATA RAM
*          -# Clear FUEL error
*
* @param   *p_fuel_instance - This is a pointer to the instance structure
*            @ref p_fuel_instance.
* @param   *p_fuel_states - This is a pointer to the return structure of states
*            @ref p_fuel_states.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_NONE - No error
*
*******************************************************************************/
uint32_t fs_etpu_fuel_get_states(
  struct fuel_instance_t *p_fuel_instance,
  struct fuel_states_t   *p_fuel_states)
{
  uint32_t *cpba;
  uint32_t *cpbae;

  cpba  = p_fuel_instance->cpba;
  cpbae = cpba + (0x4000 >> 2); /* sign-extended memory area */

  /* Read FUEL channel parameters */
  p_fuel_states->injection_time_applied = *(cpbae + ((FS_ETPU_FUEL_OFFSET_INJECTION_TIME_APPLIED_CPU - 1)>>2));
  p_fuel_states->injection_start_angle  = *(cpbae + ((FS_ETPU_FUEL_OFFSET_INJECTION_START_ANGLE_CPU  - 1)>>2));
  p_fuel_states->error                 |= *((uint8_t*)cpba + FS_ETPU_FUEL_OFFSET_ERROR);
  /* Clear FUEL error */
  *((uint8_t*)cpba + FS_ETPU_FUEL_OFFSET_ERROR) = 0;

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
 * Revision 0.2  2013/09/05  r54529
 * Addition of generation_disable.
 *
 * Revision 0.1  2013/08/28  r54529
 * Initial version of file.
 *
 ******************************************************************************/