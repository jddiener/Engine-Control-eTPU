/*******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2014 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
****************************************************************************//*!
*
* @file    etpu_spark.c
*
* @author  Milan Brejl [r54529]
*
* @version 1.0
*
* @date    17-Mar-2014
*
* @brief   This file contains API for using the eTPU function
*          Spark (SPARK).
*
****************************************************************************//*!
*
* @mainpage
*
* The eTPU SPARK APIs @ref etpu_spark.c/.h includes API functions for eTPU
* function Spark.
*
* Each instance of the SPARK eTPU function controls a single SPARK channel.
*
* There is a SPARK parameter tdc_angle, relative to which all angles are
* defined. Positive angles precede the tdc_angle, negative angles come after.
*
* Each instance of the SPARK eTPU function controls a single SPARK channel.
*
* The SPARK eTPU function enables to generate one or more spark output pulses.
* Each spark output includes the main pulse, defined by end_angle and dwell_time
* and limited by min_dwell_time and max_dwell_time. The main pulse is optionaly
* followed by a sequence of multi-pulses, defined by multi_on_time,
* multi_off_time and pulse_count.
*
* The CPU can monitor the SPARK operation using SPARK state variables
* dwell_time_applied and error.
* The reported error flags are:
*   @ref FS_ETPU_SPARK_ERROR_MIN_DWELL_APPLIED - the spark main pulse has
*     been limited by min_dwell_time and the pulse ended at a later angle than
*     the eng_angle. Hence, the commanded dwell_time and the dwell_time_applie 
*     may differ.
*   @ref FS_ETPU_SPARK_ERROR_MAX_DWELL_APPLIED - the spark main pulse has
*     been limited by max_dwell_time and the pulse ended sooner than at the
*     eng_angle. Hence, the commanded dwell_time and the dwell_time_applie 
*     may differ.
*
* Channel interrupt is generated before each single spark, on the recalc_angle.
*
*******************************************************************************/
/*******************************************************************************
* Includes
*******************************************************************************/
#include "etpu_spark.h"   /* private header file */
#include "etpu_util.h"    /* utility routines for working with the eTPU */

/*******************************************************************************
* Global variables
*******************************************************************************/
extern uint32_t fs_etpu_data_ram_start;
extern uint32_t fs_etpu_data_ram_ext;

/*******************************************************************************
* FUNCTION: fs_etpu_spark_init
****************************************************************************//*!
* @brief   This function initializes eTPU channels to run SPARK function.
*
* @note    The following actions are performed in order:
*          -# Use user-defined CPBA or allocate new eTPU DATA RAM
*          -# Write chan config registers and FM bits
*          -# Write channel parameters
*          -# Write HSR
*          -# Set channel priority
*
* @param   *p_spark_instance - This is a pointer to the instance structure
*            @ref spark_instance_t.
* @param   *p_spark_config - This is a pointer to the structure of configuration
*            parameters @ref spark_config_t.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_MALLOC - eTPU DATA RAM memory allocation error
*          - @ref FS_ETPU_ERROR_NONE - No error
*
* @warning This function does not configure the pins, only the eTPU channels.
*******************************************************************************/
uint32_t fs_etpu_spark_init(
  struct spark_instance_t   *p_spark_instance,
  struct spark_config_t     *p_spark_config)
{
  uint8_t  chan_num;
  uint8_t  priority;
  uint32_t *cpba;
  uint32_t *cpba_single_spark;
  uint8_t  spark_count;
  struct single_spark_config_t *p_single_spark_config;
  uint32_t cr;
  uint8_t  i;

  chan_num          = p_spark_instance->chan_num;
  priority          = p_spark_instance->priority;
  cpba              = p_spark_instance->cpba;
  cpba_single_spark = p_spark_instance->cpba_single_spark;

  /* Use user-defined CPBA or allocate new eTPU DATA RAM for chan. parameters */
  if(cpba == 0)
  {
    cpba = fs_etpu_malloc(FS_ETPU_SPARK_NUM_PARMS);
    if(cpba == 0)
    {
      return(FS_ETPU_ERROR_MALLOC);
    }
    else
    {
      p_spark_instance->cpba = cpba;
    }
  }
  /* Use user-defined CPBA or allocate new eTPU DATA RAM for single spark */
  spark_count = p_spark_config->spark_count;
  if(cpba_single_spark == 0)
  {
    cpba_single_spark = fs_etpu_malloc(FS_ETPU_SINGLE_SPARK_STRUCT_SIZE * spark_count);
    if(cpba_single_spark == 0)
    {
      return(FS_ETPU_ERROR_MALLOC);
    }
    else
    {
      p_spark_instance->cpba_single_spark = cpba_single_spark;
    }
  }

  /* Write chan config registers and FM bits */
  cr = (FS_ETPU_SPARK_TABLE_SELECT << 24) +
       (FS_ETPU_SPARK_FUNCTION_NUMBER << 16) +
       (((uint32_t)cpba - fs_etpu_data_ram_start) >> 3);
  eTPU->CHAN[chan_num].CR.R = cr;
  eTPU->CHAN[chan_num].SCR.R = (uint32_t)p_spark_instance->polarity;

  /* Write channel parameters */
  /* 24-bit */
  *(cpba + ((FS_ETPU_SPARK_OFFSET_TDC_ANGLE            - 1)>>2)) = p_spark_instance->tdc_angle;
  *(cpba + ((FS_ETPU_SPARK_OFFSET_TDC_ANGLE_ACTUAL     - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_SPARK_OFFSET_ANGLE_OFFSET_RECALC  - 1)>>2)) = p_spark_config->angle_offset_recalc;
  *(cpba + ((FS_ETPU_SPARK_OFFSET_DWELL_TIME_MIN       - 1)>>2)) = p_spark_config->dwell_time_min;
  *(cpba + ((FS_ETPU_SPARK_OFFSET_DWELL_TIME_MAX       - 1)>>2)) = p_spark_config->dwell_time_max;
  *(cpba + ((FS_ETPU_SPARK_OFFSET_MULTI_ON_TIME        - 1)>>2)) = p_spark_config->multi_on_time;
  *(cpba + ((FS_ETPU_SPARK_OFFSET_MULTI_OFF_TIME       - 1)>>2)) = p_spark_config->multi_off_time;
  *(cpba + ((FS_ETPU_SPARK_OFFSET_P_SINGLE_SPARK_FIRST - 1)>>2)) = (uint32_t)cpba_single_spark - fs_etpu_data_ram_start;
  *(cpba + ((FS_ETPU_SPARK_OFFSET_P_SINGLE_SPARK       - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_SPARK_OFFSET_PULSE_START_TIME     - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_SPARK_OFFSET_DWELL_TIME_APPLIED   - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_SPARK_OFFSET_DWELL_TIME           - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_SPARK_OFFSET_END_ANGLE            - 1)>>2)) = 0;
  /* 8-bit */
  *((uint8_t*)cpba + FS_ETPU_SPARK_OFFSET_SPARK_COUNT        ) = spark_count;
  *((uint8_t*)cpba + FS_ETPU_SPARK_OFFSET_SPARK_COUNTER      ) = 0;
  *((uint8_t*)cpba + FS_ETPU_SPARK_OFFSET_MULTI_PULSE_COUNT  ) = 0;
  *((uint8_t*)cpba + FS_ETPU_SPARK_OFFSET_MULTI_PULSE_COUNTER) = 0;
  *((uint8_t*)cpba + FS_ETPU_SPARK_OFFSET_STATE              ) = 0;
  *((uint8_t*)cpba + FS_ETPU_SPARK_OFFSET_ERROR              ) = 0;
  *((uint8_t*)cpba + FS_ETPU_SPARK_OFFSET_GENERATION_DISABLE ) = p_spark_config->generation_disable;

  /* Write array of single sparke array parameters */
  p_single_spark_config = p_spark_config->p_single_spark_config;
  for(i=0; i<spark_count; i++)
  {
    /* 24-bit */
    *(cpba_single_spark + ((FS_ETPU_SINGLE_SPARK_OFFSET_END_ANGLE  - 1)>>2)) = p_single_spark_config->end_angle;
    *(cpba_single_spark + ((FS_ETPU_SINGLE_SPARK_OFFSET_DWELL_TIME - 1)>>2)) = p_single_spark_config->dwell_time;
    /* 8-bit */
    *((uint8_t*)cpba_single_spark + FS_ETPU_SINGLE_SPARK_OFFSET_MULTI_PULSE_COUNT) = p_single_spark_config->multi_pulse_count;

    p_single_spark_config++;
    cpba_single_spark += FS_ETPU_SINGLE_SPARK_STRUCT_SIZE >> 2;
  }

  /* Write HSR and Set channel priority*/
  eTPU->CHAN[chan_num].HSRR.R = FS_ETPU_SPARK_HSR_INIT;
  fs_etpu_enable(chan_num, priority);

  return(FS_ETPU_ERROR_NONE);
}

/*******************************************************************************
* FUNCTION: fs_etpu_spark_config
****************************************************************************//*!
* @brief   This function changes the SPARK configuration.
*
* @note    The following actions are performed in order:
*          -# Write configuration parameter values to eTPU DATA RAM
*          -# Write HSR
*
* @warning The new single spark configurations (array of single spark structures)
*          must fit into the eTPU DATA RAM already allocated. It means the
*          spark_count can only be lower or the same as the value provided
*          on initialization.
*
* @param   *p_spark_instance - This is a pointer to the instance structure
*            @ref spark_instance_t.
* @param   *p_spark_config - This is a pointer to the structure of configuration
*            parameters @ref spark_config_t.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_NONE - No error
*          - @ref FS_ETPU_ERROR_TIMING - The mode was not set because there is
*              a HSR pending on the eTPU channel.
*              It can be caused by a call of an API function which writes
*              the HSR to the same channel shortly before this function call.
*              Try to repeat the function call several microseconds later.
*
*******************************************************************************/
uint32_t fs_etpu_spark_config(
  struct spark_instance_t *p_spark_instance,
  struct spark_config_t   *p_spark_config)
{
  uint32_t *cpba;
  uint32_t *cpba_single_spark;
  uint32_t *cpbae;
  uint8_t  spark_count;
  struct single_spark_config_t *p_single_spark_config;
  uint8_t  i;

  /* Check there is no pending HSR */
  if(eTPU->CHAN[p_spark_instance->chan_num].HSRR.R != 0)
  {
    return(FS_ETPU_ERROR_TIMING);
  }
  else
  {
    cpba              = p_spark_instance->cpba;
    cpba_single_spark = p_spark_instance->cpba_single_spark;
    spark_count       = p_spark_config->spark_count;
    cpbae = cpba + (0x4000 >> 2); /* sign-extended memory area */

    /* Write channel parameters */
    /* 24-bit */
    *(cpbae + ((FS_ETPU_SPARK_OFFSET_ANGLE_OFFSET_RECALC - 1)>>2)) = p_spark_config->angle_offset_recalc;
    *(cpbae + ((FS_ETPU_SPARK_OFFSET_DWELL_TIME_MIN      - 1)>>2)) = p_spark_config->dwell_time_min;
    *(cpbae + ((FS_ETPU_SPARK_OFFSET_DWELL_TIME_MAX      - 1)>>2)) = p_spark_config->dwell_time_max;
    *(cpbae + ((FS_ETPU_SPARK_OFFSET_MULTI_ON_TIME       - 1)>>2)) = p_spark_config->multi_on_time;
    *(cpbae + ((FS_ETPU_SPARK_OFFSET_MULTI_OFF_TIME      - 1)>>2)) = p_spark_config->multi_off_time;
    /* 8-bit */
    *((uint8_t*)cpba + FS_ETPU_SPARK_OFFSET_SPARK_COUNT       ) = spark_count;
    *((uint8_t*)cpba + FS_ETPU_SPARK_OFFSET_GENERATION_DISABLE) = p_spark_config->generation_disable;

    /* Write array of sparkection parameters */
    p_single_spark_config = p_spark_config->p_single_spark_config;
    for(i=0; i<spark_count; i++)
    {
      /* 24-bit */
      *(cpba_single_spark + ((FS_ETPU_SINGLE_SPARK_OFFSET_END_ANGLE  - 1)>>2)) = p_single_spark_config->end_angle;
      *(cpba_single_spark + ((FS_ETPU_SINGLE_SPARK_OFFSET_DWELL_TIME - 1)>>2)) = p_single_spark_config->dwell_time;
      /* 8-bit */
      *((uint8_t*)cpba_single_spark + FS_ETPU_SINGLE_SPARK_OFFSET_MULTI_PULSE_COUNT) = p_single_spark_config->multi_pulse_count;

      p_single_spark_config++;
      cpba_single_spark += FS_ETPU_SINGLE_SPARK_STRUCT_SIZE >> 2;
    }

    /* Write HSR and Set channel priority*/
    eTPU->CHAN[p_spark_instance->chan_num].HSRR.R = FS_ETPU_SPARK_HSR_UPDATE;

    return(FS_ETPU_ERROR_NONE);
  }
}

/*******************************************************************************
* FUNCTION: fs_etpu_spark_get_states
****************************************************************************//*!
* @brief   This function reads SPARK state variables, including error flags,
*          and clears the eTPU error after reading.
*
* @note    The following actions are performed in order:
*          -# Read state parameter values from eTPU DATA RAM
*          -# Clear SPARK error
*
* @param   *p_spark_instance - This is a pointer to the instance structure
*            @ref inj_instance_t.
* @param   *p_spark_states - This is a pointer to the return structure of states
*            @ref inj_states_t.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_NONE - No error
*
*******************************************************************************/
uint32_t fs_etpu_spark_get_states(
  struct spark_instance_t *p_spark_instance,
  struct spark_states_t   *p_spark_states)
{
  uint32_t *cpba;
  uint32_t *cpbae;

  cpba  = p_spark_instance->cpba;
  cpbae = cpba + (0x4000 >> 2); /* sign-extended memory area */

  /* Read SPARK channel parameters */
  p_spark_states->dwell_time_applied = *(cpbae + ((FS_ETPU_SPARK_OFFSET_DWELL_TIME_APPLIED - 1)>>2));
  p_spark_states->error             |= *((uint8_t*)cpba + FS_ETPU_SPARK_OFFSET_ERROR);
  /* Clear SPARK error */
  *((uint8_t*)cpba + FS_ETPU_SPARK_OFFSET_ERROR) = 0;

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
 * Revision 0.1  2013/09/17  r54529
 * Initial version of file.
 ******************************************************************************/