/*******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2014 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
****************************************************************************//*!
*
* @file    etpu_knock.c
*
* @author  Milan Brejl [r54529]
*
* @version 1.0
*
* @date    17-Mar-2014
*
* @brief   This file contains API for using the eTPU function
*          Knock (KNOCK).
*
****************************************************************************//*!
*
* @mainpage
*
* The eTPU KNOCK APIs @ref etpu_knock.c/.h includes API functions for eTPU
* function Knock.
*
* Each instance of the KNOCK eTPU function controls a single KNOCK channel.
*
* This eTPU function is intended to support ADC sampling of a knock signal
* in an engine control system.
* The function has 2 modes:
* - Gate Mode
* - Trigger Mode
*
* In the Gate Mode a simple angle-based pulses are generated. The output
* signal can be used to gate the ADC running in continues mode.
* In the Trigger Mode a 50% duty-cycle PWM signal is generated within
* the angle-based window. The output signal can be used to trigger the ADC.
*
* There is an KNOCK parameter tdc_angle, relative to which all windows are
* defined. Positive angles precede the tdc_angle, negative angles come after.
*
* The number of angle-based windows is configurable. The windows are
* defined by an array of window structures, consisting of TDC-relative
* window start angle and angular windows width.
*
* The KNOCK function enables to selectively generate channel interrupts
* and/or DMA requests at:
* - window start
* - window end
* - every trigger pulse (Trigger mode only)
*
*******************************************************************************/
/*******************************************************************************
* Includes
*******************************************************************************/
#include "etpu_knock.h"   /* private header file */
#include "etpu_util.h"    /* utility routines for working with the eTPU */

/*******************************************************************************
* Global variables
*******************************************************************************/
extern uint32_t fs_etpu_data_ram_start;
extern uint32_t fs_etpu_data_ram_ext;

/*******************************************************************************
* FUNCTION: fs_etpu_knock_init
****************************************************************************//*!
* @brief   This function initializes eTPU channels to run KNOCK function.
*
* @note    The following actions are performed in order:
*          -# Use user-defined CPBA or allocate new eTPU DATA RAM
*          -# Write chan config registers and FM bits
*          -# Write channel parameters
*          -# Write HSR
*          -# Set channel priority
*
* @param   *p_knock_instance - This is a pointer to the instance structure
*            @ref knock_instance_t.
* @param   *p_knock_config - This is a pointer to the structure of configuration
*            parameters @ref knock_config_t.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_MALLOC - eTPU DATA RAM memory allocation error
*          - @ref FS_ETPU_ERROR_NONE - No error
*
* @warning This function does not configure the pins, only the eTPU channels.
*******************************************************************************/
uint32_t fs_etpu_knock_init(
  struct knock_instance_t   *p_knock_instance,
  struct knock_config_t     *p_knock_config)
{
  uint8_t  chan_num;
  uint8_t  priority;
  uint32_t *cpba;
  uint32_t *cpba_windows;
  uint8_t  window_count;
  struct knock_window_config_t *p_knock_window_config;
  uint32_t cr;
  uint8_t  i;

  chan_num        = p_knock_instance->chan_num;
  priority        = p_knock_instance->priority;
  cpba            = p_knock_instance->cpba;
  cpba_windows    = p_knock_instance->cpba_windows;

  /* Use user-defined CPBA or allocate new eTPU DATA RAM for chan. parameters */
  if(cpba == 0)
  {
    cpba = fs_etpu_malloc(FS_ETPU_KNOCK_NUM_PARMS);
    if(cpba == 0)
    {
      return(FS_ETPU_ERROR_MALLOC);
    }
    else
    {
      p_knock_instance->cpba = cpba;
    }
  }
  /* Use user-defined CPBA or allocate new eTPU DATA RAM for knock windows */
  window_count = p_knock_config->window_count;
  if(cpba_windows == 0)
  {
    cpba_windows =
      fs_etpu_malloc(FS_ETPU_KNOCK_WINDOW_STRUCT_SIZE * window_count);
    if(cpba_windows == 0)
    {
      return(FS_ETPU_ERROR_MALLOC);
    }
    else
    {
      p_knock_instance->cpba_windows = cpba_windows;
    }
  }

  /* Write chan config registers and FM bits */
  cr = (FS_ETPU_KNOCK_TABLE_SELECT << 24) +
       (FS_ETPU_KNOCK_FUNCTION_NUMBER << 16) +
       (((uint32_t)cpba - fs_etpu_data_ram_start) >> 3);
  eTPU->CHAN[chan_num].CR.R = cr;
  eTPU->CHAN[chan_num].SCR.R = (uint32_t)p_knock_instance->polarity
                                       + p_knock_config->mode;

  /* Write channel parameters */
  /* 24-bit */
  *(cpba + ((FS_ETPU_KNOCK_OFFSET_P_WINDOW_FIRST    - 1)>>2)) = (uint32_t)cpba_windows - fs_etpu_data_ram_start;
  *(cpba + ((FS_ETPU_KNOCK_OFFSET_P_WINDOW          - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_KNOCK_OFFSET_TDC_ANGLE         - 1)>>2)) = p_knock_instance->tdc_angle;
  *(cpba + ((FS_ETPU_KNOCK_OFFSET_TDC_ANGLE_ACTUAL  - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_KNOCK_OFFSET_TCR2_WINDOW_START - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_KNOCK_OFFSET_TCR2_WINDOW_END   - 1)>>2)) = 0;
  *(cpba + ((FS_ETPU_KNOCK_OFFSET_TRIGGER_PERIOD    - 1)>>2)) = p_knock_config->trigger_period;
  /* 8-bit */
  *((uint8_t*)cpba + FS_ETPU_KNOCK_OFFSET_WINDOW_COUNT      ) = window_count;
  *((uint8_t*)cpba + FS_ETPU_KNOCK_OFFSET_WINDOW_COUNTER    ) = 0;
  *((uint8_t*)cpba + FS_ETPU_KNOCK_OFFSET_IRQ_DMA_OPTIONS   ) = p_knock_config->irq_dma_options;

  /* Write array of knockection parameters */
  p_knock_window_config = p_knock_config->p_knock_window_config;
  for(i=0; i<window_count; i++)
  {
    /* 24-bit */
    *(cpba_windows + ((FS_ETPU_KNOCK_WINDOW_OFFSET_START - 1)>>2)) = p_knock_window_config->angle_start;
    *(cpba_windows + ((FS_ETPU_KNOCK_WINDOW_OFFSET_WIDTH - 1)>>2)) = p_knock_window_config->angle_width;

    p_knock_window_config++;
    cpba_windows += FS_ETPU_KNOCK_WINDOW_STRUCT_SIZE >> 2;
  }

  /* Write HSR and Set channel priority*/
  eTPU->CHAN[chan_num].HSRR.R = FS_ETPU_KNOCK_HSR_INIT;
  fs_etpu_enable(chan_num, priority);

  return(FS_ETPU_ERROR_NONE);
}

/*******************************************************************************
* FUNCTION: fs_etpu_knock_config
****************************************************************************//*!
* @brief   This function changes the KNOCK configuration.
*
* @note    The following actions are performed in order:
*          -# Write FM bit.
*          -# Write configuration parameter values to eTPU DATA RAM
*
* @warning The new knock window configuration (array of windows)
*          must fit into the eTPU DATA RAM already allocated. It means the
*          windows_count can only be lower or the same as the value provided
*          on initialization.
*
* @param   *p_knock_instance - This is a pointer to the instance structure
*            @ref knock_instance_t.
* @param   *p_knock_config - This is a pointer to the structure of configuration
*            parameters @ref knock_config_t.
*
* @return  Error codes that can be returned are:
*          - @ref FS_ETPU_ERROR_NONE - No error
*
*******************************************************************************/
uint32_t fs_etpu_knock_config(
  struct knock_instance_t *p_knock_instance,
  struct knock_config_t   *p_knock_config)
{
  uint32_t *cpba;
  uint32_t *cpba_windows;
  uint32_t *cpbae;
  struct knock_window_config_t *p_knock_window_config;
  uint8_t  i;

  cpba            = p_knock_instance->cpba;
  cpba_windows    = p_knock_instance->cpba_windows;

  /* Write chan config registers and FM bits */
  eTPU->CHAN[p_knock_instance->chan_num].SCR.B.FM1 = p_knock_config->mode >> 1;

  /* Write channel parameters */
  /* 24-bit - use cpbae to prevent from overwriting bits 31:24 */
  cpbae = cpba + (0x4000 >> 2); /* sign-extended memory area */
  *(cpbae + ((FS_ETPU_KNOCK_OFFSET_TRIGGER_PERIOD - 1)>>2)) = p_knock_config->trigger_period;
  /* 8-bit */
  *((uint8_t*)cpba + FS_ETPU_KNOCK_OFFSET_IRQ_DMA_OPTIONS)  = p_knock_config->irq_dma_options;
  *((uint8_t*)cpba + FS_ETPU_KNOCK_OFFSET_WINDOW_COUNT)     = p_knock_config->window_count;

  /* Write array of knock window parameters */
  p_knock_window_config = p_knock_config->p_knock_window_config;
  for(i=0; i<p_knock_config->window_count; i++)
  {
    /* 24-bit */
    *(cpba_windows + ((FS_ETPU_KNOCK_WINDOW_OFFSET_START - 1)>>2)) = p_knock_window_config->angle_start;
    *(cpba_windows + ((FS_ETPU_KNOCK_WINDOW_OFFSET_WIDTH - 1)>>2)) = p_knock_window_config->angle_width;

    p_knock_window_config++;
    cpba_windows += FS_ETPU_KNOCK_WINDOW_STRUCT_SIZE >> 2;
  }

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
 * Revision 0.1  2013/09/09  r54529
 * Initial version of file.
 ******************************************************************************/