/*******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2014 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
****************************************************************************//*!
*
* @file    etpu_knock.h
*
* @author  Milan Brejl [r54529]
*
* @version 1.0
*
* @date    17-Mar-2014
*
* @brief   This file contains useful macros and prototypes for KNOCK API.
*
*******************************************************************************/
#ifndef _ETPU_KNOCK_H_
#define _ETPU_KNOCK_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include "etpu_util.h"        /* 24-bit types */
#include "etpu_knock_auto.h"    /* auto generated header file */

/*******************************************************************************
* Definitions
*******************************************************************************/

/*******************************************************************************
* Type Definitions
*******************************************************************************/
/** A structure to represent an instance of KNOCK.
 *  It includes static KNOCK initialization items. */
struct knock_instance_t
{
  const uint8_t  chan_num;  /**< Channel number of the KNOCK channel. */
  const uint8_t  priority;  /**< Channel priority for the KNOCK channel. */
  const uint8_t  polarity;  /**< KNOCK polarity. It can one any of:
    - @ref FS_ETPU_KNOCK_FM0_ACTIVE_HIGH - active output signal state is high
    - @ref FS_ETPU_KNOCK_FM0_ACTIVE_LOW - active output signal state is low */
  const uint24_t  tdc_angle; /**< The cylinder Top Dead Center as a number
    of TCR2 angle ticks relative to zero engine angle, in a range corresponding
    to 0-720 degrees. */
        uint32_t *cpba;     /**< Channel parameter base address.
    Set cpba = 0 to use automatic allocation of eTPU DATA RAM for KNOCK channel
    parameters using the eTPU utility function fs_etpu_malloc (recommanded),
    or assign the cpba manually by an address where the KNOCK channel parameter
    space will start from, e.g. 0xC3FC8100. */
        uint32_t *cpba_windows; /**< Base address of the knock windows array in
    eTPU DATA RAM. Set cpba_windows = 0 to use automatic allocation of the
    eTPU DATA RAM space corresponding to the window_count value,
    using the eTPU utility function fs_etpu_malloc (recommanded),
    or assign the cpba_windows manually by an address, e.g. 0xC3FC8100. */
};


/** A structure to represent a configuration of KNOCK.
 *  It includes KNOCK configuration items which can be changed in run-time. */
struct knock_config_t
{
  uint8_t  mode;  /**< KNOCK mode. It can one of:
    - @ref FS_ETPU_KNOCK_FM1_MODE_GATE
    - @ref FS_ETPU_KNOCK_FM1_MODE_TRIGGER */
  uint8_t  window_count;  /**< The count of knock windows. */
  struct knock_window_config_t *p_knock_window_config; /**< Pointer to the first
    item of an array of the knockection configuration structures. */
   int24_t trigger_period;  /**< The trigger signal (50% duty-cycle PWM) period
    as a number of TCR1 ticks. */
  uint8_t  irq_dma_options;  /**< Enables to generate channel interrupt and/or
    DMA requests at selectable moments.
    It can be assigned none, one, or more values from:
    - @ref FS_ETPU_KNOCK_IRQ_AT_WINDOW_START
    - @ref FS_ETPU_KNOCK_IRQ_AT_WINDOW_END
    - @ref FS_ETPU_KNOCK_IRQ_AT_EVERY_TRIGGER
    - @ref FS_ETPU_KNOCK_DMA_AT_WINDOW_START
    - @ref FS_ETPU_KNOCK_DMA_AT_WINDOW_END
    - @ref FS_ETPU_KNOCK_DMA_AT_EVERY_TRIGGER */
};


/** A structure to represent a single knock window configuration. */
struct knock_window_config_t
{
   int24_t angle_start;  /**< The tdc_angle-relative window start angle as a
    number of TCR2 ticks. Positive values precede the TDC, negative go after. */
   int24_t angle_width;  /**< The window width as a number of TCR2 ticks. */
};


/*******************************************************************************
* Function prototypes
*******************************************************************************/
/* Initialize */
uint32_t fs_etpu_knock_init(
  struct knock_instance_t   *p_knock_instance,
  struct knock_config_t     *p_knock_config);

/* Change configuration */
uint32_t fs_etpu_knock_config(
  struct knock_instance_t *p_knock_instance,
  struct knock_config_t   *p_knock_config);

#endif /* _ETPU_KNOCK_H_ */
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