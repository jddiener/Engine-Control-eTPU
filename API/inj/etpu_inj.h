/*******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2014 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
****************************************************************************//*!
*
* @file    etpu_inj.h
*
* @author  Milan Brejl [r54529]
*
* @version 1.0
*
* @date    17-Mar-2014
*
* @brief   This file contains useful macros and prototypes for INJ API.
*
*******************************************************************************/
#ifndef _ETPU_INJ_H_
#define _ETPU_INJ_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include "etpu_util.h"        /* 24-bit types */
#include "etpu_inj_auto.h"    /* auto generated header file */

/*******************************************************************************
* Definitions
*******************************************************************************/
/** Unused BANK channel number */
#define FS_ETPU_INJ_BANK_CHAN_NOT_USED    0xFF

/** Injection phase configuration */
#define FS_ETPU_INJ_PHASE_DURATION_MASK   0x00FFFFFF
#define FS_ETPU_INJ_PHASE_OUT_LOW         0x00000000
#define FS_ETPU_INJ_PHASE_OUT_HIGH_INJ    0x01000000
#define FS_ETPU_INJ_PHASE_OUT_HIGH_BANK_1 0x02000000
#define FS_ETPU_INJ_PHASE_OUT_HIGH_BANK_2 0x04000000
#define FS_ETPU_INJ_PHASE_OUT_HIGH_BANK_3 0x08000000
#define FS_ETPU_INJ_PHASE_DMA_INJ         0x10000000
#define FS_ETPU_INJ_PHASE_DMA_IRQ_BANK_1  0x20000000  /* eTPU2 only */
#define FS_ETPU_INJ_PHASE_DMA_IRQ_BANK_2  0x40000000  /* eTPU2 only */
#define FS_ETPU_INJ_PHASE_DMA_IRQ_BANK_3  0x80000000  /* eTPU2 only */


/*******************************************************************************
* Type Definitions
*******************************************************************************/
/** A structure to represent an instance of INJ.
 *  It includes static INJ initialization items. */
struct inj_instance_t
{
  const uint8_t  chan_num_inj;  /**< Channel number of the INJ channel. */
  const uint8_t  chan_num_bank_1;  /**< Channel number of the 1st BANK channel.
    Assign a chanel number or FS_ETPU_INJ_BANK_CHAN_NOT_USED */
  const uint8_t  chan_num_bank_2;  /**< Channel number of the 2nd BANK channel.
    Assign a chanel number or FS_ETPU_INJ_BANK_CHAN_NOT_USED */
  const uint8_t  chan_num_bank_3;  /**< Channel number of the 3rd BANK channel.
    Assign a chanel number or FS_ETPU_INJ_BANK_CHAN_NOT_USED */
  const uint8_t  priority;  /**< Channel priority for the INJ channel. */
  const uint8_t  polarity_inj;  /**< INJ polarity, applies to INJ channel.
    It can one any of:
    - @ref FS_ETPU_INJ_FM0_ACTIVE_HIGH - active output signal state is high
    - @ref FS_ETPU_INJ_FM0_ACTIVE_LOW - active output signal state is low */
  const uint8_t  polarity_bank;  /**< INJ polarity, applies to all BANK
    channels. It can one any of:
    - @ref FS_ETPU_INJ_FM0_ACTIVE_HIGH - active output signal state is high
    - @ref FS_ETPU_INJ_FM0_ACTIVE_LOW - active output signal state is low */
  const uint24_t  tdc_angle; /**< The cylinder Top Dead Center as a number
    of TCR2 angle ticks relative to zero engine angle, in a range corresponding
    to 0-720 degrees. */
        uint32_t *cpba;     /**< Channel parameter base address.
    Set cpba = 0 to use automatic allocation of eTPU DATA RAM for INJ channel
    parameters using the eTPU utility function fs_etpu_malloc (recommanded),
    or assign the cpba manually by an address where the INJ channel parameter
    space will start from, e.g. 0xC3FC8100. */
        uint32_t *cpba_injections; /**< Base address of the injections array in
    eTPU DATA RAM. Set cpba_injections = 0 to use automatic allocation of the
    eTPU DATA RAM space corresponding to the injection_count value,
    using the eTPU utility function fs_etpu_malloc (recommanded),
    or assign the cpba_injections manually by an address, e.g. 0xC3FC8100. */
        uint32_t *cpba_phases; /**< Base address of the injection phase array in
    eTPU DATA RAM. Set cpba_phase = 0 to use automatic allocation of eTPU DATA
    RAM space corresponding to the phase_count value, using the eTPU utility
    function fs_etpu_malloc (recommanded),
    or assign the cpba_phases manually by an address, e.g. 0xC3FC8100. */
};


/** A structure to represent a configuration of INJ.
 *  It includes INJ configuration items which can mostly be changed in
 *  run-time. */
struct inj_config_t
{
   int24_t angle_irq;  /**< The tdc_angle-relative IRQ angle as a
    number of TCR2 ticks. Positive values precede the TDC, negative go after.
    The INJ channel interrupt is generated in order to enable the CPU to
    update the injection parameters before the first injection start-angle. */
   int24_t angle_stop;  /**< The tdc_angle-relative injection stop angle as a
    number of TCR2 ticks. Positive values precede the TDC, negative go after.
    In case the whole injection sequence has not finieshed, it is stopped
    at this angle by setting the INJ and BANK channels to inactive state. */
  uint8_t  injection_count;  /**< The count of injections. */
  struct inj_injection_config_t *p_injection_config; /**< Pointer to the first
    item of an array of the injection configuration structures. */
};


/** A structure to represent a single injection configuration. */
struct inj_injection_config_t
{
   int24_t angle_start;  /**< The tdc_angle-relative injection start angle as a
    number of TCR2 ticks. Positive values precede the TDC, negative go after. */
  uint8_t  phase_count;  /**< The count of injection phases. */
  uint32_t *p_phase_config; /**< Pointer to the first item of in an array
    of injection phase configuration 32-bit words.
    Use the following definitions to set a single phase configuration word:
      FS_ETPU_INJ_PHASE_DURATION_MASK    - 24-bit mask for phase duration set
                                           as a number of TCR1 ticks.
      FS_ETPU_INJ_PHASE_OUT_LOW          - output low on an INJ or BANK channel
      FS_ETPU_INJ_PHASE_OUT_HIGH_INJ     - output high on the INJ channel
      FS_ETPU_INJ_PHASE_OUT_HIGH_BANK_1  - output high on the BANK 1 channel
      FS_ETPU_INJ_PHASE_OUT_HIGH_BANK_2  - output high on the BANK 2 channel
      FS_ETPU_INJ_PHASE_OUT_HIGH_BANK_3  - output high on the BANK 3 channel
      FS_ETPU_INJ_PHASE_DMA_INJ          - generate DMA request on the INJ
                                           channel at the phase start
      The following applie to eTPU2 only:
      FS_ETPU_INJ_PHASE_DMA_IRQ_BANK_1   - generate DMA and IRQ request on the
                                           BANK 1 channel at the phase start
      FS_ETPU_INJ_PHASE_DMA_IRQ_BANK_2   - generate DMA and IRQ request on the
                                           BANK 2 channel at the phase start
      FS_ETPU_INJ_PHASE_DMA_IRQ_BANK_3   - generate DMA and IRQ request on the
                                           BANK 3 channel at the phase start */
};


/** A structure to represent states of INJ. */
struct inj_states_t
{
  uint8_t error;      /**< This is the error status of CRANK. It includes
    the following error flags:
    - @ref FS_ETPU_INJ_ERROR_PREV_INJ_NOT_FINISHED
    - @ref FS_ETPU_INJ_ERROR_LATE_START_ANGLE_1ST
    - @ref FS_ETPU_INJ_ERROR_LATE_START_ANGLE_NTH
    - @ref FS_ETPU_INJ_ERROR_STOPPED_BY_STOP_ANGLE
    The eTPU sets the error flags, the CPU clears them after reading. */
  uint8_t injection_idx;  /**< This is the index of the actual injection.
    It can be 1 to num_injections in case an injection is active, or 0 in case
    no injection is active. */
  uint8_t phase_idx; /**< This is the index of the actual injection phase.
    It can be 1 to num_phases in case an injection phase is active, or 0 in case
    no injection phase is active. */
};


/*******************************************************************************
* Function prototypes
*******************************************************************************/
/* Initialize */
uint32_t fs_etpu_inj_init(
  struct inj_instance_t   *p_inj_instance,
  struct inj_config_t     *p_inj_config);

/* Change configuration */
uint32_t fs_etpu_inj_config(
  struct inj_instance_t *p_inj_instance,
  struct inj_config_t   *p_inj_config);

/* Get states */
uint32_t fs_etpu_inj_get_states(
  struct inj_instance_t *p_inj_instance,
  struct inj_states_t   *p_inj_states);

#endif /* _ETPU_INJ_H_ */
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
 * fs_etpu_inj_update() removed.
 * Separate polarity for Inj and Bank channels.
 *
 * Revision 0.1  2012/11/28  r54529
 * Initial version of file.
 ******************************************************************************/