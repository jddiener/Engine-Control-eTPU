/*******************************************************************************
* ASH WARE Inc.
* This file of eTPU utility functions has been updated to generically support
* all eTPU modules on multi-eTPU-module chips (e.g. MPC5777C).
*******************************************************************************/

/**************************************************************************
* FILE NAME: etpu_util_ext.h
* 
* DESCRIPTION: declares utility API for interacting with the eTPU module
*
*
*========================================================================
* REV      AUTHOR      DATE        DESCRIPTION OF CHANGE                 
* ---   -----------  ----------    ---------------------                 
* 1.0     J Diener   01/Jun/20     Initial version.     
*
**************************************************************************/

#ifndef _ETPU_UTIL_EXT_H_
#define _ETPU_UTIL_EXT_H_

#include "typedefs.h"     /* standard types */
#include "etpu_struct.h"  /* eTPU module structure definition */
#include "etpu_util.h"

#ifdef __cplusplus
extern "C" {
#endif
    
/*******************************************************************************
* Macros
*******************************************************************************/
/***************************************************************************//*!
* @brief   Channel number expressed by (engine, channel)
* @note    All channel numbers for both eTPUs are absolute.
*          eTPU_A has channels 0-31 and eTPU_B has channels 64-95.
*******************************************************************************/
#define FS_ETPU_ENGINE_CHANNEL(x,y) (((x)-1)*64 + y)

/***************************************************************************//*!
* @brief   Transformation from absolute channel number to link register encoding
*******************************************************************************/
#define FS_ETPU_CHANNEL_TO_LINK(x)  ((x)+64)

#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/*******************************************************************************
* Global variables
*******************************************************************************/
/** @brief   Pointer to the first free parameter in eTPU DATA RAM */
extern uint32_t *fs_etpu_free_param;
extern uint32_t *fs_etpu_c_free_param;

/** @brief   Access to eTPU registers' structure */
extern volatile struct eTPU_struct * const eTPU_AB;
extern volatile struct eTPU_struct * const eTPU_C;

extern const uint32_t fs_etpu_data_ram_start;
extern const uint32_t fs_etpu_data_ram_end;
extern const uint32_t fs_etpu_data_ram_ext;
extern const uint32_t fs_etpu_c_data_ram_start;
extern const uint32_t fs_etpu_c_data_ram_end;
extern const uint32_t fs_etpu_c_data_ram_ext;

/* etpu timing configuration data */
extern const uint32_t etpu_a_tcr1_freq;
extern const uint32_t etpu_a_tcr2_freq;
extern const uint32_t etpu_b_tcr1_freq;
extern const uint32_t etpu_b_tcr2_freq;
extern const uint32_t etpu_c_tcr1_freq;
extern const uint32_t etpu_c_tcr2_freq;

/*******************************************************************************
* Type Definitions
*******************************************************************************/

typedef enum
{
	EM_AB,
	EM_C,
} ETPU_MODULE;


/*******************************************************************************
* Function prototypes
*******************************************************************************/
/* eTPU Module Initialization */
uint32_t fs_etpu_init_ext(
  ETPU_MODULE em,
  struct etpu_config_t *p_etpu_config,
  uint32_t *code,
  uint32_t code_size,
  uint32_t *globals,
  uint32_t globals_size);

uint32_t fs_etpu2_init_ext(
  ETPU_MODULE em,
  struct etpu_config_t *p_etpu_config,
  uint32_t engine_mem_size);

/* eTPU Channel Initialization */
uint32_t *fs_etpu_chan_init_ext(
  ETPU_MODULE em,
  uint8_t channel,
  uint8_t function,
  uint8_t mode,
  uint8_t hsr,
  uint8_t num_param,
  uint32_t config,
  uint32_t *func_frame);

uint32_t *fs_etpu_malloc_ext(
  ETPU_MODULE em,
  uint16_t num_bytes);
uint32_t *fs_etpu_malloc2_ext(
  ETPU_MODULE em,
  uint8_t channel,
  uint16_t num_bytes);

/* Run-Time eTPU Module Control */
void fs_timer_start_ext(
  ETPU_MODULE em);
uint32_t fs_etpu_get_global_exceptions_ext(
  ETPU_MODULE em);
void fs_etpu_clear_global_exceptions_ext(
  ETPU_MODULE em);
uint32_t fs_etpu_get_global_error_ext(
  ETPU_MODULE em);
void fs_etpu_set_interrupt_mask_a_ext(
  ETPU_MODULE em,
  uint32_t mask);
void fs_etpu_set_interrupt_mask_b_ext(
  ETPU_MODULE em,
  uint32_t mask);
void fs_etpu_set_dma_mask_a_ext(
  ETPU_MODULE em,
  uint32_t mask);
void fs_etpu_set_dma_mask_b_ext(
  ETPU_MODULE em,
  uint32_t mask);
void fs_etpu_set_output_disable_mask_a_ext(
  ETPU_MODULE em,
  uint32_t mask,
  uint32_t polarity);
void fs_etpu_set_output_disable_mask_b_ext(
  ETPU_MODULE em,
  uint32_t mask,
  uint32_t polarity);

/* Run-Time eTPU Channel Control */
uint8_t fs_etpu_get_hsr_ext(
  ETPU_MODULE em,
  uint8_t channel);
void fs_etpu_set_hsr_ext(
  ETPU_MODULE em,
  uint8_t channel,
  uint8_t hsr);

void fs_etpu_enable_ext(
  ETPU_MODULE em,
  uint8_t channel,
  uint8_t priority);
void fs_etpu_disable_ext(
  ETPU_MODULE em,
  uint8_t channel);

void fs_etpu_interrupt_enable_ext(
  ETPU_MODULE em,
  uint8_t channel);
void fs_etpu_interrupt_disable_ext(
  ETPU_MODULE em,
  uint8_t channel);

uint8_t fs_etpu_get_chan_interrupt_flag_ext(
  ETPU_MODULE em,
  uint8_t channel);
void fs_etpu_clear_chan_interrupt_flag_ext(
  ETPU_MODULE em,
  uint8_t channel);

uint8_t fs_etpu_get_chan_interrupt_overflow_flag_ext(
  ETPU_MODULE em,
  uint8_t channel);
void fs_etpu_clear_chan_interrupt_overflow_flag_ext(
  ETPU_MODULE em,
  uint8_t channel);

void fs_etpu_dma_enable_ext(
  ETPU_MODULE em,
  uint8_t channel);
void fs_etpu_dma_disable_ext(
  ETPU_MODULE em,
  uint8_t channel);

uint8_t fs_etpu_get_chan_dma_flag_ext(
  ETPU_MODULE em,
  uint8_t channel);
void fs_etpu_clear_chan_dma_flag_ext(
  ETPU_MODULE em,
  uint8_t channel);

uint8_t fs_etpu_get_chan_dma_overflow_flag_ext(
  ETPU_MODULE em,
  uint8_t channel);
void fs_etpu_clear_chan_dma_overflow_flag_ext(
  ETPU_MODULE em,
  uint8_t channel);

/* eTPU DATA RAM Access */
uint32_t *fs_etpu_get_cpba_ext( /* equivalent to fs_etpu_data_ram() in base util API */
  ETPU_MODULE em,
  uint8_t channel);

uint32_t *fs_etpu_get_cpba_pse_ext( /* equivalent to fs_etpu_data_ram_ext() in base util API */
  ETPU_MODULE em,
  uint8_t channel);

uint32_t fs_etpu_get_chan_local_32_ext(
  ETPU_MODULE em,
  uint8_t channel,
  uint32_t offset);
uint24_t fs_etpu_get_chan_local_24_ext(
  ETPU_MODULE em,
  uint8_t channel,
  uint32_t offset);
int24_t fs_etpu_get_chan_local_24s_ext(
  ETPU_MODULE em,
  uint8_t channel,
  uint32_t offset);
uint16_t fs_etpu_get_chan_local_16_ext(
  ETPU_MODULE em,
  uint8_t channel,
  uint32_t offset);
uint8_t  fs_etpu_get_chan_local_8_ext(
  ETPU_MODULE em,
  uint8_t channel,
  uint32_t offset);

void fs_etpu_set_chan_local_32_ext(
  ETPU_MODULE em,
  uint8_t channel,
  uint32_t offset,
  uint32_t value);
void fs_etpu_set_chan_local_24_ext(
  ETPU_MODULE em,
  uint8_t channel,
  uint32_t offset,
  uint24_t value);
void fs_etpu_set_chan_local_16_ext(
  ETPU_MODULE em,
  uint8_t channel,
  uint32_t offset,
  uint16_t value);
void fs_etpu_set_chan_local_8_ext(
  ETPU_MODULE em,
  uint8_t channel,
  uint32_t offset,
  uint8_t value);

uint32_t fs_etpu_get_global_32_ext(
  ETPU_MODULE em,
  uint32_t offset);
int24_t fs_etpu_get_global_24s_ext(
  ETPU_MODULE em,
  uint32_t offset);
uint24_t fs_etpu_get_global_24_ext(
  ETPU_MODULE em,
  uint32_t offset);
uint16_t fs_etpu_get_global_16_ext(
  ETPU_MODULE em,
  uint32_t offset);
uint8_t  fs_etpu_get_global_8_ext(
  ETPU_MODULE em,
  uint32_t offset);

void fs_etpu_set_global_32_ext(
  ETPU_MODULE em,
  uint32_t offset,
  uint32_t value);
void fs_etpu_set_global_24_ext(
  ETPU_MODULE em,
  uint32_t offset,
  uint24_t value);
void fs_etpu_set_global_16_ext(
  ETPU_MODULE em,
  uint32_t offset,
  uint16_t value);
void fs_etpu_set_global_8_ext(
  ETPU_MODULE em,
  uint32_t offset,
  uint8_t value);

uint32_t fs_etpu_coherent_read_24_ext(
  ETPU_MODULE em,
  uint8_t channel,
  uint32_t offset1,
  uint32_t offset2,
  int32_t *value1,
  int32_t *value2);
uint32_t fs_etpu_coherent_read_32_ext(
  ETPU_MODULE em,
  uint8_t channel,
  uint32_t offset1,
  uint32_t offset2,
  uint32_t *value1,
  uint32_t *value2);
uint32_t fs_etpu_coherent_write_24_ext(
  ETPU_MODULE em,
  uint8_t channel,
  uint32_t offset1,
  uint32_t offset2,
  int32_t value1,
  int32_t value2);
uint32_t fs_etpu_coherent_write_32_ext(
  ETPU_MODULE em,
  uint8_t channel,
  uint32_t offset1,
  uint32_t offset2,
  uint32_t value1,
  uint32_t value2);

/* eTPU Load Evaluation */
uint24_t fs_etpu_get_idle_cnt_a_ext(
  ETPU_MODULE em);
uint24_t fs_etpu_get_idle_cnt_b_ext(
  ETPU_MODULE em);
void fs_etpu_clear_idle_cnt_a_ext(
  ETPU_MODULE em);
void fs_etpu_clear_idle_cnt_b_ext(
  ETPU_MODULE em);

/* Others */
uint32_t *fs_memcpy32_ext(
  uint32_t *dest,
  uint32_t *source,
  uint32_t size);
void fs_memset32_ext(
  uint32_t *start,
  uint32_t value,
  int32_t size);


/*******************************************************************************
* Definition of Terms
*******************************************************************************/
/* Channel Priority values used by several function APIs */
#define FS_ETPU_PRIORITY_HIGH         0x3
#define FS_ETPU_PRIORITY_MIDDLE       0x2
#define FS_ETPU_PRIORITY_LOW          0x1
#define FS_ETPU_PRIORITY_DISABLE      0x0

#define FS_ETPU_PIN_HIGH 1
#define FS_ETPU_PIN_LOW  0

/* eTPU timebases */
#define FS_ETPU_TCR1   0
#define FS_ETPU_TCR2   1
#define FS_ETPU_TCR1_A 0
#define FS_ETPU_TCR2_A 1
#define FS_ETPU_TCR1_B 0
#define FS_ETPU_TCR2_B 1

/* chan link helper macros */
#define FS_ETPU_LINK_ETPU_A(x)     ((x & 0x1f) | 0x40) 
#define FS_ETPU_LINK_ETPU_B(x)     ((x & 0x1f) | 0x80)
#define FS_ETPU_LINK_THIS_ETPU(x)  (x & 0x1f)
#define FS_ETPU_LINK_OTHER_ETPU(x) ((x & 0x1f) | 0xC0)

/* MCR - Module Configuration Register */
#define FS_ETPU_GLOBAL_EXCEPTION_CLEAR 0x80000000 /* Global Exception Clear */

#define FS_ETPU_MISC_COMPLETE_CLEAR    0x00000800 /* MISC Complete Clear (eTPU2-only) */
#define FS_ETPU_MISC_ENABLE            0x00000200 /* Code RAM MISC Enable */
#define FS_ETPU_MISC_DISABLE           0x00000000

#define FS_ETPU_VIS_ON                 0x00000040 /* Code RAM Visibility */
#define FS_ETPU_VIS_OFF                0x00000000

#define FS_ETPU_GLOBAL_TIMEBASE_ENABLE  0x00000001 /* Global Time Base Enable */
#define FS_ETPU_GLOBAL_TIMEBASE_DISABLE 0x00000000

#define FS_ETPU_SDM_READ_ERROR         0x40000000  /* eTPU2 only */
#define FS_ETPU_WATCHDOG_TIMEOUT_A     0x20000000  /* eTPU2 only */
#define FS_ETPU_WATCHDOG_TIMEOUT_B     0x10000000  /* eTPU2 only */
#define FS_ETPU_MICROCODE_GLOBAL_EX_A  0x08000000
#define FS_ETPU_MICROCODE_GLOBAL_EX_B  0x04000000
#define FS_ETPU_ILLEGAL_INSTRUCTION_A  0x02000000
#define FS_ETPU_ILLEGAL_INSTRUCTION_B  0x01000000
#define FS_ETPU_SHARED_SUBSYS_ACC_ERR  0x00800000  /* eTPU2 only */
#define FS_ETPU_SCM_MISC_FLAG          0x00000400
#define FS_ETPU_SCM_READ_ERROR         0x00000100  /* eTPU2 only */

/* ECR - Engine Configuration Register */
#define FS_ETPU_FILTER_CLOCK_DIV2      0x00000000 /* Filter Prescaler Clock */
#define FS_ETPU_FILTER_CLOCK_DIV4      0x00010000 /* Control                */
#define FS_ETPU_FILTER_CLOCK_DIV8      0x00020000
#define FS_ETPU_FILTER_CLOCK_DIV16     0x00030000
#define FS_ETPU_FILTER_CLOCK_DIV32     0x00040000
#define FS_ETPU_FILTER_CLOCK_DIV64     0x00050000
#define FS_ETPU_FILTER_CLOCK_DIV128    0x00060000
#define FS_ETPU_FILTER_CLOCK_DIV256    0x00070000

#define FS_ETPU_FCSS_DIV2              0x00000000 /* Filter Clock Source Selection */
#define FS_ETPU_FCSS_DIV1              0x00080000 /* eTPU2 only */

#define FS_ETPU_CHAN_FILTER_2SAMPLE    0x00000000 /* Channel filter mode */
#define FS_ETPU_CHAN_FILTER_3SAMPLE    0x00008000
#define FS_ETPU_CHAN_FILTER_CONT       0x0000C000
#define FS_ETPU_CHAN_FILTER_BYPASS     0x00004000 /* eTPU2 only */

#define FS_ETPU_ENGINE_ENABLE          0x00000000  /* Low Power Stop Bit */
#define FS_ETPU_ENGINE_DISABLE         0x40000000

#define FS_ETPU_PRIORITY_PASSING_ENABLE   0x00000000 /* Scheduler Priority Passing */
#define FS_ETPU_PRIORITY_PASSING_DISABLE  0x00000080 /* eTPU2 only */

/* TBCR - Time Base Configuration Register */
#define FS_ETPU_TCRCLK_MODE_2SAMPLE     0x00000000 /* TCRCLK Signal Filter Control*/
#define FS_ETPU_TCRCLK_MODE_INTEGRATION 0x10000000
#define FS_ETPU_TCRCLK_INPUT_DIV2CLOCK  0x00000000
#define FS_ETPU_TCRCLK_INPUT_CHANCLOCK  0x08000000

#define FS_ETPU_TCR1CTL_TCRCLK         0x00000000  /* TCR1 Clock/Gate Control */
#define FS_ETPU_TCR1CTL_DIV2           0x00008000
#define FS_ETPU_TCR1CTL_DIV1           0x0000A000  /* (eTPU2-only) */

#define FS_ETPU_TCR1CS_DIV2            0x00000000  /* TCR1 Clock Source */
#define FS_ETPU_TCR1CS_DIV1            0x00002000  /* eTPU2 only */

#define FS_ETPU_ANGLE_MODE_ENABLE      0x02000000   /* Angle Mode */
#define FS_ETPU_ANGLE_MODE_DISABLE     0x00000000
#define FS_ETPU_ANGLE_MODE_ENABLE_CH1  0x04000000   /* eTPU2 only */
#define FS_ETPU_ANGLE_MODE_ENABLE_CH2  0x06000000   /* eTPU2 only */

#define FS_ETPU_TCR2CTL_GATEDDIV8      0x00000000   /* TCR2 Clock/Gate Control */
#define FS_ETPU_TCR2CTL_RISE           0x20000000
#define FS_ETPU_TCR2CTL_FALL           0x40000000
#define FS_ETPU_TCR2CTL_RISEFALL       0x60000000
#define FS_ETPU_TCR2CTL_DIV8           0x80000000

#define FS_ETPU_TCR1_PRESCALER(x)      (((x)-1) & 0xFF) /* TCR1 Prescaler, x = 1 to 256 */
#define FS_ETPU_TCR2_PRESCALER(x)      ((((x)-1) & 0x3F)<<16) /* TCR2 Prescaler, x = 1 to 64 */

/* STACR - Shared Time And Angle Count Register */
#define FS_ETPU_TCR1_STAC_ENABLE       0x80000000   /* TCR1 Resource Enable */
#define FS_ETPU_TCR1_STAC_DISABLE      0x00000000

#define FS_ETPU_TCR1_STAC_CLIENT       0x00000000   /* TCR1 Resource Control */
#define FS_ETPU_TCR1_STAC_SERVER       0x40000000

#define FS_ETPU_TCR1_STAC_SRVSLOT(x)   (((x) & 0xF)<<16) /* TCR1 Server Slot, x = 0 to 3 */

#define FS_ETPU_TCR2_STAC_ENABLE       0x00008000   /* TCR2 Resource Enable */
#define FS_ETPU_TCR2_STAC_DISABLE      0x00000000

#define FS_ETPU_TCR2_STAC_CLIENT       0x00000000   /* TCR2 Resource Control */
#define FS_ETPU_TCR2_STAC_SERVER       0x00004000

#define FS_ETPU_TCR2_STAC_SRVSLOT(x)   ((x) & 0xF)  /* TCR2 Server Slot, x = 0 to 3 */

/* WDTR - Watchdog Timer Register - eTPU2 only */
#define FS_ETPU_WDM_DISABLED           0x00000000 /* Watchdog Mode */
#define FS_ETPU_WDM_THREAD_LEN         0x80000000 /* eTPU2 only */
#define FS_ETPU_WDM_BUSY_LEN           0xC0000000 /* eTPU2 only */

#define FS_ETPU_WATCHDOG_COUNT_MAX     0x0000ffff
#define FS_ETPU_WDTR_WDCNT(x)      ((x) & 0xFFFF) /* Watchdog Count - eTPU2 only */

/* CxCR - Channel x Configuration Register */
#define FS_ETPU_INTERRUPT_ENABLE       0x80000000  /* Channel Interrupt Enable */
#define FS_ETPU_INTERRUPT_DISABLE      0x00000000

#define FS_ETPU_DMA_ENABLE             0x40000000  /* Data Transfer Request Enable*/
#define FS_ETPU_DMA_DISABLE            0x00000000

#define FS_ETPU_ENTRY_TABLE_STANDARD   0x00000000  /* Entry Table Condition Select*/
#define FS_ETPU_ENTRY_TABLE_ALTERNATE  0x01000000

#define FS_ETPU_ENTRY_TABLE_PIN_INPUT  0x00000000  /* Entry Table Pin Direction*/
#define FS_ETPU_ENTRY_TABLE_PIN_OUTPUT 0x02000000

#define FS_ETPU_OUTPUT_DISABLE_OFF     0x00000000  /* Output Disable */
#define FS_ETPU_OUTPUT_DISABLE_LOW     0x0000C000
#define FS_ETPU_OUTPUT_DISABLE_HIGH    0x00008000

/* eTPU error return codes */
#define FS_ETPU_ERROR_NONE             0
#define FS_ETPU_ERROR_MALLOC           1
#define FS_ETPU_ERROR_FREQ             2
#define FS_ETPU_ERROR_VALUE            3
#define FS_ETPU_ERROR_CODESIZE         4
#define FS_ETPU_ERROR_VIS_BIT_NOT_SET  5
#define FS_ETPU_ERROR_ADDRESS          6
#define FS_ETPU_ERROR_TIMING           7
#define FS_ETPU_ERROR_UNINITIALIZED    8
#define FS_ETPU_ERROR_NOT_READY        9

#ifdef __cplusplus
}
#endif

#endif /* _ETPU_UTIL_EXT_H_ */
/*******************************************************************************
 *
 * Copyright:
 *	Freescale Semiconductor, INC. All Rights Reserved.
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
