/******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2014 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
***************************************************************************//*!
*
* @file    etpu_gct.h
*
* @author  Milan Brejl [r54529]
*
* @version 1.0
*
* @date    27-Jul-2014
*
* @brief   This file contains prototypes and definese for etpu_gct.c
*
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "typedefs.h"     /* standard types */

/******************************************************************************
* General Macros
******************************************************************************/
#define ETPU_ENGINE_A_CHANNEL(x)  (x)
#define ETPU_ENGINE_B_CHANNEL(x)  ((x)+64)

#define FS_ETPU_ENTRY_TABLE_ADDR  (((FS_ETPU_ENTRY_TABLE)>>11) & 0x1F)

/******************************************************************************
* Application Constants and Macros
******************************************************************************/
#define SYS_FREQ_HZ                                                       100E6
#define TCR1_FREQ_HZ                                            (SYS_FREQ_HZ/1)
#define TEETH_TILL_GAP                                                       35
#define TEETH_IN_GAP                                                          1
#define TEETH_PER_CYCLE                                                      72
#define TCR2_TICKS_PER_TOOTH                                               1000
#define TCR2_TICKS_PER_CYCLE         ((TEETH_PER_CYCLE)*(TCR2_TICKS_PER_TOOTH))
#define MSEC2TCR1(x)                                 (TCR1_FREQ_HZ/1E3*(x)/1E0)
#define USEC2TCR1(x)                                 (TCR1_FREQ_HZ/1E3*(x)/1E3)
#define NSEC2TCR1(x)                                 (TCR1_FREQ_HZ/1E3*(x)/1E6)
#define DEG2TCR2(x)                              ((x)*TCR2_TICKS_PER_CYCLE/720)
#define UFRACT24(x)                                              ((x)*0xFFFFFF)

/* Tooth Period [TCR1] and RPM */
#define RPM2TP(x)                     (TCR1_FREQ_HZ/(x)*60/(TEETH_PER_CYCLE/2))
#define TP2RPM(x)                     (TCR1_FREQ_HZ/(x)*60/(TEETH_PER_CYCLE/2))

/* Top-Dead Centers */
#define TDC1_DEG       0    
#define TDC3_DEG     180
#define TDC4_DEG     360
#define TDC2_DEG     540

/* Cam log */
#define CAM_LOG_SIZE                                                          8

/******************************************************************************
* Define Functions to Channels
******************************************************************************/
#define ETPU_CAM_CHAN             ETPU_ENGINE_A_CHANNEL(0)
#define ETPU_TG_CAM_CHAN          ETPU_ENGINE_A_CHANNEL(1)
#define ETPU_CRANK_CHAN           ETPU_ENGINE_A_CHANNEL(2)
#define ETPU_TG_CRANK_CHAN        ETPU_ENGINE_A_CHANNEL(3)
#define ETPU_SPARK_1_CHAN         ETPU_ENGINE_A_CHANNEL(4)
#define ETPU_SPARK_2_CHAN         ETPU_ENGINE_A_CHANNEL(5)
#define ETPU_SPARK_3_CHAN         ETPU_ENGINE_A_CHANNEL(6)
#define ETPU_SPARK_4_CHAN         ETPU_ENGINE_A_CHANNEL(7)
#define ETPU_FUEL_1_CHAN          ETPU_ENGINE_A_CHANNEL(8)
#define ETPU_FUEL_2_CHAN          ETPU_ENGINE_A_CHANNEL(9)
#define ETPU_FUEL_3_CHAN          ETPU_ENGINE_A_CHANNEL(10)
#define ETPU_FUEL_4_CHAN          ETPU_ENGINE_A_CHANNEL(11)
#define ETPU_KNOCK_1_CHAN         ETPU_ENGINE_A_CHANNEL(12)
#define ETPU_KNOCK_2_CHAN         ETPU_ENGINE_A_CHANNEL(13)
#define ETPU_INJ_BANK_1_CHAN      ETPU_ENGINE_A_CHANNEL(14)
#define ETPU_INJ_BANK_2_CHAN      ETPU_ENGINE_A_CHANNEL(15)
#define ETPU_INJ_1_CHAN           ETPU_ENGINE_A_CHANNEL(16)
#define ETPU_INJ_2_CHAN           ETPU_ENGINE_A_CHANNEL(17)
#define ETPU_INJ_3_CHAN           ETPU_ENGINE_A_CHANNEL(18)
#define ETPU_INJ_4_CHAN           ETPU_ENGINE_A_CHANNEL(19)

/******************************************************************************
* Define Interrupt Enable, DMA Enable and Output Disable
******************************************************************************/
#define ETPU_CIE_A    ( (1<<ETPU_CRANK_CHAN) \
                       +(1<<ETPU_CAM_CHAN) \
                       +(1<<ETPU_FUEL_1_CHAN) \
                       +(1<<ETPU_FUEL_2_CHAN) \
                       +(1<<ETPU_FUEL_3_CHAN) \
                       +(1<<ETPU_FUEL_4_CHAN) \
                       +(1<<ETPU_SPARK_1_CHAN) \
                       +(1<<ETPU_SPARK_2_CHAN) \
                       +(1<<ETPU_SPARK_3_CHAN) \
                       +(1<<ETPU_SPARK_4_CHAN) \
                       +(1<<ETPU_KNOCK_1_CHAN) \
                       +(1<<ETPU_KNOCK_2_CHAN) \
                       +(1<<ETPU_INJ_1_CHAN) \
                       +(1<<ETPU_INJ_2_CHAN) \
                       +(1<<ETPU_INJ_3_CHAN) \
                       +(1<<ETPU_INJ_4_CHAN) \
                       +(1<<ETPU_TG_CRANK_CHAN))
#define ETPU_DTRE_A   0x00000000
#define ETPU_ODIS_A   0x00000000
#define ETPU_OPOL_A   0x00000000
#define ETPU_CIE_B    0x00000000
#define ETPU_DTRE_B   0x00000000
#define ETPU_ODIS_B   0x00000000
#define ETPU_OPOL_B   0x00000000

/******************************************************************************
* Global Variables Access
******************************************************************************/
/* Global CRANK structures defined in etpu_gct.c */
extern struct crank_instance_t crank_instance;
extern struct crank_config_t   crank_config;
extern struct crank_states_t   crank_states;

/* Global CAM structures defined in etpu_gct.c */
extern struct cam_instance_t cam_instance;
extern struct cam_config_t   cam_config;
extern struct cam_states_t   cam_states;

/* Global SPARK structures defined in etpu_gct.c */
extern struct spark_instance_t spark_1_instance;
extern struct spark_instance_t spark_2_instance;
extern struct spark_instance_t spark_3_instance;
extern struct spark_instance_t spark_4_instance;
extern struct spark_config_t   spark_config;
extern struct spark_states_t   spark_1_states;
extern struct spark_states_t   spark_2_states;
extern struct spark_states_t   spark_3_states;
extern struct spark_states_t   spark_4_states;

/* Global FUEL structures defined in etpu_gct.c */
extern struct fuel_instance_t fuel_1_instance;
extern struct fuel_instance_t fuel_2_instance;
extern struct fuel_instance_t fuel_3_instance;
extern struct fuel_instance_t fuel_4_instance;
extern struct fuel_config_t   fuel_config;
extern struct fuel_states_t   fuel_1_states;
extern struct fuel_states_t   fuel_2_states;
extern struct fuel_states_t   fuel_3_states;
extern struct fuel_states_t   fuel_4_states;

/* Global INJ structures defined in etpu_gct.c */
extern struct inj_instance_t inj_1_instance;
extern struct inj_instance_t inj_2_instance;
extern struct inj_instance_t inj_3_instance;
extern struct inj_instance_t inj_4_instance;
extern struct inj_config_t   inj_config;
extern struct inj_states_t   inj_1_states;
extern struct inj_states_t   inj_2_states;
extern struct inj_states_t   inj_3_states;
extern struct inj_states_t   inj_4_states;

/* Global KNOCK structures defined in etpu_gct.c */
extern struct knock_instance_t knock_1_instance;
extern struct knock_instance_t knock_2_instance;
extern struct knock_config_t   knock_1_config;
extern struct knock_config_t   knock_2_config;

/* Global TG structures defined in etpu_gct.c */
extern struct tg_instance_t tg_instance;
extern struct tg_config_t   tg_config;
extern struct tg_states_t   tg_states;


/******************************************************************************
* Function Prototypes
******************************************************************************/
int32_t my_system_etpu_init ();
void    my_system_etpu_start();

/******************************************************************************
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
 *****************************************************************************/
