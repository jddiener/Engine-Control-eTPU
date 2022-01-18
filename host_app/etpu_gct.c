/*******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2015 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
****************************************************************************//*!
*
* @file    etpu_gct.c
*
* @author  Milan Brejl [r54529]
*
* @version 1.0
*
* @date    29-Jun-2015
*
* @brief   This file contains eTPU module initialization.
*          There are 2 functions to be used by the application:
*          - my_system_etpu_init - initialize eTPU global and channel setting
*          - my_system_etpu_start - run the eTPU
*
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "etpu_util.h"     /* General C Functions for the eTPU */
#include "etpu_gct.h"      /* private header file */
#include "mpc5674f_vars.h" /* eTPU module addresses */
#include "etpu_set.h"      /* eTPU function set code binary image */
#include "etpu_crank.h"    /* eTPU function CRANK API */
#include "etpu_cam.h"      /* eTPU function CAM API */
#include "etpu_fuel.h"     /* eTPU function FUEL API */
#include "etpu_spark.h"    /* eTPU function SPARK API */
#include "etpu_knock.h"    /* eTPU function KNOCK API */
#include "etpu_inj.h"      /* eTPU function INJ API */
#include "etpu_tg.h"       /* eTPU function TG API */
#ifndef CPU32SIM
#include "freemaster.h"
#endif

/*******************************************************************************
* Global variables
*******************************************************************************/
/** @brief   Pointer to the first free parameter in eTPU DATA RAM */
uint32_t *fs_free_param;

/** @brief   Enable FreeMaster to transform eTPU counters: TCR1 to time and TCR2 to angle */
const uint32_t etpu_tcr1_1000ms = MSEC2TCR1(1000);
const uint32_t etpu_tcr1_1000us = USEC2TCR1(1000);
const uint32_t etpu_tcr2_720deg = DEG2TCR2(720);
/** @brief   Enable FreeMaster to transform Tooth Period to Engine Speed */
const uint32_t etpu_rpm2tp = TP2RPM(1);

/*******************************************************************************
 * Global eTPU settings - etpu_config structure
 ******************************************************************************/
/** @brief   Structure handling configuration of all global settings */
struct etpu_config_t my_etpu_config =
{
  /* etpu_config.mcr - Module Configuration Register */
  FS_ETPU_GLOBAL_TIMEBASE_DISABLE  /* keep time-bases stopped during intialization (GTBE=0) */
  | FS_ETPU_MISC_DISABLE, /* SCM operation disabled (SCMMISEN=0) */

  /* etpu_config.misc - MISC Compare Register*/
  FS_ETPU_MISC, /* MISC compare value from etpu_set.h */

  /* etpu_config.ecr_a - Engine A Configuration Register */
  FS_ETPU_ENTRY_TABLE_ADDR /* entry table base address = shifted FS_ETPU_ENTRY_TABLE from etpu_set.h */
  | FS_ETPU_CHAN_FILTER_2SAMPLE /* channel filter mode = three-sample mode (CDFC=0) */
  | FS_ETPU_FCSS_DIV2 /* filter clock source selection = div 2 (FSCC=0) */
  | FS_ETPU_FILTER_CLOCK_DIV2 /* filter prescaler clock control = div 2 (FPSCK=0) */
  | FS_ETPU_PRIORITY_PASSING_ENABLE /* scheduler priority passing is enabled (SPPDIS=0) */
  | FS_ETPU_ENGINE_ENABLE, /* engine is enabled (MDIS=0) */

  /* etpu_config.tbcr_a - Time Base Configuration Register A */
  FS_ETPU_TCRCLK_MODE_2SAMPLE /* TCRCLK signal filter control mode = two-sample mode (TCRCF=0x) */
  | FS_ETPU_TCRCLK_INPUT_DIV2CLOCK /* TCRCLK signal filter control clock = div 2 (TCRCF=x0) */
  | FS_ETPU_TCR1CS_DIV1 /* TCR1 clock source = div 1 (TCR1CS=1)*/
  | FS_ETPU_TCR1CTL_DIV2 /* TCR1 source = div 2 (TCR1CTL=2) */
  | FS_ETPU_TCR1_PRESCALER(1) /* TCR1 prescaler = 1 (TCR1P=0) */
  | FS_ETPU_TCR2CTL_FALL /* TCR2 source = falling external (TCR2CTL=2) */
  | FS_ETPU_TCR2_PRESCALER(1) /* TCR2 prescaler = 1 (TCR2P=0) */
  | FS_ETPU_ANGLE_MODE_ENABLE_CH2, /* TCR2 angle mode is enabled (AM=3) */

  /* etpu_config.stacr_a - Shared Time And Angle Count Register A */
  FS_ETPU_TCR1_STAC_DISABLE /* TCR1 on STAC bus = disabled (REN1=0) */
  | FS_ETPU_TCR1_STAC_CLIENT /* TCR1 resource control = client (RSC1=0) */
  | FS_ETPU_TCR1_STAC_SRVSLOT(0) /* TCR1 server slot = 0 (SRV1=0) */
  | FS_ETPU_TCR2_STAC_DISABLE /* TCR2 on STAC bus = disabled (REN2=0) */
  | FS_ETPU_TCR1_STAC_CLIENT /* TCR2 resource control = client (RSC2=0) */
  | FS_ETPU_TCR2_STAC_SRVSLOT(0), /* TCR2 server slot = 0 (SRV2=0) */

  /* etpu_config.ecr_b - Engine B Configuration Register */
  FS_ETPU_ENTRY_TABLE_ADDR /* entry table base address = shifted FS_ETPU_ENTRY_TABLE from etpu_set.h */
  | FS_ETPU_CHAN_FILTER_2SAMPLE /* channel filter mode = three-sample mode (CDFC=0) */
  | FS_ETPU_FCSS_DIV2 /* filter clock source selection = div 2 (FSCC=0) */
  | FS_ETPU_FILTER_CLOCK_DIV2 /* filter prescaler clock control = div 2 (FPSCK=0) */
  | FS_ETPU_PRIORITY_PASSING_ENABLE /* scheduler priority passing is enabled (SPPDIS=0) */
  | FS_ETPU_ENGINE_ENABLE, /* engine is enabled (MDIS=0) */

  /* etpu_config.tbcr_b - Time Base Configuration Register B */
  FS_ETPU_TCRCLK_MODE_2SAMPLE /* TCRCLK signal filter control mode = two-sample mode (TCRCF=0x) */
  | FS_ETPU_TCRCLK_INPUT_DIV2CLOCK /* TCRCLK signal filter control clock = div 2 (TCRCF=x0) */
  | FS_ETPU_TCR1CS_DIV2 /* TCR1 clock source = div 2 (TCR1CS=0)*/
  | FS_ETPU_TCR1CTL_DIV2 /* TCR1 source = div 2 (TCR1CTL=2) */
  | FS_ETPU_TCR1_PRESCALER(1) /* TCR1 prescaler = 1 (TCR1P=0) */
  | FS_ETPU_TCR2CTL_DIV8 /* TCR2 source = etpuclk div 8 (TCR2CTL=4) */
  | FS_ETPU_TCR2_PRESCALER(1) /* TCR2 prescaler = 1 (TCR2P=0) */
  | FS_ETPU_ANGLE_MODE_DISABLE, /* TCR2 angle mode is disabled (AM=0) */

  /* etpu_config.stacr_b - Shared Time And Angle Count Register B */
  FS_ETPU_TCR1_STAC_DISABLE /* TCR1 on STAC bus = disabled (REN1=0) */
  | FS_ETPU_TCR1_STAC_CLIENT /* TCR1 resource control = client (RSC1=0) */
  | FS_ETPU_TCR1_STAC_SRVSLOT(0) /* TCR1 server slot = 0 (SRV1=0) */
  | FS_ETPU_TCR2_STAC_DISABLE /* TCR2 on STAC bus = disabled (REN2=0) */
  | FS_ETPU_TCR1_STAC_CLIENT /* TCR2 resource control = client (RSC2=0) */
  | FS_ETPU_TCR2_STAC_SRVSLOT(0), /* TCR2 server slot = 0 (SRV2=0) */

  /* etpu_config.wdtr_a - Watchdog Timer Register A(eTPU2 only) */
  FS_ETPU_WDM_DISABLED /* watchdog mode = disabled */
  | FS_ETPU_WDTR_WDCNT(0), /* watchdog count = 0 */

  /* etpu_config.wdtr_b - Watchdog Timer Register B (eTPU2 only) */
  FS_ETPU_WDM_DISABLED /* watchdog mode = disabled */
  | FS_ETPU_WDTR_WDCNT(0) /* watchdog count = 0 */
};

/*******************************************************************************
 * eTPU channel settings - CRANK
 ******************************************************************************/
/** @brief   Initialization of CRANK structures */
struct crank_instance_t crank_instance =
{
  ETPU_CRANK_CHAN,         /* chan_num */
  FS_ETPU_PRIORITY_HIGH,   /* priority */
  FS_ETPU_CRANK_FM0_USE_TRANS_FALLING, /* polarity */
  TEETH_TILL_GAP,          /* teeth_till_gap */
  TEETH_IN_GAP,            /* teeth_in_gap */
  TEETH_PER_CYCLE,         /* teeth_per_cycle */
  FS_ETPU_TCR1CS_DIV1,     /* TCR1CS is DIV1 (eTPU clock) */
  TCR2_TICKS_PER_TOOTH,    /* tcr2_ticks_per_tooth */
  0,                       /* tcr2_ticks_per_add_tooth */
  FS_ETPU_CRANK_FM1_TOOTH_PERIODS_LOG_ON, /* log_tooth_periods */
  ((ETPU_CAM_CHAN <<  0) +
   (ETPU_CAM_CHAN <<  8) +
   (ETPU_CAM_CHAN << 16) +
   (ETPU_CAM_CHAN << 24)),     /* link_cam */
  ((ETPU_SPARK_1_CHAN <<  0) +
   (ETPU_SPARK_2_CHAN <<  8) +
   (ETPU_SPARK_3_CHAN << 16) +
   (ETPU_SPARK_4_CHAN << 24)), /* link_1 - sparks */
  ((ETPU_FUEL_1_CHAN <<  0) +
   (ETPU_FUEL_2_CHAN <<  8) +
   (ETPU_FUEL_3_CHAN << 16) +
   (ETPU_FUEL_4_CHAN << 24)),  /* link_2 - fuels */
  ((ETPU_KNOCK_1_CHAN <<  0) +
   (ETPU_KNOCK_2_CHAN <<  8) +
   (ETPU_INJ_BANK_1_CHAN << 16) +
   (ETPU_INJ_BANK_2_CHAN << 24)),/* link_3 - knocks and inj_banks */
  ((ETPU_INJ_1_CHAN <<  0) +
   (ETPU_INJ_2_CHAN <<  8) +
   (ETPU_INJ_3_CHAN << 16) +
   (ETPU_INJ_4_CHAN << 24)),   /* link_4 - injs */
  0,                       /* *cpba */  /* 0 for automatic allocation */
  0                        /* *cpba_tooth_period_log */  /* automatic allocation */
};

struct crank_config_t crank_config =
{
  1*(TEETH_TILL_GAP+TEETH_IN_GAP), /* teeth_per_sync */
  MSEC2TCR1(10), /* blank_time */
  5,             /* blank_teeth */
  UFRACT24(0.6), /* gap_ratio */
  UFRACT24(0.2), /* win_ratio_normal */
  UFRACT24(0.5), /* win_ratio_across_gap */
  UFRACT24(0.2), /* win_ratio_after_gap */
  UFRACT24(0.5), /* win_ratio_after_timeout */
  MSEC2TCR1(50)  /* first_tooth_timeout */
};

struct crank_states_t crank_states;

/*******************************************************************************
 * eTPU channel settings - CAM
 ******************************************************************************/
/** @brief   Initialization of CAM structures */
struct cam_instance_t cam_instance =
{
  ETPU_CAM_CHAN,         /* chan_num */
  FS_ETPU_PRIORITY_LOW,  /* priority */
  CAM_LOG_SIZE,          /* log_size */ 
  0,                     /* *cpba */     /* 0 for automatic allocation */
  0                      /* *cpba_log */ /* 0 for automatic allocation */
};

struct cam_config_t cam_config =
{
  FS_ETPU_CAM_LOG_BOTH /* mode */
};

struct cam_states_t cam_states;

/*******************************************************************************
 * eTPU channel settings - SPARKs
 ******************************************************************************/
/** @brief   Initialization of SPARK structures */
struct spark_instance_t spark_1_instance =
{
  ETPU_SPARK_1_CHAN,       /* chan_num */
  FS_ETPU_PRIORITY_MIDDLE, /* priority */
  FS_ETPU_SPARK_FM0_ACTIVE_HIGH, /* polarity */
  DEG2TCR2(TDC1_DEG),      /* tdc_angle */
  0,                       /* *cpba */               /* 0 for automatic allocation */
  0                        /* *cpba_single_spark */  /* 0 for automatic allocation */
};

struct spark_instance_t spark_2_instance =
{
  ETPU_SPARK_2_CHAN,       /* chan_num */
  FS_ETPU_PRIORITY_MIDDLE, /* priority */
  FS_ETPU_SPARK_FM0_ACTIVE_HIGH, /* polarity */
  DEG2TCR2(TDC2_DEG),      /* tdc_angle */
  0,                       /* *cpba */               /* 0 for automatic allocation */
  0                        /* *cpba_single_spark */  /* 0 for automatic allocation */
};

struct spark_instance_t spark_3_instance =
{
  ETPU_SPARK_3_CHAN,       /* chan_num */
  FS_ETPU_PRIORITY_MIDDLE, /* priority */
  FS_ETPU_SPARK_FM0_ACTIVE_HIGH, /* polarity */
  DEG2TCR2(TDC3_DEG),      /* tdc_angle */
  0,                       /* *cpba */               /* 0 for automatic allocation */
  0                        /* *cpba_single_spark */  /* 0 for automatic allocation */
};

struct spark_instance_t spark_4_instance =
{
  ETPU_SPARK_4_CHAN,       /* chan_num */
  FS_ETPU_PRIORITY_MIDDLE, /* priority */
  FS_ETPU_SPARK_FM0_ACTIVE_HIGH, /* polarity */
  DEG2TCR2(TDC4_DEG),      /* tdc_angle */
  0,                       /* *cpba */               /* 0 for automatic allocation */
  0                        /* *cpba_single_spark */  /* 0 for automatic allocation */
};

struct single_spark_config_t single_spark_config[2] =
{
  {
    DEG2TCR2(0),        /* end_angle */
    USEC2TCR1(2000),    /* dwell_time */
    3                   /* multi_pulse_count */
  },
  {
    DEG2TCR2(-360),     /* end_angle */
    USEC2TCR1(2000),    /* dwell_time */
    3                   /* multi_pulse_count */
  }
};

struct spark_config_t spark_config =
{
  DEG2TCR2(30),    /* angle_offset_recalc */
  USEC2TCR1(1900), /* dwell_time_min */
  USEC2TCR1(2100), /* dwell_time_max */
  USEC2TCR1(100),  /* multi_on_time */
  USEC2TCR1(100),  /* multi_off_time */
  1,               /* spark_count */
  &single_spark_config[0],  /* p_single_spark_config */
  FS_ETPU_SPARK_GENERATION_ALLOWED  /* generation_disable */
};

struct spark_states_t spark_1_states;
struct spark_states_t spark_2_states;
struct spark_states_t spark_3_states;
struct spark_states_t spark_4_states;

/*******************************************************************************
 * eTPU channel settings - FUELs
 ******************************************************************************/
/** @brief   Initialization of FUEL structures */
struct fuel_instance_t fuel_1_instance =
{
  ETPU_FUEL_1_CHAN,         /* chan_num */
  FS_ETPU_PRIORITY_MIDDLE, /* priority */
  FS_ETPU_FUEL_FM0_ACTIVE_HIGH, /* polarity */
  DEG2TCR2(TDC1_DEG),      /* tdc_angle */
  0                        /* *cpba */  /* 0 for automatic allocation */
};

struct fuel_instance_t fuel_2_instance =
{
  ETPU_FUEL_2_CHAN,         /* chan_num */
  FS_ETPU_PRIORITY_MIDDLE, /* priority */
  FS_ETPU_FUEL_FM0_ACTIVE_HIGH, /* polarity */
  DEG2TCR2(TDC2_DEG),      /* tdc_angle */
  0                        /* *cpba */  /* 0 for automatic allocation */
};

struct fuel_instance_t fuel_3_instance =
{
  ETPU_FUEL_3_CHAN,         /* chan_num */
  FS_ETPU_PRIORITY_MIDDLE, /* priority */
  FS_ETPU_FUEL_FM0_ACTIVE_HIGH, /* polarity */
  DEG2TCR2(TDC3_DEG),      /* tdc_angle */
  0                        /* *cpba */  /* 0 for automatic allocation */
};

struct fuel_instance_t fuel_4_instance =
{
  ETPU_FUEL_4_CHAN,         /* chan_num */
  FS_ETPU_PRIORITY_MIDDLE, /* priority */
  FS_ETPU_FUEL_FM0_ACTIVE_HIGH, /* polarity */
  DEG2TCR2(TDC4_DEG),      /* tdc_angle */
  0                        /* *cpba */  /* 0 for automatic allocation */
};

struct fuel_config_t fuel_config =
{
  DEG2TCR2(60),     /* angle_normal_end */
  DEG2TCR2(40),     /* angle_stop */
  DEG2TCR2(30),     /* angle_offset_recalc */
  USEC2TCR1(20000), /* injection_time */
  USEC2TCR1(1000),  /* compensation_time */
  USEC2TCR1(1000),  /* injection_time_minimum */
  USEC2TCR1(1000)   /* off_time_minimum */
};

struct fuel_states_t fuel_1_states;
struct fuel_states_t fuel_2_states;
struct fuel_states_t fuel_3_states;
struct fuel_states_t fuel_4_states;

/*******************************************************************************
 * eTPU channel settings - INJ
 ******************************************************************************/
/** @brief   Initialization of INJ structures */
struct inj_instance_t inj_1_instance =
{
  ETPU_INJ_1_CHAN,       /* chan_num_inj */
  ETPU_INJ_BANK_1_CHAN,  /* chan_num_bank_1 */
  ETPU_INJ_BANK_2_CHAN,  /* chan_num_bank_2 */
  FS_ETPU_INJ_BANK_CHAN_NOT_USED, /* chan_num_bank_3 */
  FS_ETPU_PRIORITY_HIGH,  /* priority */
  FS_ETPU_INJ_FM0_ACTIVE_HIGH, /* polarity_inj */
  FS_ETPU_INJ_FM0_ACTIVE_HIGH, /* polarity_bank */
  DEG2TCR2(TDC1_DEG),    /* tdc_angle */
  0,                     /* *cpba */  /* 0 for automatic allocation */
  0,                     /* *cpba_injections */
  0                      /* *cpba_phases */
};
struct inj_instance_t inj_2_instance =
{
  ETPU_INJ_2_CHAN,       /* chan_num_inj */
  ETPU_INJ_BANK_1_CHAN,  /* chan_num_bank_1 */
  ETPU_INJ_BANK_2_CHAN,  /* chan_num_bank_2 */
  FS_ETPU_INJ_BANK_CHAN_NOT_USED, /* chan_num_bank_3 */
  FS_ETPU_PRIORITY_HIGH,  /* priority */
  FS_ETPU_INJ_FM0_ACTIVE_HIGH, /* polarity_inj */
  FS_ETPU_INJ_FM0_ACTIVE_HIGH, /* polarity_bank */
  DEG2TCR2(TDC2_DEG),    /* tdc_angle */
  0,                     /* *cpba */  /* 0 for automatic allocation */
  0,                     /* *cpba_injections */
  0                      /* *cpba_phases */
};
struct inj_instance_t inj_3_instance =
{
  ETPU_INJ_3_CHAN,       /* chan_num_inj */
  ETPU_INJ_BANK_1_CHAN,  /* chan_num_bank_1 */
  ETPU_INJ_BANK_2_CHAN,  /* chan_num_bank_2 */
  FS_ETPU_INJ_BANK_CHAN_NOT_USED, /* chan_num_bank_3 */
  FS_ETPU_PRIORITY_HIGH,  /* priority */
  FS_ETPU_INJ_FM0_ACTIVE_HIGH, /* polarity_inj */
  FS_ETPU_INJ_FM0_ACTIVE_HIGH, /* polarity_bank */
  DEG2TCR2(TDC3_DEG),    /* tdc_angle */
  0,                     /* *cpba */  /* 0 for automatic allocation */
  0,                     /* *cpba_injections */
  0                      /* *cpba_phases */
};
struct inj_instance_t inj_4_instance =
{
  ETPU_INJ_4_CHAN,       /* chan_num_inj */
  ETPU_INJ_BANK_1_CHAN,  /* chan_num_bank_1 */
  ETPU_INJ_BANK_2_CHAN,  /* chan_num_bank_2 */
  FS_ETPU_INJ_BANK_CHAN_NOT_USED, /* chan_num_bank_3 */
  FS_ETPU_PRIORITY_HIGH,  /* priority */
  FS_ETPU_INJ_FM0_ACTIVE_HIGH, /* polarity_inj */
  FS_ETPU_INJ_FM0_ACTIVE_HIGH, /* polarity_bank */
  DEG2TCR2(TDC4_DEG),    /* tdc_angle */
  0,                     /* *cpba */  /* 0 for automatic allocation */
  0,                     /* *cpba_injections */
  0                      /* *cpba_phases */
};

uint32_t inj_injection_1_phase_config[5] =
{
/*     duration,  BANK 1 output,                      BANK 2 output,                      INJ output */
  USEC2TCR1(20) + FS_ETPU_INJ_PHASE_OUT_HIGH_BANK_1 + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_HIGH_INJ,
  USEC2TCR1(10) + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_HIGH_INJ,
  USEC2TCR1(30) + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_HIGH_BANK_2 + FS_ETPU_INJ_PHASE_OUT_HIGH_INJ,
  USEC2TCR1(10) + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_HIGH_INJ,
  USEC2TCR1(50) + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_HIGH_BANK_2 + FS_ETPU_INJ_PHASE_OUT_HIGH_INJ,
};
uint32_t inj_injection_2_phase_config[7] =
{
/*     duration,  BANK 1 output,                      BANK 2 output,                      INJ output */
  USEC2TCR1(20) + FS_ETPU_INJ_PHASE_OUT_HIGH_BANK_1 + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_HIGH_INJ,
  USEC2TCR1(10) + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_HIGH_INJ,
  USEC2TCR1(30) + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_HIGH_BANK_2 + FS_ETPU_INJ_PHASE_OUT_HIGH_INJ,
  USEC2TCR1(10) + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_HIGH_INJ,
  USEC2TCR1(100)+ FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_HIGH_BANK_2 + FS_ETPU_INJ_PHASE_OUT_HIGH_INJ,
  USEC2TCR1( 5) + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_HIGH_INJ,
  USEC2TCR1(50) + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_HIGH_BANK_2 + FS_ETPU_INJ_PHASE_OUT_HIGH_INJ,
};
uint32_t inj_injection_3_phase_config[3] =
{
/*     duration,  BANK 1 output,                      BANK 2 output,                      INJ output */
  USEC2TCR1(20) + FS_ETPU_INJ_PHASE_OUT_HIGH_BANK_1 + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_HIGH_INJ,
  USEC2TCR1(10) + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_HIGH_INJ,
  USEC2TCR1(40) + FS_ETPU_INJ_PHASE_OUT_LOW         + FS_ETPU_INJ_PHASE_OUT_HIGH_BANK_2 + FS_ETPU_INJ_PHASE_OUT_HIGH_INJ,
};

struct inj_injection_config_t inj_injection_config[3] =
{
  {
    DEG2TCR2(20),        /* angle_start */
    5,                   /* phase_count */
    &inj_injection_1_phase_config[0]  /* *p_phase_config */
  },
  {
    DEG2TCR2(10),        /* angle_start */
    7,                   /* phase_count */
    &inj_injection_2_phase_config[0]  /* *p_phase_config */
  },
  {
    DEG2TCR2(-5),        /* angle_start */
    3,                   /* phase_count */
    &inj_injection_3_phase_config[0]  /* *p_phase_config */
  }
};

struct inj_config_t inj_config =
{
  DEG2TCR2(90),            /* angle_irq */
  DEG2TCR2(-20),           /* angle_stop */
  3,                       /* injection_count */
  &inj_injection_config[0] /* *p_inj_injection_config */
};

struct inj_states_t inj_1_states;
struct inj_states_t inj_2_states;
struct inj_states_t inj_3_states;
struct inj_states_t inj_4_states;

/*******************************************************************************
 * eTPU channel settings - KNOCKs
 ******************************************************************************/
/** @brief   Initialization of KNOCK structures */
struct knock_instance_t knock_1_instance =
{
  ETPU_KNOCK_1_CHAN,       /* chan_num */
  FS_ETPU_PRIORITY_MIDDLE, /* priority */
  FS_ETPU_KNOCK_FM0_ACTIVE_HIGH, /* polarity */
  DEG2TCR2(TDC1_DEG),      /* tdc_angle */
  0,                       /* *cpba */          /* 0 for automatic allocation */
  0                        /* *cpba_windows */  /* 0 for automatic allocation */
};

struct knock_instance_t knock_2_instance =
{
  ETPU_KNOCK_2_CHAN,       /* chan_num */
  FS_ETPU_PRIORITY_MIDDLE, /* priority */
  FS_ETPU_KNOCK_FM0_ACTIVE_HIGH, /* polarity */
  DEG2TCR2(TDC3_DEG),      /* tdc_angle */
  0,                       /* *cpba */          /* 0 for automatic allocation */
  0                        /* *cpba_windows */  /* 0 for automatic allocation */
};

struct knock_window_config_t knock_window_config[2] =
{
  {
    DEG2TCR2(90),     /* angle_start */
    DEG2TCR2(180)     /* angle_width */
  },
  {
    DEG2TCR2(90-360), /* angle_start */
    DEG2TCR2(180)     /* angle_width */
  }
};

struct knock_config_t knock_1_config =
{
  FS_ETPU_KNOCK_FM1_MODE_TRIGGER,   /* mode */
  2,                                /* window_count */
  &knock_window_config[0],          /* p_knock_window_config */
  USEC2TCR1(100),                   /* trigger_period */
  FS_ETPU_KNOCK_IRQ_AT_WINDOW_END   /* irq_dma_options */
};

struct knock_config_t knock_2_config =
{
  FS_ETPU_KNOCK_FM1_MODE_TRIGGER,   /* mode */
  2,                                /* window_count */
  &knock_window_config[0],          /* p_knock_window_config */
  USEC2TCR1(100),                   /* trigger_period */
  FS_ETPU_KNOCK_IRQ_AT_WINDOW_END   /* irq_dma_options */
};

/*******************************************************************************
 * eTPU channel settings - TG
 ******************************************************************************/
/** @brief   Initialization of TG structures */
uint8_t cam_edge_teeth[] = {
  6, 12, 27, 
  36+15, 36+24, 36+30
};

struct tg_instance_t tg_instance =
{
  ETPU_TG_CRANK_CHAN,    /* chan_num_crank */
  ETPU_TG_CAM_CHAN,      /* chan_num_cam */
  FS_ETPU_PRIORITY_LOW,  /* priority */
  FS_ETPU_TG_FM0_POLARITY_LOW, /* polarity_crank */
  FS_ETPU_TG_FM0_POLARITY_LOW, /* polarity_cam */
  TEETH_TILL_GAP,        /* teeth_till_gap */
  TEETH_IN_GAP,          /* teeth_in_gap */
  TEETH_PER_CYCLE,       /* teeth_per_cycle */
  sizeof(cam_edge_teeth),/* cam_edge_count */
  &cam_edge_teeth[0],    /* *p_cam_edge_tooth */
  0,                     /* *cpba */  /* 0 for automatic allocation */
  0                      /* *cpba_cam_edge_tooth */
};

struct tg_config_t tg_config =
{
  RPM2TP(5000),    /* tooth_period_target */
  UFRACT24(0.1),  /* accel_rate */
  FS_ETPU_TG_GENERATION_ALLOWED /* generation_disable */
};

struct tg_states_t tg_states;

#ifndef CPU32SIM
/******************************************************************************
* FreeMASTER TSA tables
******************************************************************************/
/*
 * With TSA enabled, the user describes the global and static variables using 
 * so-called TSA tables. There can be any number of tables defined in 
 * the project files. Each table does have the identifier which should be
 * unique across the project. 
 *
 * Note that you can declare variables as Read-Only or Read-Write.
 * The FreeMASTER driver denies any write access to the Read-Only variables
 * when TSA_SAFETY is enabled.
 */

FMSTR_TSA_TABLE_BEGIN(fmstr_tsa_table_etpu_scaling)
    FMSTR_TSA_RO_VAR(etpu_tcr1_1000ms, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(etpu_tcr1_1000us, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(etpu_tcr2_720deg, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(etpu_rpm2tp, FMSTR_TSA_UINT32)
FMSTR_TSA_TABLE_END()

FMSTR_TSA_TABLE_BEGIN(fmstr_tsa_table_crank)
    FMSTR_TSA_RO_VAR(crank_instance, FMSTR_TSA_USERTYPE(struct crank_instance_t))
    FMSTR_TSA_RW_VAR(crank_config, FMSTR_TSA_USERTYPE(struct crank_config_t))
    FMSTR_TSA_RO_VAR(crank_states, FMSTR_TSA_USERTYPE(struct crank_states_t))
    
    FMSTR_TSA_STRUCT(struct crank_instance_t)
    FMSTR_TSA_MEMBER(struct crank_instance_t, chan_num, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct crank_instance_t, priority, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct crank_instance_t, polarity, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct crank_instance_t, teeth_till_gap, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct crank_instance_t, teeth_in_gap, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct crank_instance_t, teeth_per_cycle, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct crank_instance_t, tcr2_ticks_per_tooth, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct crank_instance_t, tcr2_ticks_per_add_tooth, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct crank_instance_t, log_tooth_periods, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct crank_instance_t, cpba, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct crank_instance_t, link_cam, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct crank_instance_t, link_1, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct crank_instance_t, link_2, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct crank_instance_t, link_3, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct crank_instance_t, link_4, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct crank_instance_t, cpba, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct crank_instance_t, cpba_tooth_period_log, FMSTR_TSA_UINT32)
    FMSTR_TSA_STRUCT(struct crank_config_t)
    FMSTR_TSA_MEMBER(struct crank_config_t, teeth_per_sync, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct crank_config_t, blank_time, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct crank_config_t, blank_teeth, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct crank_config_t, gap_ratio, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct crank_config_t, win_ratio_normal, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct crank_config_t, win_ratio_across_gap, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct crank_config_t, win_ratio_after_gap, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct crank_config_t, win_ratio_after_timeout, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct crank_config_t, first_tooth_timeout, FMSTR_TSA_UINT32)
    FMSTR_TSA_STRUCT(struct crank_states_t)
    FMSTR_TSA_MEMBER(struct crank_states_t, error, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct crank_states_t, state, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct crank_states_t, eng_pos_state, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct crank_states_t, tooth_counter_gap, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct crank_states_t, tooth_counter_cycle, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct crank_states_t, last_tooth_period, FMSTR_TSA_UINT32)
FMSTR_TSA_TABLE_END()

FMSTR_TSA_TABLE_BEGIN(fmstr_tsa_table_cam)
    FMSTR_TSA_RO_VAR(cam_instance, FMSTR_TSA_USERTYPE(struct cam_instance_t))
    FMSTR_TSA_RW_VAR(cam_config, FMSTR_TSA_USERTYPE(struct cam_config_t))
    FMSTR_TSA_RO_VAR(cam_states, FMSTR_TSA_USERTYPE(struct cam_states_t))
    
    FMSTR_TSA_STRUCT(struct cam_instance_t)
    FMSTR_TSA_MEMBER(struct cam_instance_t, chan_num, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct cam_instance_t, priority, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct cam_instance_t, log_size, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct cam_instance_t, cpba, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct cam_instance_t, cpba_log, FMSTR_TSA_UINT32)
    FMSTR_TSA_STRUCT(struct cam_config_t)
    FMSTR_TSA_MEMBER(struct cam_config_t, mode, FMSTR_TSA_UINT8)
    FMSTR_TSA_STRUCT(struct cam_states_t)
    FMSTR_TSA_MEMBER(struct cam_states_t, error, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct cam_states_t, log_count, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct cam_states_t, log_idx, FMSTR_TSA_UINT8)
FMSTR_TSA_TABLE_END()

FMSTR_TSA_TABLE_BEGIN(fmstr_tsa_table_spark)
    FMSTR_TSA_RO_VAR(spark_1_instance, FMSTR_TSA_USERTYPE(struct spark_instance_t))
    FMSTR_TSA_RO_VAR(spark_2_instance, FMSTR_TSA_USERTYPE(struct spark_instance_t))
    FMSTR_TSA_RO_VAR(spark_3_instance, FMSTR_TSA_USERTYPE(struct spark_instance_t))
    FMSTR_TSA_RO_VAR(spark_4_instance, FMSTR_TSA_USERTYPE(struct spark_instance_t))
    FMSTR_TSA_RW_VAR(spark_config, FMSTR_TSA_USERTYPE(struct spark_config_t))
    FMSTR_TSA_RW_VAR(single_spark_config, FMSTR_TSA_USERTYPE(struct single_spark_config_t))
    FMSTR_TSA_RO_VAR(spark_1_states, FMSTR_TSA_USERTYPE(struct spark_states_t))
    FMSTR_TSA_RO_VAR(spark_2_states, FMSTR_TSA_USERTYPE(struct spark_states_t))
    FMSTR_TSA_RO_VAR(spark_3_states, FMSTR_TSA_USERTYPE(struct spark_states_t))
    FMSTR_TSA_RO_VAR(spark_4_states, FMSTR_TSA_USERTYPE(struct spark_states_t))
    
    FMSTR_TSA_STRUCT(struct spark_instance_t)
    FMSTR_TSA_MEMBER(struct spark_instance_t, chan_num, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct spark_instance_t, priority, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct spark_instance_t, polarity, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct spark_instance_t, tdc_angle, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct spark_instance_t, cpba, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct spark_instance_t, cpba_single_spark, FMSTR_TSA_UINT32)
    FMSTR_TSA_STRUCT(struct spark_config_t)
    FMSTR_TSA_MEMBER(struct spark_config_t, angle_offset_recalc, FMSTR_TSA_SINT32)
    FMSTR_TSA_MEMBER(struct spark_config_t, dwell_time_min, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct spark_config_t, dwell_time_max, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct spark_config_t, multi_on_time, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct spark_config_t, multi_off_time, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct spark_config_t, spark_count, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct spark_config_t, p_single_spark_config, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct spark_config_t, generation_disable, FMSTR_TSA_UINT8)
    FMSTR_TSA_STRUCT(struct single_spark_config_t)
    FMSTR_TSA_MEMBER(struct single_spark_config_t, end_angle, FMSTR_TSA_SINT32)
    FMSTR_TSA_MEMBER(struct single_spark_config_t, dwell_time, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct single_spark_config_t, multi_pulse_count, FMSTR_TSA_UINT8)
    FMSTR_TSA_STRUCT(struct spark_states_t)
    FMSTR_TSA_MEMBER(struct spark_states_t, error, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct spark_states_t, dwell_time_applied, FMSTR_TSA_UINT32)
FMSTR_TSA_TABLE_END()

FMSTR_TSA_TABLE_BEGIN(fmstr_tsa_table_fuel)
    FMSTR_TSA_RO_VAR(fuel_1_instance, FMSTR_TSA_USERTYPE(struct fuel_instance_t))
    FMSTR_TSA_RO_VAR(fuel_2_instance, FMSTR_TSA_USERTYPE(struct fuel_instance_t))
    FMSTR_TSA_RO_VAR(fuel_3_instance, FMSTR_TSA_USERTYPE(struct fuel_instance_t))
    FMSTR_TSA_RO_VAR(fuel_4_instance, FMSTR_TSA_USERTYPE(struct fuel_instance_t))
    FMSTR_TSA_RW_VAR(fuel_config, FMSTR_TSA_USERTYPE(struct fuel_config_t))
    FMSTR_TSA_RO_VAR(fuel_1_states, FMSTR_TSA_USERTYPE(struct fuel_states_t))
    FMSTR_TSA_RO_VAR(fuel_2_states, FMSTR_TSA_USERTYPE(struct fuel_states_t))
    FMSTR_TSA_RO_VAR(fuel_3_states, FMSTR_TSA_USERTYPE(struct fuel_states_t))
    FMSTR_TSA_RO_VAR(fuel_4_states, FMSTR_TSA_USERTYPE(struct fuel_states_t))
    
    FMSTR_TSA_STRUCT(struct fuel_instance_t)
    FMSTR_TSA_MEMBER(struct fuel_instance_t, chan_num, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct fuel_instance_t, priority, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct fuel_instance_t, polarity, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct fuel_instance_t, tdc_angle, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct fuel_instance_t, cpba, FMSTR_TSA_UINT32)
    FMSTR_TSA_STRUCT(struct fuel_config_t)
    FMSTR_TSA_MEMBER(struct fuel_config_t, angle_normal_end, FMSTR_TSA_SINT32)
    FMSTR_TSA_MEMBER(struct fuel_config_t, angle_stop, FMSTR_TSA_SINT32)
    FMSTR_TSA_MEMBER(struct fuel_config_t, angle_offset_recalc, FMSTR_TSA_SINT32)
    FMSTR_TSA_MEMBER(struct fuel_config_t, injection_time, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct fuel_config_t, compensation_time, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct fuel_config_t, injection_time_minimum, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct fuel_config_t, off_time_minimum, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct fuel_config_t, generation_disable, FMSTR_TSA_UINT8)
    FMSTR_TSA_STRUCT(struct fuel_states_t)
    FMSTR_TSA_MEMBER(struct fuel_states_t, error, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct fuel_states_t, injection_time_applied, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct fuel_states_t, injection_start_angle, FMSTR_TSA_SINT32)
FMSTR_TSA_TABLE_END()

FMSTR_TSA_TABLE_BEGIN(fmstr_tsa_table_inj)
    FMSTR_TSA_RO_VAR(inj_1_instance, FMSTR_TSA_USERTYPE(struct inj_instance_t))
    FMSTR_TSA_RO_VAR(inj_2_instance, FMSTR_TSA_USERTYPE(struct inj_instance_t))
    FMSTR_TSA_RO_VAR(inj_3_instance, FMSTR_TSA_USERTYPE(struct inj_instance_t))
    FMSTR_TSA_RO_VAR(inj_4_instance, FMSTR_TSA_USERTYPE(struct inj_instance_t))
    FMSTR_TSA_RW_VAR(inj_config, FMSTR_TSA_USERTYPE(struct inj_config_t))
    FMSTR_TSA_RW_VAR(inj_injection_config, FMSTR_TSA_USERTYPE(struct inj_injection_config_t))
    FMSTR_TSA_RW_VAR(inj_injection_1_phase_config, FMSTR_TSA_UINT32)
    FMSTR_TSA_RW_VAR(inj_injection_2_phase_config, FMSTR_TSA_UINT32)
    FMSTR_TSA_RW_VAR(inj_injection_3_phase_config, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(inj_1_states, FMSTR_TSA_USERTYPE(struct inj_states_t))
    FMSTR_TSA_RO_VAR(inj_2_states, FMSTR_TSA_USERTYPE(struct inj_states_t))
    FMSTR_TSA_RO_VAR(inj_3_states, FMSTR_TSA_USERTYPE(struct inj_states_t))
    FMSTR_TSA_RO_VAR(inj_4_states, FMSTR_TSA_USERTYPE(struct inj_states_t))

    FMSTR_TSA_STRUCT(struct inj_instance_t)
    FMSTR_TSA_MEMBER(struct inj_instance_t, chan_num_inj, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct inj_instance_t, chan_num_bank_1, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct inj_instance_t, chan_num_bank_2, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct inj_instance_t, chan_num_bank_3, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct inj_instance_t, priority, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct inj_instance_t, polarity_inj, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct inj_instance_t, polarity_bank, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct inj_instance_t, tdc_angle, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct inj_instance_t, cpba, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct inj_instance_t, cpba_injections, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct inj_instance_t, cpba_phases, FMSTR_TSA_UINT32)
    FMSTR_TSA_STRUCT(struct inj_config_t)
    FMSTR_TSA_MEMBER(struct inj_config_t, angle_irq, FMSTR_TSA_SINT32)
    FMSTR_TSA_MEMBER(struct inj_config_t, angle_stop, FMSTR_TSA_SINT32)
    FMSTR_TSA_MEMBER(struct inj_config_t, injection_count, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct inj_config_t, p_injection_config, FMSTR_TSA_UINT32)
    FMSTR_TSA_STRUCT(struct inj_injection_config_t)
    FMSTR_TSA_MEMBER(struct inj_injection_config_t, angle_start, FMSTR_TSA_SINT32)
    FMSTR_TSA_MEMBER(struct inj_injection_config_t, phase_count, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct inj_injection_config_t, p_phase_config, FMSTR_TSA_UINT32)
    FMSTR_TSA_STRUCT(struct inj_states_t)
    FMSTR_TSA_MEMBER(struct inj_states_t, error, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct inj_states_t, injection_idx, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct inj_states_t, phase_idx, FMSTR_TSA_UINT8)
FMSTR_TSA_TABLE_END()

FMSTR_TSA_TABLE_BEGIN(fmstr_tsa_table_knock)
    FMSTR_TSA_RO_VAR(knock_1_instance, FMSTR_TSA_USERTYPE(struct knock_instance_t))
    FMSTR_TSA_RO_VAR(knock_2_instance, FMSTR_TSA_USERTYPE(struct knock_instance_t))
    FMSTR_TSA_RW_VAR(knock_1_config, FMSTR_TSA_USERTYPE(struct knock_config_t))
    FMSTR_TSA_RW_VAR(knock_2_config, FMSTR_TSA_USERTYPE(struct knock_config_t))
    FMSTR_TSA_RW_VAR(knock_window_config, FMSTR_TSA_USERTYPE(struct knock_window_config_t))

    FMSTR_TSA_STRUCT(struct knock_instance_t)
    FMSTR_TSA_MEMBER(struct knock_instance_t, chan_num, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct knock_instance_t, priority, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct knock_instance_t, polarity, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct knock_instance_t, tdc_angle, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct knock_instance_t, cpba, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct knock_instance_t, cpba_windows, FMSTR_TSA_UINT32)
    FMSTR_TSA_STRUCT(struct knock_config_t)
    FMSTR_TSA_MEMBER(struct knock_config_t, mode, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct knock_config_t, window_count, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct knock_config_t, p_knock_window_config, FMSTR_TSA_SINT32)
    FMSTR_TSA_MEMBER(struct knock_config_t, trigger_period, FMSTR_TSA_SINT32)
    FMSTR_TSA_MEMBER(struct knock_config_t, irq_dma_options, FMSTR_TSA_UINT8)
    FMSTR_TSA_STRUCT(struct knock_window_config_t)
    FMSTR_TSA_MEMBER(struct knock_window_config_t, angle_start, FMSTR_TSA_SINT32)
    FMSTR_TSA_MEMBER(struct knock_window_config_t, angle_width, FMSTR_TSA_SINT32)
FMSTR_TSA_TABLE_END()

FMSTR_TSA_TABLE_BEGIN(fmstr_tsa_table_tg)
    FMSTR_TSA_RO_VAR(tg_instance, FMSTR_TSA_USERTYPE(struct tg_instance_t))
    FMSTR_TSA_RW_VAR(tg_config, FMSTR_TSA_USERTYPE(struct tg_config_t))
    FMSTR_TSA_RO_VAR(tg_states, FMSTR_TSA_USERTYPE(struct tg_states_t))
    
    FMSTR_TSA_STRUCT(struct tg_instance_t)
    FMSTR_TSA_MEMBER(struct tg_instance_t, chan_num_crank, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct tg_instance_t, chan_num_cam, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct tg_instance_t, priority, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct tg_instance_t, polarity_crank, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct tg_instance_t, polarity_cam, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct tg_instance_t, teeth_till_gap, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct tg_instance_t, teeth_in_gap, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct tg_instance_t, teeth_per_cycle, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct tg_instance_t, cam_edge_count, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct tg_instance_t, p_cam_edge_tooth, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct tg_instance_t, cpba, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct tg_instance_t, cpba8_cam_edge_tooth, FMSTR_TSA_UINT32)
    FMSTR_TSA_STRUCT(struct tg_config_t)
    FMSTR_TSA_MEMBER(struct tg_config_t, tooth_period_target, FMSTR_TSA_SINT32)
    FMSTR_TSA_MEMBER(struct tg_config_t, accel_ratio, FMSTR_TSA_UINT32)
    FMSTR_TSA_MEMBER(struct tg_config_t, generation_disable, FMSTR_TSA_UINT8)
    FMSTR_TSA_STRUCT(struct tg_states_t)
    FMSTR_TSA_MEMBER(struct tg_states_t, tooth_counter_cycle, FMSTR_TSA_UINT8)
    FMSTR_TSA_MEMBER(struct tg_states_t, tooth_period_actual, FMSTR_TSA_SINT32)
FMSTR_TSA_TABLE_END()
#endif

/*******************************************************************************
* FUNCTION: my_system_etpu_init
****************************************************************************//*!
* @brief   This function initialize the eTPU module:
*          -# Initialize global setting using fs_etpu_init function
*             and the my_etpu_config structure
*          -# On eTPU2, initialize the additional eTPU2 setting using
*             fs_etpu2_init function
*          -# Initialize channel setting using channel function APIs
*
* @return  Zero or an error code is returned.
*******************************************************************************/
int32_t my_system_etpu_init()
{
  int32_t err_code;

  /* this app is using original utility library and only using eTPU-AB */
  eTPU = eTPU_AB;

  /* Clear eTPU DATA RAM to make debugging easier */
  fs_memset32((uint32_t*)fs_etpu_data_ram_start, 0, fs_etpu_data_ram_end - fs_etpu_data_ram_start);
  
  /* Initialization of eTPU global settings */
  err_code = fs_etpu_init(
    &my_etpu_config,
    (uint32_t *)etpu_code, sizeof(etpu_code),
    (uint32_t *)etpu_globals, sizeof(etpu_globals));
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code);

#ifdef FS_ETPU_ARCHITECTURE
 #if FS_ETPU_ARCHITECTURE == ETPU2
  /* Initialization of additional eTPU2-only global settings */
  err_code = fs_etpu2_init(
    &my_etpu_config,
  #ifdef FS_ETPU_ENGINE_MEM_SIZE
    FS_ETPU_ENGINE_MEM_SIZE);
  #else
    0);
  #endif
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code);
 #endif
#endif

  /* Initialization of eTPU channel settings */
  err_code = fs_etpu_crank_init(
    &crank_instance,
    &crank_config);
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code + (ETPU_CRANK_CHAN<<16));

  err_code = fs_etpu_cam_init(
    &cam_instance,
    &cam_config);
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code + (ETPU_CAM_CHAN<<16));


  err_code = fs_etpu_spark_init(
    &spark_1_instance,
    &spark_config);
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code + (ETPU_SPARK_1_CHAN<<16));

  err_code = fs_etpu_spark_init(
    &spark_2_instance,
    &spark_config);
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code + (ETPU_SPARK_2_CHAN<<16));

  err_code = fs_etpu_spark_init(
    &spark_3_instance,
    &spark_config);
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code + (ETPU_SPARK_3_CHAN<<16));

  err_code = fs_etpu_spark_init(
    &spark_4_instance,
    &spark_config);
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code + (ETPU_SPARK_4_CHAN<<16));

  err_code = fs_etpu_fuel_init(
    &fuel_1_instance,
    &fuel_config);
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code + (ETPU_FUEL_1_CHAN<<16));

  err_code = fs_etpu_fuel_init(
    &fuel_2_instance,
    &fuel_config);
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code + (ETPU_FUEL_2_CHAN<<16));

  err_code = fs_etpu_fuel_init(
    &fuel_3_instance,
    &fuel_config);
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code + (ETPU_FUEL_3_CHAN<<16));

  err_code = fs_etpu_fuel_init(
    &fuel_4_instance,
    &fuel_config);
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code + (ETPU_FUEL_4_CHAN<<16));

  err_code = fs_etpu_inj_init(
    &inj_1_instance,
    &inj_config);
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code + (ETPU_INJ_1_CHAN<<16));

  err_code = fs_etpu_inj_init(
    &inj_2_instance,
    &inj_config);
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code + (ETPU_INJ_2_CHAN<<16));

  err_code = fs_etpu_inj_init(
    &inj_3_instance,
    &inj_config);
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code + (ETPU_INJ_3_CHAN<<16));

  err_code = fs_etpu_inj_init(
    &inj_4_instance,
    &inj_config);
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code + (ETPU_INJ_4_CHAN<<16));

  err_code = fs_etpu_knock_init(
    &knock_1_instance,
    &knock_1_config);
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code + (ETPU_KNOCK_1_CHAN<<16));

  err_code = fs_etpu_knock_init(
    &knock_2_instance,
    &knock_2_config);
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code + (ETPU_KNOCK_2_CHAN<<16));

  err_code = fs_etpu_tg_init(
    &tg_instance,
    &tg_config);
  if(err_code != FS_ETPU_ERROR_NONE) return(err_code + (ETPU_TG_CRANK_CHAN<<16));

  return(FS_ETPU_ERROR_NONE);
}

/*******************************************************************************
* FUNCTION: my_system_etpu_start
****************************************************************************//*!
* @brief   This function enables channel interrupts, DMA requests and "output
*          disable" feature on selected channels and starts TCR time bases using
*          Global Timebase Enable (GTBE) bit.
* @warning This function should be called after all device modules, including
*          the interrupt and DMA controller, are configured.
*******************************************************************************/
void my_system_etpu_start()
{
  /* Initialization of Interrupt Enable, DMA Enable
     and Output Disable channel options */
  fs_etpu_set_interrupt_mask_a(ETPU_CIE_A);
  fs_etpu_set_interrupt_mask_b(ETPU_CIE_B);
  fs_etpu_set_dma_mask_a(ETPU_DTRE_A);
  fs_etpu_set_dma_mask_b(ETPU_DTRE_B);
  fs_etpu_set_output_disable_mask_a(ETPU_ODIS_A, ETPU_OPOL_A);
  fs_etpu_set_output_disable_mask_b(ETPU_ODIS_B, ETPU_OPOL_B);

  /* Synchronous start of all TCR time bases */
  fs_timer_start();
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
