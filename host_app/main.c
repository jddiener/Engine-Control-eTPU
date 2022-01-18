/******************************************************************************
*
* (c) Copyright 2014, Freescale Semiconductor Inc.
*
***************************************************************************//*!
*
* @file     main.c
*
* @author   Milan Brejl
*
* @date     31-Jul-2014
*
* @brief    Automotive eTPU functions demo.
*
* @note     Make sure to have the following connections on your EVB:
*             ETPUA1 -> ETPUA0
*             ETPUA3 -> ETPUA2
*             
******************************************************************************/
#ifndef CPU32SIM
#include "MPC5674F.h"
#include "IntcInterrupts.h"
#include "freemaster.h"
#include "fs_gpio.h"       /* SIU pin configuration */
#else
#include "isrLib.h"
#include "scriptLib.h"
#endif
#include "main.h"
#include "etpu_util.h"     /* General C Functions for the eTPU */
#include "etpu_gct.h"      /* eTPU configuration */
#include "etpu_crank.h"    /* eTPU CRANK API */
#include "etpu_cam.h"      /* eTPU CAM API */
#include "etpu_spark.h"    /* eTPU SPARK API */
#include "etpu_fuel.h"     /* eTPU FUEL API */
#include "etpu_inj.h"      /* eTPU INJ API */
#include "etpu_knock.h"    /* eTPU KNOCK API */
#include "etpu_tg.h"       /* eTPU TG API */

/******************************************************************************
* Global variables
******************************************************************************/
/* eTPU Engine A load as a percentage */
uint32_t etpu_engine_load;

/* eTPU log arrays */
uint24_t etpu_cam_log[CAM_LOG_SIZE];
uint24_t etpu_tooth_period_log[TEETH_PER_CYCLE];

#define TCR22DEG(x) (x * 720.0 / TCR2_TICKS_PER_CYCLE)
/* current (sampled repeatably) engine position in degrees */
double engine_position;
/* current (sampled repeatably) engine speed in rpm */
double engine_speed;

#ifdef CPU32SIM
enum ETPU_ISR_TYPE
{
    EIT_INACTIVE,
    EIT_TG,
    EIT_CRANK,
    EIT_CAM,
    EIT_SPARK,
    EIT_FUEL,
    EIT_KNOCK,
    EIT_INJ,
};
enum ETPU_ISR_TYPE etpu_isr_active = EIT_INACTIVE;
volatile struct eTPU_struct * const eTPU;
uint32_t *fs_etpu_free_param;
int g_complete_flag = 0;
int g_testbed_flag = 0;
#endif

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

FMSTR_TSA_TABLE_BEGIN(fmstr_tsa_table_etpu_load)
    FMSTR_TSA_RO_VAR(etpu_engine_load, FMSTR_TSA_UINT32)
FMSTR_TSA_TABLE_END()

FMSTR_TSA_TABLE_BEGIN(fmstr_tsa_table_etpu_logs)
    FMSTR_TSA_RO_VAR(etpu_cam_log, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(etpu_tooth_period_log, FMSTR_TSA_UINT32)
FMSTR_TSA_TABLE_END()

/*
 * This list describes all TSA tables which should be exported to the 
 * FreeMASTER application.
 */
FMSTR_TSA_TABLE_LIST_BEGIN()
    FMSTR_TSA_TABLE(fmstr_tsa_table_etpu_load)
    FMSTR_TSA_TABLE(fmstr_tsa_table_etpu_logs)
    FMSTR_TSA_TABLE(fmstr_tsa_table_etpu_scaling)
    FMSTR_TSA_TABLE(fmstr_tsa_table_crank)
    FMSTR_TSA_TABLE(fmstr_tsa_table_cam)
    FMSTR_TSA_TABLE(fmstr_tsa_table_spark)
    FMSTR_TSA_TABLE(fmstr_tsa_table_fuel)
    FMSTR_TSA_TABLE(fmstr_tsa_table_inj)
    FMSTR_TSA_TABLE(fmstr_tsa_table_knock)
    FMSTR_TSA_TABLE(fmstr_tsa_table_tg)
FMSTR_TSA_TABLE_LIST_END()
#endif

/******************************************************************************
* Local function prototypes
******************************************************************************/
void gpio_init(void);
void fmpll_init(void);
void esci_a_init(void);
void intc_init(void);
uint32_t get_etpu_load_a(void);

/******************************************************************************
* Interrupt handlers
******************************************************************************/
void etpu_crank_isr(void);
void etpu_cam_isr(void);
void etpu_fuel_1_isr(void);
void etpu_fuel_2_isr(void);
void etpu_fuel_3_isr(void);
void etpu_fuel_4_isr(void);
void etpu_spark_1_isr(void);
void etpu_spark_2_isr(void);
void etpu_spark_3_isr(void);
void etpu_spark_4_isr(void);
void etpu_knock_1_isr(void);
void etpu_knock_2_isr(void);
void etpu_inj_1_isr(void);
void etpu_inj_2_isr(void);
void etpu_inj_3_isr(void);
void etpu_inj_4_isr(void);
void etpu_tg_isr(void);

#ifdef CPU32SIM
void aw_to_nxp_isr_translator(int32_t fint, uint32_t chan_mask)
{
    typedef void(*ISR_NXP_FUNC_PTR)(void);
    ISR_NXP_FUNC_PTR fptr = (ISR_NXP_FUNC_PTR)fint;
    /* eTPU->CISR_A.R = chan_mask; */
    (fptr)();
}
#endif

/***************************************************************************//*!
*
* @brief   Interrupt from eTPU channel CRANK
*
* @return  N/A
*
* @note    This interrupt is generated every time the engine position state
*          is changed. When the engine position state is 
*          FS_ETPU_ENG_POS_PRE_FULL_SYNC the logged Cam patern is decoded
*          in order to set tcr2_adjustment and achieve FULL_SYNC.
* 
******************************************************************************/
void etpu_crank_isr(void)
{
  uint24_t tcr2_adjustment;

#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_CRANK, 1);
#else
  etpu_isr_active = EIT_CRANK;
#endif

  fs_etpu_clear_chan_interrupt_flag(ETPU_CRANK_CHAN);
 
  /* Follow Engine Position state */
  fs_etpu_crank_get_states(&crank_instance, &crank_states);
  switch(crank_states.eng_pos_state)
  {
  case FS_ETPU_ENG_POS_SEEK:
    /* Crank has stalled. Read Crank error to know the reason. */
    break;
  case FS_ETPU_ENG_POS_FIRST_HALF_SYNC:
    /* Crank has found the gap, or
       Crank did not received CRANK_HSR_SET_SYNC and hence Crank reset 
       the Cam log to repeat recording the Cam log. */
    break;
  case FS_ETPU_ENG_POS_PRE_FULL_SYNC:
    /* Cam signal is logged, the log corresponds to a segment of 
       teeth_per_sync Crank teeth starting from a gap.
       NOW THE CPU MUST RECOGNIZE THE LOGGED CAM PATTERN AND 
       1) SET tcr2_adjustment
       2) ASSIGN CRANK HSR = CRANK_HSR_SET_SYNC. */
    fs_etpu_cam_get_states(&cam_instance, &cam_states);
    fs_etpu_cam_copy_log(&cam_instance, &etpu_cam_log[0]);
    
    if((cam_states.log_idx == 3) && (etpu_cam_log[0] & 0x01000000))
    {
      /* 3 transition logged,
         the first transition is a rising one 
         => the  first half-cycle was logged */
      /* Set, what angle should have been at the last gap */
      tcr2_adjustment = DEG2TCR2(360);
    }
    else if((cam_states.log_idx == 3) && ~(etpu_cam_log[0] & 0x01000000))
    {
      /* 3 transitions logged,
         the first transition is a falling one
         => the second half-cycle was logged */
      /* Set, what angle should have been at the last gap */
      tcr2_adjustment = DEG2TCR2(0);
    }
    else
    {
      /* Cam pattern is not recognized */
      break;
    }
    fs_etpu_crank_set_sync(&crank_instance, tcr2_adjustment);
    break;
  case FS_ETPU_ENG_POS_FULL_SYNC:
    /* Regular interrupt on the first tooth every engine cycle. */
    /* Clear errors */
    crank_states.error = 0;
    cam_states.error = 0;
    break;
  }

  /* Interface CRANK eTPU function */
  fs_etpu_crank_get_states(&crank_instance, &crank_states);
  fs_etpu_crank_config(&crank_instance, &crank_config);
  fs_etpu_crank_copy_tooth_period_log(&crank_instance, &etpu_tooth_period_log[0]);
  /* Interface CAM eTPU function */
  fs_etpu_cam_get_states(&cam_instance, &cam_states);
  fs_etpu_cam_config(&cam_instance, &cam_config);
  fs_etpu_cam_copy_log(&cam_instance, &etpu_cam_log[0]);

    /* Evaluate eTPU load */
  etpu_engine_load = get_etpu_load_a();

#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_CRANK, 0);
#else
  etpu_isr_active = EIT_INACTIVE;
#endif
}


/***************************************************************************//*!
*
* @brief   Interrupt from eTPU channel CAM
*
* @return  N/A
*
* @note    This interrupt is generated on detecting an error condition.
* 
******************************************************************************/
void etpu_cam_isr(void)
{
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_CAM, 1);
#else
  etpu_isr_active = EIT_CAM;
#endif

  fs_etpu_clear_chan_interrupt_flag(ETPU_CAM_CHAN);

  /* Interface CAM eTPU function */
  fs_etpu_cam_get_states(&cam_instance, &cam_states);
  fs_etpu_cam_config(&cam_instance, &cam_config);
  
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_CAM, 0);
#else
  etpu_isr_active = EIT_INACTIVE;
#endif
}


/***************************************************************************//*!
*
* @brief   Interrupt from eTPU channel FUEL_1
*
* @return  N/A
*
* @note    This interrupt is generated at each stop angle. The Fuel 
*          parameters can be adjusted.
* 
******************************************************************************/
void etpu_fuel_1_isr(void)
{
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_FUEL, 1);
#else
  etpu_isr_active = EIT_FUEL;
#endif

  fs_etpu_clear_chan_interrupt_flag(ETPU_FUEL_1_CHAN);

  fuel_1_states.error = 0;
  /* Interface FUEL eTPU function */
  fs_etpu_fuel_get_states(&fuel_1_instance, &fuel_1_states);
  fs_etpu_fuel_config(&fuel_1_instance, &fuel_config);
  
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_FUEL, 0);
#else
  etpu_isr_active = EIT_INACTIVE;
#endif
}


/***************************************************************************//*!
*
* @brief   Interrupt from eTPU channel FUEL_2
*
* @return  N/A
*
* @note    This interrupt is generated at each stop angle. The Fuel 
*          parameters can be adjusted.
* 
******************************************************************************/
void etpu_fuel_2_isr(void)
{
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_FUEL, 1);
#else
  etpu_isr_active = EIT_FUEL;
#endif

  fs_etpu_clear_chan_interrupt_flag(ETPU_FUEL_2_CHAN);

  fuel_2_states.error = 0;
  /* Interface FUEL eTPU function */
  fs_etpu_fuel_get_states(&fuel_2_instance, &fuel_2_states);
  fs_etpu_fuel_config(&fuel_2_instance, &fuel_config);
  
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_FUEL, 0);
#else
  etpu_isr_active = EIT_INACTIVE;
#endif
}


/***************************************************************************//*!
*
* @brief   Interrupt from eTPU channel FUEL_3
*
* @return  N/A
*
* @note    This interrupt is generated at each stop angle. The Fuel 
*          parameters can be adjusted.
* 
******************************************************************************/
void etpu_fuel_3_isr(void)
{
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_FUEL, 1);
#else
  etpu_isr_active = EIT_FUEL;
#endif

  fs_etpu_clear_chan_interrupt_flag(ETPU_FUEL_3_CHAN);

  fuel_3_states.error = 0;
  /* Interface FUEL eTPU function */
  fs_etpu_fuel_get_states(&fuel_3_instance, &fuel_3_states);
  fs_etpu_fuel_config(&fuel_3_instance, &fuel_config);
  
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_FUEL, 0);
#else
  etpu_isr_active = EIT_INACTIVE;
#endif
}


/***************************************************************************//*!
*
* @brief   Interrupt from eTPU channel FUEL_4
*
* @return  N/A
*
* @note    This interrupt is generated at each stop angle. The Fuel 
*          parameters can be adjusted.
* 
******************************************************************************/
void etpu_fuel_4_isr(void)
{
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_FUEL, 1);
#else
  etpu_isr_active = EIT_FUEL;
#endif

  fs_etpu_clear_chan_interrupt_flag(ETPU_FUEL_4_CHAN);

  fuel_4_states.error = 0;
  /* Interface FUEL eTPU function */
  fs_etpu_fuel_get_states(&fuel_4_instance, &fuel_4_states);
  fs_etpu_fuel_config(&fuel_4_instance, &fuel_config);
  
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_FUEL, 0);
#else
  etpu_isr_active = EIT_INACTIVE;
#endif
}


/***************************************************************************//*!
*
* @brief   Interrupt from eTPU channel SPARK_1
*
* @return  N/A
*
* @note    This interrupt is generated at each recalc angle. The Spark 
*          parameters can be adjusted.
* 
******************************************************************************/
void etpu_spark_1_isr(void)
{
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_SPARK, 1);
#else
  etpu_isr_active = EIT_SPARK;
#endif

  fs_etpu_clear_chan_interrupt_flag(ETPU_SPARK_1_CHAN);

  spark_1_states.error = 0;
  /* Interface SPARK eTPU function */
  fs_etpu_spark_get_states(&spark_1_instance, &spark_1_states);
  fs_etpu_spark_config(&spark_1_instance, &spark_config);
  
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_SPARK, 0);
#else
  etpu_isr_active = EIT_INACTIVE;
#endif
}


/***************************************************************************//*!
*
* @brief   Interrupt from eTPU channel SPARK_2
*
* @return  N/A
*
* @note    This interrupt is generated at each recalc angle. The Spark 
*          parameters can be adjusted.
* 
******************************************************************************/
void etpu_spark_2_isr(void)
{
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_SPARK, 1);
#else
  etpu_isr_active = EIT_SPARK;
#endif

  fs_etpu_clear_chan_interrupt_flag(ETPU_SPARK_2_CHAN);

  spark_2_states.error = 0;
  /* Interface SPARK eTPU function */
  fs_etpu_spark_get_states(&spark_2_instance, &spark_2_states);
  fs_etpu_spark_config(&spark_2_instance, &spark_config);
  
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_SPARK, 0);
#else
  etpu_isr_active = EIT_INACTIVE;
#endif
}


/***************************************************************************//*!
*
* @brief   Interrupt from eTPU channel SPARK_3
*
* @return  N/A
*
* @note    This interrupt is generated at each recalc angle. The Spark 
*          parameters can be adjusted.
* 
******************************************************************************/
void etpu_spark_3_isr(void)
{
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_SPARK, 1);
#else
  etpu_isr_active = EIT_SPARK;
#endif

  fs_etpu_clear_chan_interrupt_flag(ETPU_SPARK_3_CHAN);

  spark_3_states.error = 0;
  /* Interface SPARK eTPU function */
  fs_etpu_spark_get_states(&spark_3_instance, &spark_3_states);
  fs_etpu_spark_config(&spark_3_instance, &spark_config);
  
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_SPARK, 0);
#else
  etpu_isr_active = EIT_INACTIVE;
#endif
}


/***************************************************************************//*!
*
* @brief   Interrupt from eTPU channel SPARK_4
*
* @return  N/A
*
* @note    This interrupt is generated at each recalc angle. The Spark 
*          parameters can be adjusted.
* 
******************************************************************************/
void etpu_spark_4_isr(void)
{
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_SPARK, 1);
#else
  etpu_isr_active = EIT_SPARK;
#endif

  fs_etpu_clear_chan_interrupt_flag(ETPU_SPARK_4_CHAN);

  spark_4_states.error = 0;
  /* Interface SPARK eTPU function */
  fs_etpu_spark_get_states(&spark_4_instance, &spark_4_states);
  fs_etpu_spark_config(&spark_4_instance, &spark_config);
  
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_SPARK, 0);
#else
  etpu_isr_active = EIT_INACTIVE;
#endif
}


/***************************************************************************//*!
*
* @brief   Interrupt from eTPU channel KNOCK_1
*
* @return  N/A
*
* @note    This interrupt is generated at each window end. The Knock 
*          parameters can be adjusted.
* 
******************************************************************************/
void etpu_knock_1_isr(void)
{
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_KNOCK, 1);
#else
  etpu_isr_active = EIT_KNOCK;
#endif

  fs_etpu_clear_chan_interrupt_flag(ETPU_KNOCK_1_CHAN);

  /* Interface KNOCK eTPU function */
  fs_etpu_knock_config(&knock_1_instance, &knock_1_config);
  
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_KNOCK, 0);
#else
  etpu_isr_active = EIT_INACTIVE;
#endif
}


/***************************************************************************//*!
*
* @brief   Interrupt from eTPU channel KNOCK_2
*
* @return  N/A
*
* @note    This interrupt is generated at each window end. The Knock 
*          parameters can be adjusted.
* 
******************************************************************************/
void etpu_knock_2_isr(void)
{
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_KNOCK, 1);
#else
  etpu_isr_active = EIT_KNOCK;
#endif

  fs_etpu_clear_chan_interrupt_flag(ETPU_KNOCK_2_CHAN);

  /* Interface KNOCK eTPU function */
  fs_etpu_knock_config(&knock_2_instance, &knock_2_config);
  
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_KNOCK, 0);
#else
  etpu_isr_active = EIT_INACTIVE;
#endif
}


/***************************************************************************//*!
*
* @brief   Interrupt from eTPU channel INJ_1
*
* @return  N/A
*
* @note    This interrupt is generated before the start of injection sequence
*          on INJ 1 channel. The injection sequence parameters can be adjusted.
* 
******************************************************************************/
void etpu_inj_1_isr(void)
{
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_INJ, 1);
#else
  etpu_isr_active = EIT_INJ;
#endif

  fs_etpu_clear_chan_interrupt_flag(ETPU_INJ_1_CHAN);

  inj_1_states.error = 0;
  /* Interface INJ eTPU function */
  fs_etpu_inj_get_states(&inj_1_instance, &inj_1_states);
  fs_etpu_inj_config(&inj_1_instance, &inj_config);
  
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_INJ, 0);
#else
  etpu_isr_active = EIT_INACTIVE;
#endif
}


/***************************************************************************//*!
*
* @brief   Interrupt from eTPU channel INJ_2
*
* @return  N/A
*
* @note    This interrupt is generated before the start of injection sequence
*          on INJ 1 channel. The injection sequence parameters can be adjusted.
* 
******************************************************************************/
void etpu_inj_2_isr(void)
{
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_INJ, 1);
#else
  etpu_isr_active = EIT_INJ;
#endif

  fs_etpu_clear_chan_interrupt_flag(ETPU_INJ_2_CHAN);

  inj_2_states.error = 0;
  /* Interface INJ eTPU function */
  fs_etpu_inj_get_states(&inj_2_instance, &inj_2_states);
  fs_etpu_inj_config(&inj_2_instance, &inj_config);
  
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_INJ, 0);
#else
  etpu_isr_active = EIT_INACTIVE;
#endif
}


/***************************************************************************//*!
*
* @brief   Interrupt from eTPU channel INJ_3
*
* @return  N/A
*
* @note    This interrupt is generated before the start of injection sequence
*          on INJ 1 channel. The injection sequence parameters can be adjusted.
* 
******************************************************************************/
void etpu_inj_3_isr(void)
{
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_INJ, 1);
#else
  etpu_isr_active = EIT_INJ;
#endif

  fs_etpu_clear_chan_interrupt_flag(ETPU_INJ_3_CHAN);

  inj_3_states.error = 0;
  /* Interface INJ eTPU function */
  fs_etpu_inj_get_states(&inj_3_instance, &inj_3_states);
  fs_etpu_inj_config(&inj_3_instance, &inj_config);
  
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_INJ, 0);
#else
  etpu_isr_active = EIT_INACTIVE;
#endif
}


/***************************************************************************//*!
*
* @brief   Interrupt from eTPU channel INJ_4
*
* @return  N/A
*
* @note    This interrupt is generated before the start of injection sequence
*          on INJ 1 channel. The injection sequence parameters can be adjusted.
* 
******************************************************************************/
void etpu_inj_4_isr(void)
{
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_INJ, 1);
#else
  etpu_isr_active = EIT_INJ;
#endif

  fs_etpu_clear_chan_interrupt_flag(ETPU_INJ_4_CHAN);

  inj_4_states.error = 0;
  /* Interface INJ eTPU function */
  fs_etpu_inj_get_states(&inj_4_instance, &inj_4_states);
  fs_etpu_inj_config(&inj_4_instance, &inj_config);
  
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_INJ, 0);
#else
  etpu_isr_active = EIT_INACTIVE;
#endif
}


/***************************************************************************//*!
*
* @brief   Interrupt from eTPU channel TG
*
* @return  N/A
*
* @note    This interrupt is generated in each gap. The Tooth Generator 
*          parameters can be adjusted.
* 
******************************************************************************/
void etpu_tg_isr(void)
{
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_TG, 1);
#else
  etpu_isr_active = EIT_TG;
#endif

  fs_etpu_clear_chan_interrupt_flag(ETPU_TG_CRANK_CHAN);

  /* Interface TG eTPU function */
  fs_etpu_tg_get_states(&tg_instance, &tg_states);
  fs_etpu_tg_config(&tg_instance, &tg_config);
  
#ifndef CPU32SIM
  fs_gpio_write_data(TEST_PAD_TG, 0);
#else
  etpu_isr_active = EIT_INACTIVE;
#endif
}


/***************************************************************************//*!
*
* @brief   Main. Initialization and background loop.
*
* @return  N/A
*
* @note    The main routine includes only initialization and start of 
*          individual modules and a background loop. 
*
******************************************************************************/
int user_main(void) 
{
  double current_time;
  int test_step = 0;
  
#ifndef CPU32SIM
  /* Initialize GPIO, FMPLL, eSCI A */
  gpio_init();
  fmpll_init();
  esci_a_init();
#endif
  
  /* Initialize eTPU */
  my_system_etpu_init();

#ifndef CPU32SIM
  /* Initialize FreeMASTER */
  FMSTR_Init();
#endif

  /* Initialize interrupts */
  intc_init();

  /* Start eTPU */
  my_system_etpu_start();
  get_etpu_load_a();

#if 0  
  /* crank for 1 second before accelerating */
  while (1)
  {
    current_time = read_time();
    if (current_time >= 1000000 || g_testbed_flag) break;
  }
  tg_config.tooth_period_target = RPM2TP(5000);
  tg_config.accel_ratio = UFRACT24(0.01);
  fs_etpu_tg_config(&tg_instance, &tg_config);
#endif
  
  /* Loop forever */
  for (;;)
  {
    /* Set Fuel injection time - the value is updated in by FreeMASTER */
    fs_etpu_fuel_update_injection_time(&fuel_1_instance, &fuel_config);

    /* Interface TG eTPU function - this sets engine speed updated by FreeMASTER */
    fs_etpu_tg_get_states(&tg_instance, &tg_states);
    fs_etpu_tg_config(&tg_instance, &tg_config);
  
    /* update Crank and Cam latest states to see them in FreeMaster*/
    fs_etpu_crank_get_states(&crank_instance, &crank_states);
    fs_etpu_cam_get_states(&cam_instance, &cam_states);

    /* refresh current engine position */
    engine_position = TCR22DEG(fs_etpu_crank_get_angle_reseting());
    /* refresh current engine speed */
    engine_speed = TP2RPM(crank_states.last_tooth_period_norm);

    current_time = read_time();
    if (test_step == 0 && current_time > 60000.0)
    {
        //fuel_config.injection_time = USEC2TCR1(30000);
        tg_config.tooth_period_target = RPM2TP(2000);
        test_step = 1;
    }
    if (test_step == 1 && current_time > 66000.0)
    {
        //fuel_config.injection_time = USEC2TCR1(30000);
        tg_config.tooth_period_target = RPM2TP(5000);
        test_step = 2;
    }
    
#ifndef CPU32SIM
    /* FreeMASTER processing on background */
    FMSTR_Poll();
#else
    if (g_testbed_flag) break;
#endif
  }
#ifdef CPU32SIM
  g_complete_flag = 1;
#endif

  return 0;
}


/***************************************************************************//*!
*
* @brief   Initialize interrupts.
*
* @return  N/A
*
******************************************************************************/
void intc_init(void)
{
#ifndef CPU32SIM
	/* Install interrupt handlers */
	INTC_InstallINTCInterruptHandler(etpu_crank_isr,   68 + ETPU_CRANK_CHAN, 2);
	INTC_InstallINTCInterruptHandler(etpu_cam_isr,     68 + ETPU_CAM_CHAN, 2);
	INTC_InstallINTCInterruptHandler(etpu_fuel_1_isr,  68 + ETPU_FUEL_1_CHAN, 2);
	INTC_InstallINTCInterruptHandler(etpu_fuel_2_isr,  68 + ETPU_FUEL_2_CHAN, 2);
	INTC_InstallINTCInterruptHandler(etpu_fuel_3_isr,  68 + ETPU_FUEL_3_CHAN, 2);
	INTC_InstallINTCInterruptHandler(etpu_fuel_4_isr,  68 + ETPU_FUEL_4_CHAN, 2);
	INTC_InstallINTCInterruptHandler(etpu_spark_1_isr, 68 + ETPU_SPARK_1_CHAN, 2);
	INTC_InstallINTCInterruptHandler(etpu_spark_2_isr, 68 + ETPU_SPARK_2_CHAN, 2);
	INTC_InstallINTCInterruptHandler(etpu_spark_3_isr, 68 + ETPU_SPARK_3_CHAN, 2);
	INTC_InstallINTCInterruptHandler(etpu_spark_4_isr, 68 + ETPU_SPARK_4_CHAN, 2);
	INTC_InstallINTCInterruptHandler(etpu_knock_1_isr, 68 + ETPU_KNOCK_1_CHAN, 2);
	INTC_InstallINTCInterruptHandler(etpu_knock_2_isr, 68 + ETPU_KNOCK_2_CHAN, 2);
	INTC_InstallINTCInterruptHandler(etpu_inj_1_isr,   68 + ETPU_INJ_1_CHAN, 2);
	INTC_InstallINTCInterruptHandler(etpu_inj_2_isr,   68 + ETPU_INJ_2_CHAN, 2);
	INTC_InstallINTCInterruptHandler(etpu_inj_3_isr,   68 + ETPU_INJ_3_CHAN, 2);
	INTC_InstallINTCInterruptHandler(etpu_inj_4_isr,   68 + ETPU_INJ_4_CHAN, 2);
	INTC_InstallINTCInterruptHandler(etpu_tg_isr,      68 + ETPU_TG_CRANK_CHAN, 2);

	/* Enable interrupts */
	INTC.MCR.B.HVEN = 0;
	INTC.MCR.B.VTES = 0;
	INTC.CPR.B.PRI = 0;
	asm("wrteei 1");
#else
	/* initialize interrupt support */
	isrLibInit();
	/* enable interrupt acknowledgement */
	isrEnableAllInterrupts();

    isrConnect(ETPU_CAM_CHAN, aw_to_nxp_isr_translator, (int)&etpu_cam_isr, (1<<ETPU_CAM_CHAN)&0x1f);
    isrConnect(ETPU_CRANK_CHAN, aw_to_nxp_isr_translator, (int)&etpu_crank_isr, (1<<ETPU_CRANK_CHAN)&0x1f);
    isrConnect(ETPU_FUEL_1_CHAN, aw_to_nxp_isr_translator, (int)&etpu_fuel_1_isr, (1<<ETPU_FUEL_1_CHAN)&0x1f);
    isrConnect(ETPU_FUEL_2_CHAN, aw_to_nxp_isr_translator, (int)&etpu_fuel_2_isr, (1<<ETPU_FUEL_2_CHAN)&0x1f);
    isrConnect(ETPU_FUEL_3_CHAN, aw_to_nxp_isr_translator, (int)&etpu_fuel_3_isr, (1<<ETPU_FUEL_3_CHAN)&0x1f);
    isrConnect(ETPU_FUEL_4_CHAN, aw_to_nxp_isr_translator, (int)&etpu_fuel_4_isr, (1<<ETPU_FUEL_4_CHAN)&0x1f);
    isrConnect(ETPU_SPARK_1_CHAN, aw_to_nxp_isr_translator, (int)&etpu_spark_1_isr, (1<<ETPU_SPARK_1_CHAN)&0x1f);
    isrConnect(ETPU_SPARK_2_CHAN, aw_to_nxp_isr_translator, (int)&etpu_spark_2_isr, (1<<ETPU_SPARK_2_CHAN)&0x1f);
    isrConnect(ETPU_SPARK_3_CHAN, aw_to_nxp_isr_translator, (int)&etpu_spark_3_isr, (1<<ETPU_SPARK_3_CHAN)&0x1f);
    isrConnect(ETPU_SPARK_4_CHAN, aw_to_nxp_isr_translator, (int)&etpu_spark_4_isr, (1<<ETPU_SPARK_4_CHAN)&0x1f);
    isrConnect(ETPU_KNOCK_1_CHAN, aw_to_nxp_isr_translator, (int)&etpu_knock_1_isr, (1<<ETPU_KNOCK_1_CHAN)&0x1f);
    isrConnect(ETPU_KNOCK_2_CHAN, aw_to_nxp_isr_translator, (int)&etpu_knock_2_isr, (1<<ETPU_KNOCK_2_CHAN)&0x1f);
    isrConnect(ETPU_INJ_1_CHAN, aw_to_nxp_isr_translator, (int)&etpu_inj_1_isr, (1<<ETPU_INJ_1_CHAN)&0x1f);
    isrConnect(ETPU_INJ_2_CHAN, aw_to_nxp_isr_translator, (int)&etpu_inj_2_isr, (1<<ETPU_INJ_2_CHAN)&0x1f);
    isrConnect(ETPU_INJ_3_CHAN, aw_to_nxp_isr_translator, (int)&etpu_inj_3_isr, (1<<ETPU_INJ_3_CHAN)&0x1f);
    isrConnect(ETPU_INJ_4_CHAN, aw_to_nxp_isr_translator, (int)&etpu_inj_4_isr, (1<<ETPU_INJ_4_CHAN)&0x1f);
    isrConnect(ETPU_TG_CRANK_CHAN, aw_to_nxp_isr_translator, (int)&etpu_tg_isr, (1<<ETPU_TG_CRANK_CHAN)&0x1f);
#endif
}

#ifndef CPU32SIM
/***************************************************************************//*!
*
* @brief   Init device I/O pin.
*
* @return  N/A
*
******************************************************************************/
void gpio_init(void)
{
  const uint16_t gpio_output_pad_config =
    FS_GPIO_IO_FUNCTION +
    FS_GPIO_MAXIMUM_SLEW_RATE + 
    FS_GPIO_OUTPUT_DRAIN_DISABLE +
    FS_GPIO_READBACK_ENABLE +
    FS_GPIO_OUTPUT_BUFFER_ENABLE;
  const uint16_t etpu_output_pad_config =
    FS_GPIO_PRIMARY_FUNCTION +
    FS_GPIO_MAXIMUM_SLEW_RATE + 
    FS_GPIO_OUTPUT_DRAIN_DISABLE +
    FS_GPIO_READBACK_ENABLE +
    FS_GPIO_OUTPUT_BUFFER_ENABLE;
  const uint16_t etpu_input_pad_config =
    FS_GPIO_PRIMARY_FUNCTION +
    FS_GPIO_INPUT_BUFFER_ENABLE;
  
  /* initialize eTPU inputs and outputs */
  fs_gpio_config(FS_GPIO_ETPUA0, etpu_input_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA1, etpu_output_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA2, etpu_input_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA3, etpu_output_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA4, etpu_output_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA5, etpu_output_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA6, etpu_output_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA7, etpu_output_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA8, etpu_output_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA9, etpu_output_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA10, etpu_output_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA11, etpu_output_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA12, etpu_output_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA13, etpu_output_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA14, etpu_output_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA15, etpu_output_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA16, etpu_output_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA17, etpu_output_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA18, etpu_output_pad_config);
  fs_gpio_config(FS_GPIO_ETPUA19, etpu_output_pad_config);
  /* test pad outputs */
  fs_gpio_config(TEST_PAD_TG,    gpio_output_pad_config);
  fs_gpio_config(TEST_PAD_CRANK, gpio_output_pad_config);
  fs_gpio_config(TEST_PAD_CAM,   gpio_output_pad_config);
  fs_gpio_config(TEST_PAD_SPARK, gpio_output_pad_config);
  fs_gpio_config(TEST_PAD_FUEL,  gpio_output_pad_config);
  fs_gpio_config(TEST_PAD_KNOCK, gpio_output_pad_config);
  fs_gpio_config(TEST_PAD_INJ,   gpio_output_pad_config);
}

/***************************************************************************//*!
*
* @brief   Init FMPLL - 100MHz System Clock.
*
* @return  N/A
*
******************************************************************************/
void fmpll_init(void)
{
  volatile uint32_t timeout = 0;
      
  /* system clock divider is bypassed */
  SIU.SYSDIV.B.BYPASS = 1;          

  /* set the default Output Divide Ratios (RDF) to highest value */ 
  FMPLL.ESYNCR2.B.ERFD = 63;

  /* set the enhanced Pre-Divider to 10 */
  FMPLL.ESYNCR1.B.EPREDIV = 9;

  /* set the Feedback Divide Ratios (EMFD) to 84 */
  FMPLL.ESYNCR1.B.EMFD = 84;

  /* Clock Mode Selection - Normal mode with crystal reference */
  FMPLL.ESYNCR1.B.CLKCFG = 7;

  /* Wait for PLL to LOCK  */
  while ((FMPLL.SYNSR.B.LOCK==0) && (timeout < 10000))
  {
    timeout++;      /* Just do a simple software based loop counter */
  } 

  if (timeout < 10000 )
  {
    /* PLL locked - update the EFRD field with computed value */
    FMPLL.ESYNCR2.B.ERFD = 1;
  }
}

/***************************************************************************//*!
*
* @brief   Init eSCI A to 115kbd @ 100MHz, for FreeMASTER.
*
* @return  N/A
*
******************************************************************************/
void esci_a_init(void)
{
  SIU.PCR[89].B.PA =1;               // Pin asigned to ESCI A Tx
  SIU.PCR[89].B.OBE =1;              // Open drain enable
  SIU.PCR[90].B.PA =1;               // Pin asigned to ESCI A Rx
  SIU.PCR[90].B.IBE =1;              // Input buffer enable

  ESCI_A.LCR.B.LIN= 0;               // disable LIN and enable SCI
  ESCI_A.CR2.R = 0x2000;             // Enable ESCI and set all bits to reset value
  
  ESCI_A.CR1.B.TE = 1;               // receiver enable
  ESCI_A.CR1.B.RE = 1;               // transmitter enable
  ESCI_A.CR1.B.PT = 0;               // parity is even
  ESCI_A.CR1.B.PE = 0;               // parity control disable
  ESCI_A.CR1.B.SBR = 53;             // Baud rate = 115200 @ 100MHz
}
#endif

/***************************************************************************//*!
*
* @brief   Evaluate eTPU engine A load.
*
* @warning - This function is applicable on eTPU2 only.
*          - The first call of this function does not return a correct result.
*          - This function presumes the TCR1 clock is configured for system 
*            clock div 1.
*
* @return  eTPU engine A load percentage (0 - 100).
*
******************************************************************************/
uint32_t get_etpu_load_a(void)
{
  static uint24_t tcr1;
         uint24_t tcr1_last;
         uint24_t idle_cnt;
         uint24_t time_cnt;

  tcr1_last = tcr1;
  tcr1 = eTPU->TB1R_A.R;
  idle_cnt = fs_etpu_get_idle_cnt_a();
  fs_etpu_clear_idle_cnt_a();

  time_cnt = 0xFFFFFF & (tcr1 - tcr1_last);
  time_cnt /= 2;  /* compensate TCR1 prescaler (TCR1=sysclk) */
  return(100*(time_cnt - idle_cnt)/time_cnt);
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
