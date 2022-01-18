/*******************************************************************************
*
*  FREESCALE SEMICONDUCTOR INC. 2013-2014
*  ALL RIGHTS RESERVED
*  COPYRIGHT (c)
*
********************************************************************************
*
*  FILE NAME:  etpuc_tg.h
*
*******************************************************************************/
#ifndef __ETPUC_TG_H
#define __ETPUC_TG_H

/*******************************************************************************
*  Definitions
*******************************************************************************/
/* Host Service Requests */
#define TG_HSR_INIT                  7

/* Function Modes */
#define TG_FM0_POLARITY_LOW          0
#define TG_FM0_POLARITY_HIGH         1
#define TG_FM1_CRANK                 0
#define TG_FM1_CAM                   1

/* Generation Disable flags */
#define TG_GENERATION_ALLOWED        0
#define TG_GENERATION_DISABLED       1


/* TG eTPU function class declaration */
_eTPU_class TG
{
    /* channel frame */
public:
        uint24_t   tooth_tcr1_time; 
         int24_t   tooth_period_actual;
  const  int24_t   tooth_period_target;
  const ufract24_t accel_ratio;
  const uint8_t   *p_cam_tooth_first;
        uint8_t   *p_cam_tooth;
  const uint8_t    teeth_till_gap;
  const  int8_t    teeth_in_gap;
  const uint8_t    teeth_per_cycle;
        uint8_t    tooth_counter_gap;
        uint8_t    tooth_counter_cycle;
  const uint8_t    cam_chan;
  const uint8_t    generation_disable;


    /************************************/
    
    /* threads */
    
    _eTPU_thread INIT(_eTPU_matches_disabled);
    _eTPU_thread FIRST_EDGE(_eTPU_matches_disabled);
    _eTPU_thread SECOND_EDGE(_eTPU_matches_disabled);
    
    
    /************************************/
    
    /* methods and fragments */
    
    /* none */
    
    
    /************************************/
    
    /* entry table */
    
    _eTPU_entry_table TG;
};


#endif /* __ETPUC_TG_H */
/*********************************************************************
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
 ********************************************************************/

/*******************************************************************************
*
*  REVISION HISTORY:
*
*  FILE OWNER: Milan Brejl [r54529]
*  Revision 1.0  2014/03/06  r54529
*  Minor comment and formating improvements. MISRA compliancy check.
*  Ready for eTPU Engine Control Library release 1.0.
*
*  Revision 0.4  2013/10/29  r54529
*  generation_disable switch added.
*
*  Revision 0.3  2013/07/25  r54529
*  FM1 (polarity) option separated for Crank and Cam.
*
*  Revision 0.2  2013/06/19  r54529
*  Acceleration and deceleration added.
*
*  Revision 0.1  2012/11/28  r54529
*  Initial version.
*
*******************************************************************************/
