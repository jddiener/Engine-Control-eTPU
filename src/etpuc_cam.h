/*******************************************************************************
*
*  FREESCALE SEMICONDUCTOR INC. 2012-2014
*  ALL RIGHTS RESERVED
*  COPYRIGHT (c)
*
********************************************************************************
*
*  FILE NAME:  etpuc_cam.h
*
*******************************************************************************/
#ifndef __ETPUC_CAM_H
#define __ETPUC_CAM_H

/*******************************************************************************
*  Definitions
*******************************************************************************/
/* Host Service Requests */
#define CAM_HSR_INIT                  7
#define CAM_HSR_RESET                 1

/* Function Modes */
#define CAM_FM0_LOG_FALLING           1
#define CAM_FM1_LOG_RISING            1

/* Cam log transition types */
#define CAM_FALLING                   0
#define CAM_RISING                    1

/* Error codes */
/* - IRQ is set together with setting an error bit */
#define CAM_ERROR_NO                  0
#define CAM_ERROR_ZERO_TRANS          1  /* no transition detected between 
                                            last 2 resets (links) */
#define CAM_ERROR_LOG_OVERFLOW        2  /* log array is too small to log all 
                                            transitions */

/* Data Structure Types */
typedef struct CAM_LOG
{
	  int8_t trans;     /* transition: 0-falling, 1-rising */
	uint24_t angle;     /* TCR2 counter value captured */
};

/* CAM eTPU function class declaration */
_eTPU_class CAM
{
    /* channel frame */
public:
	const uint24_t log_size;
	      uint24_t log_idx;
	      uint24_t log_count;
	      uint8_t  error;
	const struct CAM_LOG *log;


    /************************************/
    
    /* threads */
    
    _eTPU_thread INIT(_eTPU_matches_disabled);
    _eTPU_thread RESET(_eTPU_matches_enabled);
    _eTPU_thread LOG_1_TRANS(_eTPU_matches_enabled);
    _eTPU_thread LOG_2_TRANS(_eTPU_matches_enabled);
    
    
    /************************************/
    
    /* methods and fragments */
    
    /* none */
    
    
    /************************************/
    
    /* entry table */
    
    _eTPU_entry_table CAM;
};


#endif /* __ETPUC_CAM_H */

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
*  Revision 1.0  2014/03/05  r54529
*  Minor comment and formating improvements. MISRA compliancy check.
*  Ready for eTPU Engine Control Library release 1.0.
*
*  Revision 0.1  2012/06/12  r54529
*  Initial version.
*
*******************************************************************************/
