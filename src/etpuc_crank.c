/*******************************************************************************
* This file has been modified by ASH WARE Inc. as allowed per the original
* license (see bottom of file), to add features and fix issues.
*******************************************************************************/

/*******************************************************************************
*
*  FREESCALE SEMICONDUCTOR INC. 2012-2014
*  ALL RIGHTS RESERVED
*  COPYRIGHT (c)
*
********************************************************************************
*
*  FILE NAME:  etpuc_crank.c
*
*  DESCRIPTION:
*    This eTPU function processes the tooth signal from a crankshaft sensor and
*    generates an internal angle-base using the Enhanced Angle Counter
*    (EAC) eTPU hardware. The Angle Base is maintained in TCR2.
*    The Crank eTPU function keeps track of the overall synchronization state.
*    It reports via interrupt to the CPU when the overall synchronization state
*    changes. Additionally, and interrupt and DMA request (eTPU2 only) is
*    generated periodically every engine cycle.
*
*    There are differences in functionality between this function and
*    the original set2 Crank. In order to achieve the full synchronization state,
*    this function asks the CPU to decode a Cam pattern logged during a defined
*    number of Crank teeth (teeth_per_sync) and set the angle value of the last
*    gap within 0-720. Thanks to this, the Crank (and Cam) function is much more
*    general and enables to handle a wide range of Crank patters:
*    - a single gap pattern or a multiple equally-spaced gaps
*    - a 1, 2 or 3-teeth gap (eTPU and eTPU2) or up to 7-teeth gap (eTPU2+)
*    - an additional tooth instead of the gap
*    Generally any kind of a Cam pattern, or even a pattern of more Cam signals,
*    can be handled, because the Cam log is decoded by the CPU.
*
*******************************************************************************/

/*******************************************************************************
*  Includes
*******************************************************************************/
#include <etpu_std.h>
#include "etpuc_crank.h"
#include "etpuc_set.h"

#pragma verify_version GE, "3.00A", "use ETEC version 3.00A or newer"

/*******************************************************************************
*  eTPU Function Parameters:
*
*   teeth_till_gap         - number of physical teeth gap to gap
*   teeth_in_gap           - number if missing teeth in the gap.
*                            If there is an additional tooth instead of the gap,
*                            this parameter must be set to 0.
*   teeth_per_cycle        - number of teeth (including missing teeth in gap)
*                            per an engine cycle (720 degrees). It must be
*                            a multiple of (teeth_till_gap + teeth_in_gap).
*   teeth_per_sync         - number of teeth (including missing teeth in gap)
*                            corresponding to a segment needed for Cam log.
*                            It must be a multiple of
*                            (teeth_till_gap + teeth_in_gap).
*   tooth_counter_gap      - it counts from 1 to teeth_till_gap
*   tooth_counter_cycle    - it counts from 1 to teeth_per_cycle
*   blank_teeth            - number of teeth ignored after initialization
*   blank_time             - TCR1 time period after initialization during which
*                            teeth are ignored,
*   tcr2_ticks_per_tooth   - number of TCR2 angle ticks per tooth
*   tcr2_ticks_per_add_tooth-number of TCR2 angle ticks from the last tooth to the additional tooth
*   last_tooth_tcr1_time   - TCR1 time of the last tooth transition
*   last_tooth_period      - TCR1 period between last 2 teeth
*   last_tooth_period_norm - TCR1 period normalized over the gap or additional tooth
*   tcr2_adjustment        - TCR2 angle value corresponding to the angle on the
*                            first tooth after gap, at which the PRE_FULL_SYNC
*                            state was set and CPU was asked to recognize the
*                            Cam log pattern.
*   gap_ratio              - fraction used to perform the ABA gap test:
*                              gap_ratio * tooth_period_B > tooth_period_A
*   win_ratio_normal       - fraction used to derive the acceptance window for
*                            the next normal tooth
*   win_ratio_across_gap   - fraction used to derive the acceptance window for
*                            the first tooth after the gap
*   win_ratio_after_gap    - fraction used to derive the acceptance window for
*                            the second tooth after the gap
*   win_ratio_after_timeout- fraction used to derive the acceptance window for
*                            the tooth following a timeout condition
*   first_tooth_timeout    - TCR1 time after the first tooth (after blank_teeth)
*                            when a timeout will be deemed to have happened
*   link_cam               - set of 4 link numbers to send to reset the Cam log
*                            (up to 4 Cam channel numbers)
*   link_1                 - the first  set of 4 link numbers to send on stall
*   link_2                 - the second set of 4 link numbers to send on stall
*   link_3                 - the third  set of 4 link numbers to send on stall
*   link_4                 - the fourth set of 4 link numbers to send on stall
*   state                  - used to keep track of the CRANK state. See header
*                            file for possible values.
*   error                  - crank error flags. See header file for individual
*                            bits meaning. The eTPU sets them, the CPU should
*                            read and clear.
*   *tooth_period_log      - pointer to an array of tooth periods.
*                            The array must include teeth_per_cycle items.
*   err2477_tcr2_target    - used to keep track of when the Angle Counter is not
*                            in high rate mode for errata 2477 workaround
*
********************************************************************************
*
*  Channel Flag usage
*    Flag0 is used to handle a crank wheel with gap(s) and a crank wheel with
*      additional tooth separately:
*      - CRANK_FLAG0_GAP
*      - CRANK_FLAG0_ADDITIONAL_TOOTH
*    Flag1 is not used.
*
********************************************************************************
*
*  Channel Function Mode (FM) bits usage
*    FM0 is used to select the polarity of crank transition to detect:
*      - CRANK_FM0_USE_TRANS_RISING
*      - CRANK_FM0_USE_TRANS_FALLING
*    FM1 is used to turn on logging of crank tooth periods:
*      - CRANK_FM1_LOG_TOOTH_PERIODS
*
********************************************************************************
*
*  Channel Interrupt usage
*    The channel interrupt on the CRANK channel is set when the global Engine
*    Position State is changed and on the first tooth every engine cycle when
*    the global Engine Position State is FULL_SYNC.
*    The Crank channel interrupt handler should follow this example:
*
*    eTPUCrankInterruptHandler()
*    {
*      * Read Engine Position State eng_pos_state.
*      switch(eng_pos_state)
*      {
*      case FS_ETPU_ENG_POS_SEEK:
*        * Crank has stalled. Read Crank error to know the reason.
*        break;
*      case FS_ETPU_ENG_POS_FIRST_HALF_SYNC:
*        * Crank has found the gap, or
*        * Crank did not received CRANK_HSR_SET_SYNC and hence Crank reset
*        * the Cam log to repeat recording the Cam log.
*        break;
*      case FS_ETPU_ENG_POS_PRE_FULL_SYNC:
*        * Cam signal is logged, the log corresponds to a segment of
*        * teeth_per_sync Crank teeth starting from a gap.
*        * NOW THE CPU MUST RECOGNIZE THE LOGGED CAM PATTERN AND
*        * 1) SET tcr2_adjustment
*        * 2) ASSIGN CRANK HSR = CRANK_HSR_SET_SYNC.
*        break;
*      case FS_ETPU_ENG_POS_FULL_SYNC:
*        * Regular interrupt on the first tooth every engine cycle.
*        break;
*      }
*    }
*
*******************************************************************************/

/*******************************************************************************
*  Global Variables
*******************************************************************************/
uint8_t   eng_pos_state;
const uint24_t  eng_cycle_tcr2_ticks = 0; /* initilaized by host driver */
uint24_t  eng_cycle_tcr2_start;
uint24_t  eng_trr_norm = 0xffffff;


/*******************************************************************************
*  Global Functions
*******************************************************************************/

#define TRR_FRACTIONAL_BITS         9
#define TRR_HIGHRES_REMAINDER_SHIFT 6
#define TRR_HIGHRES_DIVISOR_MASK    0xfc0000 /* # bits matches remainder shift */

/*******************************************************************************
*  FUNCTION NAME: CRANK_Time_to_Angle_HighRes
*  DESCRIPTION: Converts a time in TCR1 ticks to an angle value, using the current
*    angle velocity.  This version is more computationally intensive, but yields
*    a more accurate result.  Should be used when calculating the match value for
*    a signal edge.  Accuracy is always within 1 TCR2 tick of theoretical
*    rounded floating point division.
*******************************************************************************/
int24_t CRANK_Time_to_Angle_HighRes(
    register_a uint24_t time)
{
    uint24_t atr = eng_trr_norm;
    uint24_t time_shift = 1;
    uint24_t round_up;
    uint24_t result;
    register _MACH uint24_t remainder;
    register _MACL uint24_t quotient;

    /* step 1 : maximize dividend to improve integer division accuracy */

    time <<= 1; /* times passed to this routine must be less than 0x800000, can shift by 1 immediately */
    while ((time & 0x800000) == 0)
    {
        if (time < 0x10000)
        {
            time <<= 8;
            time_shift += 8;
        }
        else if (time < 0x100000)
        {
            time <<= 4;
            time_shift += 4;
        }
        else
        {
            time <<= 1;
            time_shift += 1;
        }
    }

    /* step 2: decrease the number of dividend fractional bits if it is large */

    round_up = 0;
    while (atr & TRR_HIGHRES_DIVISOR_MASK)
    {
        round_up = atr & 1;
        atr >>= 1;
        time_shift += 1;
    }
    atr += round_up;

    /* step 3: perform first division */

    result = time / atr;
    remainder <<= TRR_HIGHRES_REMAINDER_SHIFT;

    /* step 4: divide remainder from first division to gain resolution for the result */

    if (time_shift <= 4)
    {
        uint24_t tmp;

        /* in this case the divisor resolution must be reduced in steps to improve final accuracy */

        /* put result in final resolution */
        result <<= 4;
        time_shift += 4;
        tmp = 9 - time_shift;
        while (tmp-- > 0)
            result <<= 1;

        /* only down-res the divisor by 4 bits for the second divide */
        atr >>= 3;
        round_up = atr & 1;
        atr >>= 1;
        atr += round_up;

        /* peform second divide and add its result to final result sum */
        tmp = time_shift - (TRR_FRACTIONAL_BITS - TRR_HIGHRES_REMAINDER_SHIFT);
        quotient = remainder / atr;
        while (tmp-- > 0)
            quotient >>= 1;
        result += quotient;
        remainder <<= TRR_HIGHRES_REMAINDER_SHIFT;

        /* perform final divisor de-resolution as necessary to ensure last division has resolution to final bit */
        round_up = 0;
        while (time_shift < 9)
        {
            round_up = atr & 1;
            atr >>= 1;
            time_shift += 1;
        }
        atr += round_up;
    }
    else
    {
        /* in this case desired result accuracy can be achieved by going direct to final divisor resolution */

        round_up = 0;
        while (time_shift < TRR_FRACTIONAL_BITS)
        {
            result <<= 1;
            round_up = atr & 1;
            atr >>= 1;
            time_shift += 1;
        }
        atr += round_up;

        /* peform second divide and add its result to final result sum */
        result += (remainder / atr) >> TRR_HIGHRES_REMAINDER_SHIFT;
        remainder <<= TRR_HIGHRES_REMAINDER_SHIFT;
    }

    /* step 5: perform 3rd division on remainder to get final bits of accuracy */

    result += (remainder / atr) >> (TRR_HIGHRES_REMAINDER_SHIFT * 2);
    /* down-res final result if too high (still contains fractional bits) */
    round_up = 0;
    while (time_shift > TRR_FRACTIONAL_BITS)
    {
        round_up = result & 1;
        result >>= 1;
        time_shift -= 1;
    }
    result += round_up;

    return result;
}

/*******************************************************************************
*  FUNCTION NAME: CRANK_Time_to_Angle_LowRes
*  DESCRIPTION: Converts a time in TCR1 ticks to an angle value, using the current
*    angle velocity.  This version is quick but has lower resolution output.
*******************************************************************************/
int24_t CRANK_Time_to_Angle_LowRes(
    register_a uint24_t time)
{
    return (time / (uint24_t)(eng_trr_norm >> 6)) << 3;
}


/*******************************************************************************
*  eTPU Class Methods/Fragments
*******************************************************************************/

/*******************************************************************************
*  FUNCTION NAME: Window_NoReturn
*  DESCRIPTION: Schedule transition acceptance window using win_ratio and
*    tooth_period, and end the thread.
*******************************************************************************/
_eTPU_fragment CRANK::Window_NoReturn(
    register_a fract24_t win_ratio,
    register_d uint24_t tooth_period)
{
    uint24_t half_window_width;

    half_window_width = muliur(tooth_period, win_ratio);
    erta = erta + tooth_period - half_window_width;
    ertb = erta + (half_window_width << 1);
    channel.MRLA = MRL_CLEAR;
    channel.MRLB = MRL_CLEAR;
    channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
    channel.ERWB = ERW_WRITE_ERT_TO_MATCH;
    /**
    * Only clear TDLA if it was what triggered this channel service.
    * It is only cleared AFTER the window has been set up so that any noise transitions are properly ignored
    */
    if (cc.TDLA == 1)
        channel.TDL = TDL_CLEAR;
}

/*******************************************************************************
*  FUNCTION NAME: WindowAcrossGap_NoReturn
*  DESCRIPTION: Open transition acceptance window using win_ratio and
*    close it using win_ratio_across_gap and the number of teeth_in_gap,
*    and end the thread.
*******************************************************************************/
_eTPU_fragment CRANK::WindowAcrossGap_NoReturn(
    register_a uint24_t tooth_period)
{
    uint24_t half_window_width;

    half_window_width = muliur(tooth_period, win_ratio_across_gap);
    ertb = erta + (tooth_period * (teeth_in_gap + 1U))
        + half_window_width;
    half_window_width = muliur(tooth_period, win_ratio_normal);
    erta = erta + tooth_period - half_window_width;
    channel.MRLA = MRL_CLEAR;
    channel.MRLB = MRL_CLEAR;
    channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
    channel.ERWB = ERW_WRITE_ERT_TO_MATCH;
    /**
    * Only clear TDLA if it was what triggered this channel service.
    * It is only cleared AFTER the window has been set up so that any noise transitions are properly ignored
    */
    if (cc.TDLA == 1)
        channel.TDL = TDL_CLEAR;
}

/*******************************************************************************
*  FUNCTION NAME: WindowCloseAt_NoReturn
*  DESCRIPTION: Open transition acceptance window immediately and
*    close it at close_tcr1_time, and end the thread.
*******************************************************************************/
_eTPU_fragment CRANK::WindowCloseAt_NoReturn(
    register_a uint24_t close_tcr1_time)
{
    erta = tcr1;
    ertb = close_tcr1_time;
    channel.MRLA = MRL_CLEAR;
    channel.MRLB = MRL_CLEAR;
    channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
    channel.ERWB = ERW_WRITE_ERT_TO_MATCH;
}

/*******************************************************************************
*  FUNCTION NAME: WindowClose_NoReturn
*  DESCRIPTION: Open transition acceptance window immediately and close using
*    win_ratio and tooth_period, and end the thread.
*******************************************************************************/
_eTPU_fragment CRANK::WindowClose_NoReturn(
    register_a fract24_t win_ratio,
    register_d uint24_t tooth_period)
{
    uint24_t half_window_width;

    half_window_width = muliur(tooth_period, win_ratio);
    ertb = erta + tooth_period + half_window_width;
    erta = tcr1;
    channel.MRLA = MRL_CLEAR;
    channel.MRLB = MRL_CLEAR;
    channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
    channel.ERWB = ERW_WRITE_ERT_TO_MATCH;
}

/*******************************************************************************
*  FUNCTION NAME: Stall_NoReturn
*  DESCRIPTION: Revert to FIRST_TRANS, signal other functions that Crank has
*    stalled.
*******************************************************************************/
_eTPU_fragment CRANK::Stall_NoReturn(void)
{
    /* set error */
    error |= CRANK_ERR_STALL;
    /* set state */
    state = CRANK_FIRST_TRANS;
    /* set global eng_pos state and channel interrupt */
    eng_pos_state = ENG_POS_SEEK;
    /* Channel interrupt */
    channel.CIRC = CIRC_INT_FROM_SERVICED;
    /* signal other functions that crank restarts */
    Link4(link_1);
    Link4(link_2);
    Link4(link_3);
    Link4(link_4);
    /* set default values */
    eng_trr_norm = trr = 0xffffff;
    tpr = 0;
    /* reset TCR2 if it is in a range that could cause immediate macthes to occur when
       dependent channels (fuel, spark, etc.) re-initialize, otherwise it will be reset
       in CRANK_FIRST_TRANS to prevent short spurious outputs between now and dependent
       channel re-initialization. */
    if ((unsigned int24)tcr2 < 0x800000U + 2 * eng_cycle_tcr2_ticks)
    {
        tcr2 = 0;
    }
    eng_cycle_tcr2_start = eng_cycle_tcr2_ticks;
    tooth_counter_gap = 0;
    tooth_counter_cycle = 0;
    last_tooth_tcr1_time = 0;
    last_tooth_period = 0;
    last_tooth_period_norm = 0;
    /* open the acceptance window immediately and do not close it */
    erta = tcr1;
    channel.MRLA = MRL_CLEAR;
    channel.MRLB = MRL_CLEAR;
    channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
}

/*******************************************************************************
*  FUNCTION NAME: ToothTcr2Sync_NoReturn
*  DESCRIPTION: Set a match for the tooth-tcr2 sync point so that TCR2 can be
*    reset at a known safe time.
*******************************************************************************/
_eTPU_fragment CRANK::ToothTcr2Sync_NoReturn()
{
    /* switch to match on TCR2 */
    channel.TBSA = TBS_M2C1GE;
    channel.TBSB = TBS_M2C2GE;
    if (teeth_per_cycle > teeth_per_sync)
        erta = ertb = eng_cycle_tcr2_ticks >> 1;
    else
        erta = ertb = eng_cycle_tcr2_ticks;
    channel.MRLA = MRL_CLEAR;
    channel.MRLB = MRL_CLEAR;
    channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
    channel.ERWB = ERW_WRITE_ERT_TO_MATCH;

    /**
    * TDL only cleared AFTER the window has been set up so that any noise transitions are properly ignored
    */
    channel.TDL = TDL_CLEAR;

    channel.FLAG1 = CRANK_FLAG1_TOOTH_TCR2_SYNC;
    state = CRANK_TOOTH_TCR2_SYNC;
}

/*******************************************************************************
*  FUNCTION NAME: ToothArray_Log
*  DESCRIPTION: If enabled (FM1 set) log tooth_period
*    into the tooth_period_log array at position [tooth_counter_cycle-1].
*******************************************************************************/
void CRANK::ToothArray_Log(
    register_a uint24_t tooth_period)
{
    uint24_t* ptr;

    if (cc.FM1 == CRANK_FM1_LOG_TOOTH_PERIODS)
    {
        ptr = tooth_period_log + (tooth_counter_cycle - 1);
        *ptr = tooth_period;
    }
}

/*******************************************************************************
*  FUNCTION NAME: Set_TRR
*  DESCRIPTION: Calculates the tick rate and sets the Tick Rate Register (TRR).
*******************************************************************************/
void CRANK::Set_TRR(
    register_a uint24_t tooth_period_norm)
{
    register_mach uint24_t mach; /* MAC High register (keeps reminder after division */
    int24_t tmp = 0;

    /* calculate and apply acceleration compensation */
    if (trr != 0xffffff)
    {
        /* calculate the acceleration */
        tmp = tooth_period_norm - last_last_tooth_period_norm;
        /* dampen the adjustment by 25% */
        tmp = mulir(tmp, 0.75);
    }
    last_last_tooth_period_norm = tooth_period_norm;

    eng_trr_norm = (((tooth_period_norm + tmp) / tcr2_ticks_per_tooth) << TRR_FRACTIONAL_BITS); /* integer part of TRR */
    eng_trr_norm += (mach << TRR_FRACTIONAL_BITS) / tcr2_ticks_per_tooth;                       /* fractional part of TRR */
    if (tcr1_clock_source_div1)
    {
        /* if the TCR1 clock source is the full eTPU clock, the tooth period input
           to the TRR calculation needs to be compensated by dividing by 2 */
        trr = eng_trr_norm >> 1;
    }
    else
    {
        trr = eng_trr_norm;
    }
}


/*******************************************************************************
*  eTPU Function
*******************************************************************************/

/**************************************************************************
* THREAD NAME: INIT
* DESCRIPTION: Initialize the channel to run the CRANK function.
**************************************************************************/
_eTPU_thread CRANK::INIT(_eTPU_matches_disabled)
{
    /* Stop the channel */
    /* Disable event handling */
    channel.MTD = MTD_DISABLE;
    /* Disable match detection */
    channel.MRLE = MRLE_DISABLE;
    /* Reset all latches */
    channel.TDL = TDL_CLEAR;
    channel.LSR = LSR_CLEAR;
    channel.MRLA = MRL_CLEAR;
    channel.MRLB = MRL_CLEAR;

    /* Initialize the channel */
    /* Set channel mode: mach2 ordered single transition */
    channel.PDCM = PDCM_M2_O_ST;
    /* Time base selection */
    channel.TBSA = TBS_M1C1GE;  /* capture time to erta*/
    channel.TBSB = TBS_M1C2GE;  /* capture angle to ertb */
    /* Input pin action control */
    channel.IPACA = IPAC_FALLING;
    channel.IPACB = IPAC_NO_DETECT;
    if (cc.FM0 == CRANK_FM0_USE_TRANS_RISING)
    {
        channel.IPACA = IPAC_RISING;
    }
    /* Output pin action control */
    channel.OPACA = OPAC_NO_CHANGE;
    channel.OPACB = OPAC_NO_CHANGE;

    /* Channel flags */
    channel.FLAG0 = CRANK_FLAG0_GAP;
    if (teeth_in_gap == 0)
    {
        channel.FLAG0 = CRANK_FLAG0_ADDITIONAL_TOOTH;
    }
    channel.FLAG1 = CRANK_FLAG1_NORMAL_MODE;

    /* Default values */
    trr = 0xffffff;
    tpr = 0;
    if ((int24_t)tcr2 > 0)	tcr2 = 0;
    eng_pos_state = ENG_POS_SEEK;
    eng_cycle_tcr2_start = eng_cycle_tcr2_ticks;
    state = CRANK_SEEK;

    /* Schedule Match A to open window */
    erta = tcr1 + 1; /* the +1 means that the window won't open until the
                        timebase has been started */
    channel.ERWA = ERW_WRITE_ERT_TO_MATCH;

    /* Enable event handling */
    channel.MTD = MTD_ENABLE;
}

/**************************************************************************
* THREAD NAME: ANGLE_ADJUST
* DESCRIPTION: Update TCR2 value, set ENG_POS_FULL_SYNC.
**************************************************************************/
_eTPU_thread CRANK::ANGLE_ADJUST(_eTPU_matches_disabled)
{
    tcr2 += tcr2_adjustment;
#ifdef ERRATA_2477
    err2477_tcr2_target += tcr2_adjustment;
#endif
    /* set global eng_pos state */
    eng_pos_state = ENG_POS_FULL_SYNC;

    /* signal other functions that crank has reached full sync */
    Link4(link_1);
    Link4(link_2);
    Link4(link_3);
    Link4(link_4);
}

/**************************************************************************
* THREAD NAME: CRANK_WITH_GAP
* DESCRIPTION: A transition or a timeout, handling a crank wheel with gap.
**************************************************************************/
_eTPU_thread CRANK::CRANK_WITH_GAP(_eTPU_matches_enabled)
{
    uint24_t   tooth_period;
    uint24_t   half_window_width;
    uint8_t  tmp;

    if (cc.TDLA == 1)
    {
        /* A tooth transition detected */
        switch (state)
        {
        case CRANK_SEEK:
            /**************************************************************
            * STATE: T0 - SEEK
            * DESCRIPTION:
            *   First transition after INIT was detected.
            *   Wait for blank_time without detecting transitions.
            **************************************************************/
            channel.TDL = TDL_CLEAR;
            tcr2 = 0;
            /* set_state */
            state = CRANK_BLANK_TIME;
            /* do not detect transitions */
            channel.IPACA = IPAC_NO_DETECT;
            /* keep window opened, close window after blank_time */
            WindowCloseAt_NoReturn(erta + blank_time);
            break;

        case CRANK_BLANK_TEETH:
            /**************************************************************
            * STATE: T2 - BLANK_TEETH
            * DESCRIPTION:
            *   Count down blank_teeth without tooth period measurement.
            **************************************************************/
            channel.TDL = TDL_CLEAR;
            tcr2 = 0;
            /* Count down blank_teeth */
            if (--blank_teeth <= 0)
            {
                /* set_state */
                state = CRANK_FIRST_TRANS;
            }
            break;

        case CRANK_FIRST_TRANS:
            /**************************************************************
            * STATE: T3 - FIRST_TRANS
            * DESCRIPTION:
            *   First transition after blank_teeth was detected.
            *   Record transition time.
            *   Next transition is expected within first_tooth_timeout.
            **************************************************************/
            channel.TDL = TDL_CLEAR;
            tcr2 = 0;
            /* set_state */
            state = CRANK_SECOND_TRANS;
            /* record last_tooth_tcr1_time */
            last_tooth_tcr1_time = erta;
            /* keep window opened, close window after first_tooth_timeout */
            WindowCloseAt_NoReturn(erta + first_tooth_timeout);
            break;

        case CRANK_SECOND_TRANS:
            /**************************************************************
            * STATE: T4 - SECOND_TRANS
            * DESCRIPTION:
            *   Second transition after blank_teeth was detected.
            *   Calculate tooth period and record transition time.
            *   Next transition is expected within a long timeout over
            *   a possible gap.
            **************************************************************/
            channel.TDL = TDL_CLEAR;
            tcr2 = 0;
            /* set_state */
            state = CRANK_TEST_POSSIBLE_GAP;
            /* record last_tooth_period and last_tooth_tcr1_time */
            tooth_period = erta - last_tooth_tcr1_time;
            last_tooth_tcr1_time = erta;
            last_tooth_period = tooth_period;
            /* keep window opened, have to assume gap is next - close window after
               a long timeout over a possible gap */
            half_window_width = muliur(tooth_period, win_ratio_across_gap);
            WindowCloseAt_NoReturn(erta +
                (tooth_period * (teeth_in_gap + 1U)) + half_window_width);
            break;

        case CRANK_TEST_POSSIBLE_GAP:
            /**************************************************************
            * STATE: T5 - TEST_POSSIBLE_GAP
            * DESCRIPTION:
            *   Transition detected, no synchronization yet.
            *   Calculate tooth period and record transition time.
            *   Test for a possible gap (AB of ABA test).
            *   If possible gap found, expect next transition in window
            *     across gap.
            *   Else, expect next transition within a long timeout over
            *     a possible gap.
            **************************************************************/
            // channel.TDL = TDL_CLEAR; - ONLY CLEAR TDL after the next window is set
            tcr2 = 0;
            /* calc tooth_period and record last_tooth_tcr1_time */
            tooth_period = erta - last_tooth_tcr1_time;
            last_tooth_tcr1_time = erta;
            /* test for a possible gap (AB test) */
            if (muliur(tooth_period, gap_ratio)
               > last_tooth_period)
            { /* a possible gap found */
                /* record last_tooth_period */
                last_tooth_period = tooth_period;
                /* calculate an average tooth_period within the gap */
                last_tooth_period_norm = tooth_period / (teeth_in_gap + 1U);
                /* set state */
                state = CRANK_VERIFY_GAP;
                /* open and close window using win_ratio_after_gap */
                Window_NoReturn(win_ratio_after_gap,
                    last_tooth_period_norm);
            }
            else
            { /* gap not found */
                /* record last_tooth_period */
                last_tooth_period = tooth_period;
                last_tooth_period_norm = tooth_period;
                /* open and close window, have to assume the gap is next */
                WindowAcrossGap_NoReturn(tooth_period);
            }
            break;

        case CRANK_VERIFY_GAP:
            /**************************************************************
            * STATE: T6 - VERIFY_GAP
            * DESCRIPTION:
            *   Transition detected in window across gap, first sync.
            *   Calculate tooth period and record transition time.
            *   Verify a possible gap (BA of ABA test).
            *   If gap verified, this is the second tooth after gap,
            *     start TCR2, set ENG_POS_FIRST_HALF_SYNC and IRQ,
            *     reset Cam log, expect next transition in normal window.
            *   Else, test possible gap again, expect next transition
            *     within a long timeout over a possible gap.
            **************************************************************/
            // channel.TDL = TDL_CLEAR; - ONLY CLEAR TDL after the next window is set
            /* calc tooth_period and record last_tooth_tcr1_time */
            tooth_period = erta - last_tooth_tcr1_time;
            last_tooth_tcr1_time = erta;
            /* verify a possible gap (BA portion of the ABA test) */
            if (muliur(last_tooth_period, gap_ratio)
               > tooth_period)
            { /* gap verified */
                /* set states */
                state = CRANK_COUNTING;
                /* record last_tooth_period */
                last_tooth_period = tooth_period;
                last_tooth_period_norm = tooth_period;
                /* set tooth counters - second tooth after gap */
                tooth_counter_cycle = 2;
                //tooth_counter_gap = 2;
                tooth_counter_gap = 1;
                tooth_counter_gap++;
                /* set TRR and TICKS */
                tpr_str.TICKS = (uint16_t)tcr2_ticks_per_tooth - 1;
                Set_TRR(tooth_period);
                tcr2 = tcr2_ticks_per_tooth;
#ifdef ERRATA_2477
                err2477_tcr2_target = tcr2_ticks_per_tooth;
#endif
                /* set global eng_pos state and channel interrupt */
                eng_pos_state = ENG_POS_FIRST_HALF_SYNC;
                channel.CIRC = CIRC_INT_FROM_SERVICED;
                /* reset Cam log */
                Link4(link_cam);
                /* open and close window using win_ratio_normal */
                Window_NoReturn(win_ratio_normal, tooth_period);
            }
            else
            { /* gap not verified */
                /* set state */
                state = CRANK_TEST_POSSIBLE_GAP;
                /* record last_tooth_period */
                last_tooth_period = tooth_period;
                last_tooth_period_norm = tooth_period;
                /* open and close window, have to assume the gap is next */
                WindowAcrossGap_NoReturn(tooth_period);
            }
            break;

        case CRANK_COUNTING_TIMEOUT:
            /**************************************************************
            * STATE: T8 - COUNTING_TIMEOUT
            * DESCRIPTION:
            *   Transition detected within window after a single timeout.
            *   Recover and continue normal counting.
            **************************************************************/
            /* recover from counting timeout */
            /* set state */
            state = CRANK_COUNTING;
            /* continue to normal processing at CRANK_COUNTING */

        case CRANK_COUNTING:
            /**************************************************************
            * STATE: T7 - COUNTING
            * DESCRIPTION:
            *   Transition detected in normal window.
            *   Calculate tooth period and record transition time.
            *   Increment tooth counters.
            *   Check if the next tooth is the last before gap.
            *   Adjust TCR2 rate.
            *   Expect next transition in normal window.
            **************************************************************/
            // channel.TDL = TDL_CLEAR; - ONLY CLEAR TDL after the next window is set
            /* record last_tooth_period and last_tooth_tcr1_time */
            tooth_period = erta - last_tooth_tcr1_time;
            last_tooth_tcr1_time = erta;
            last_tooth_period = tooth_period;
            last_tooth_period_norm = tooth_period;
            /* increment tooth counters */
            tooth_counter_gap++;
            tooth_counter_cycle++;
            /* test if before the gap */
            if (tooth_counter_gap == teeth_till_gap - 1)
            {
                /* there is one more teeth till the gap */
                state = CRANK_TOOTH_BEFORE_GAP;
            }
            /* set TRR */
            Set_TRR(tooth_period);
#ifdef ERRATA_2477
            err2477_tcr2_target += tcr2_ticks_per_tooth;
#endif
            /* log tooth period */
            ToothArray_Log(tooth_period);
            /* open and close window using win_ratio_normal */
            Window_NoReturn(win_ratio_normal, tooth_period);
            break;

        case CRANK_TOOTH_BEFORE_GAP:
            /**************************************************************
            * STATE: T9 - TOOTH_BEFORE_GAP
            * DESCRIPTION:
            *   Transition detected in normal window, gap expected next.
            *   Calculate tooth period and record transition time.
            *   Increment tooth counters.
            *   Adjust TCR2 rate.
            *   Expect next transition within window across the gap.
            **************************************************************/
            // channel.TDL = TDL_CLEAR; - ONLY CLEAR TDL after the next window is set
            /* record last_tooth_period and last_tooth_tcr1_time */
            tooth_period = erta - last_tooth_tcr1_time;
            last_tooth_tcr1_time = erta;
            last_tooth_period = tooth_period;
            last_tooth_period_norm = tooth_period;
            /* increment tooth counters */
            tooth_counter_gap++;
            tooth_counter_cycle++;
            /* set TRR */
            Set_TRR(tooth_period);
            /* log tooth period */
            ToothArray_Log(tooth_period);
#ifdef ERRATA_2477
            /* set state */
            state = CRANK_TOOTH_BEFORE_GAP_NOT_HRM;
            /* workaround for errata 2477
             * a match is scheduled in angle so that the Angle Hardware is
             * not in HRM */
            channel.TBSB = TBS_M2C2GE;
            /* open window immediately to enable match B
               and schedule the match B for when the EAC will be out of HRM */
            channel.TDL = TDL_CLEAR;
            CRANK_WindowCloseAt_NoReturn(err2477_tcr2_target);
#else
            /* set state */
            state = CRANK_TOOTH_AFTER_GAP;
            /* write MISSCNT */
            tpr |= misscnt_mask;
            /* open and close window using win_ratio_normal
               and win_ratio_across_gap */
            WindowAcrossGap_NoReturn(tooth_period);
#endif
            break;

        case CRANK_TOOTH_AFTER_GAP:
            /**************************************************************
            * STATE: T11 - TOOTH_AFTER_GAP
            * DESCRIPTION:
            *   Transition detected in window across gap.
            *   Calculate tooth period and record transition time.
            *   Verify the gap (AB of ABA test).
            *   If gap verified, adjust TCR2 rate and tooth counters,
            *     sync-cycle or engine-cycle is finished:
            *     In ENG_POS_FIRST_HALF_SYNC,
            *       ask CPU to decode the Cam log,
            *       reset TCR2, reset tooth_counter_cycle,
            *       set ENG_POS_PRE_FULL_SYNC and IRQ.
            *     In ENG_POS_PRE_FULL_SYNC, there was no response from CPU,
            *       reset Cam log, reset TCR2, reset tooth_counter_cycle,
            *       set ENG_POS_FIRST_HALF_SYNC and IRQ.
            *     In ENG_POS_FULL_SYNC,
            *       reset Cam log, reset tooth_counter_cycle,
            *       set IRQ (once per cycle in full sync)
            *       increment eng_cycle_tcr2_start by one cycle
            *     Expect next transition in window after gap.
            *   Else, gap not verified, set CRANK_ERR_TOOTH_IN_GAP,
            *     set ENG_POS_SEEK and IRQ, signal output functions and
            *     restart searching for the gap
            **************************************************************/
            /* calc tooth_period and record last_tooth_tcr1_time */
            tooth_period = erta - last_tooth_tcr1_time;
            last_tooth_tcr1_time = erta;
            /* verify gap (AB test) */
            if (muliur(tooth_period, gap_ratio)
               > last_tooth_period)
            { /* gap verified */
                // channel.TDL = TDL_CLEAR; - ONLY CLEAR TDL after the next window is set
                /* record last_tooth_period */
                last_tooth_period = tooth_period;
                /* calculate an average tooth_period within the gap */
                last_tooth_period_norm = tooth_period / (teeth_in_gap + 1U);
                /* set TRR */
                Set_TRR(last_tooth_period_norm);
                /* set state - if the second tooth after the gap times out then
                   the state machine will revert to FIRST_TRANS */
                state = CRANK_COUNTING_TIMEOUT;
                /* set tooth counters - first tooth after gap */
                tooth_counter_gap = 1;
                tmp = tooth_counter_cycle + teeth_in_gap;
                while (++tooth_counter_cycle <= tmp)
                {
                    /* log average tooth period for all teeth in gap */
                    ToothArray_Log(last_tooth_period_norm);
                }
#ifdef ERRATA_2477
                err2477_tcr2_target +=
                    (teeth_in_gap + 1U) * tcr2_ticks_per_tooth;
#endif
                /* when the sync-cycle or engine-cycle is finished */
                switch (eng_pos_state)
                {
                case ENG_POS_FIRST_HALF_SYNC:
                    /* if the sync cycle is finished */
                    if (tooth_counter_cycle >= teeth_per_sync)
                    { /* It is time to ask the CPU to decode which half-cycle it was */
                        /* set global eng_pos state and channel interrupt */
                        eng_pos_state = ENG_POS_PRE_FULL_SYNC;
                        channel.CIRC = CIRC_INT_FROM_SERVICED;
                        ToothTcr2Sync_NoReturn();
                    }
                    break;
                case ENG_POS_PRE_FULL_SYNC:
                    /* if the sync cycle is finished */
                    if (tooth_counter_cycle >= teeth_per_sync)
                    { /* no answer from the CPU has been received during the whole
                         sync cycle */
                         /* set global eng_pos state and channel interrupt */
                        eng_pos_state = ENG_POS_FIRST_HALF_SYNC;
                        channel.CIRC = CIRC_INT_FROM_SERVICED;
                        /* reset Cam log */
                        Link4(link_cam);
                        ToothTcr2Sync_NoReturn();
                    }
                    break;
                case ENG_POS_FULL_SYNC:
                    /* if the engine cycle is finished */
                    if (tooth_counter_cycle >= teeth_per_cycle)
                    {
                        /* set channel interrupt - once per cycle in full-sync */
#if defined(__TARGET_ETPU2__) || defined(__ETPU2__)
                        channel.CIRC = CIRC_BOTH_FROM_SERVICED;  /* on eTPU2, set also DMA request */
#else
                        channel.CIRC = CIRC_INT_FROM_SERVICED;
#endif
                        /* reset Cam log */
                        Link4(link_cam);
                        /* reset tooth_counter_cycle */
                        tooth_counter_cycle = 1;
                        /* collect diagnostic data */
                        tcr2_error_at_cycle_start = tcr2 - eng_cycle_tcr2_start - tcr2_adjustment;
                        /* increment eng_cycle_tcr2_start by one cycle */
                        eng_cycle_tcr2_start += eng_cycle_tcr2_ticks;
                    }
                    break;
                }
                /* log tooth period (after possible tooth_counter_cycle reset) */
                ToothArray_Log(last_tooth_period_norm);
                /* open and close window using win_ratio_after_gap */
                Window_NoReturn(win_ratio_after_gap,
                    last_tooth_period_norm);
            }
            else
            { /* gap not verified - an unexpected tooth in gap */
                /* record last_tooth_period */
                last_tooth_period = tooth_period;
                last_tooth_period_norm = tooth_period;
                /* set error */
                error |= CRANK_ERR_TOOTH_IN_GAP;
                /* restart searching for the gap */
                channel.TDL = TDL_CLEAR;
                Stall_NoReturn();
            }
            break;

        case CRANK_BLANK_TIME:
        case CRANK_TOOTH_BEFORE_GAP_NOT_HRM:
            /**************************************************************
            * STATE: T1, T10 - BLANK_TIME, TOOTH_BEFORE_GAP_NOT_HRM
            * DESCRIPTION:
            *   Transition detection should never happen in this state.
            *   Set CRANK_ERR_INVALID_TRANS.
            **************************************************************/
            channel.TDL = TDL_CLEAR;
            error |= CRANK_ERR_INVALID_TRANS;
            break;

        default:
            channel.TDL = TDL_CLEAR;
            error |= CRANK_ERR_INTERNAL;
            break;
        }
    }
    else /* cc.MRLB == 1 */
    {
        /* A timeout detected */
        channel.MRLB = MRL_CLEAR;
        switch (state)
        {
        case CRANK_BLANK_TIME:
            /**************************************************************
            * STATE: M1 - BLANK_TIME
            * DESCRIPTION:
            *   Blank_time after the first transition has passed.
            *   Start to detect transitions.
            **************************************************************/
            tcr2 = 0;
            if (cc.FM0 == CRANK_FM0_USE_TRANS_RISING)
            {
                channel.IPACA = IPAC_RISING;
            }
            else
            {
                channel.IPACA = IPAC_FALLING;
            }
            /* open window immediately, do not close it */
            erta = tcr1;
            channel.MRLA = MRL_CLEAR;
            channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
            /* set next state */
            state = CRANK_BLANK_TEETH;
            if (blank_teeth == 0)
            {
                state = CRANK_FIRST_TRANS;
            }
            break;

        case CRANK_SECOND_TRANS:      /* first_tooth_timeout */
        case CRANK_TEST_POSSIBLE_GAP: /* long timeout over a possible gap */
        case CRANK_VERIFY_GAP:        /* win_ratio_after_gap */
            /**************************************************************
            * STATE: M4, M5, M6 - SECOND_TRANS, TEST_POSSIBLE_GAP,
            *                     VERIFY_GAP
            * DESCRIPTION:
            *   Transition not detected in a window, timeout happened
            *   while gap is not verified.
            *   Set CRANK_ERR_TIMEOUT.
            *   Open the acceptance window immediately and do not close it.
            **************************************************************/
            /* timeout happened while gap is not verified */
            tcr2 = 0;
            error |= CRANK_ERR_TIMEOUT;
            state = CRANK_FIRST_TRANS;
            /* open the acceptance window immediately and do not close it */
            erta = tcr1;
            channel.MRLA = MRL_CLEAR;
            channel.MRLB = MRL_CLEAR;
            channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
            break;

        case CRANK_COUNTING:          /* win_ratio_normal */
            /**************************************************************
            * STATE: M7 - COUNTING
            * DESCRIPTION:
            *   Transition not detected in normal window, this is the first
            *   timeout, there has not been one immediately before.
            *   Set CRANK_ERR_TIMEOUT.
            *   Insert physical tooth, increment tooth counters.
            *   Expect next transition in window after timeout.
            **************************************************************/
            error |= CRANK_ERR_TIMEOUT;
            state = CRANK_COUNTING_TIMEOUT;
            /* approximate when the missed tooth should have happened */
            tooth_period = last_tooth_period;
            erta = last_tooth_tcr1_time + tooth_period;
            last_tooth_tcr1_time = erta;
            /* set IPH because one tooth was missing */
            tpr_str.IPH = 1;
#ifdef ERRATA_2477
            err2477_tcr2_target += tcr2_ticks_per_tooth;
#endif
            /* increment tooth counters */
            tooth_counter_gap++;
            tooth_counter_cycle++;
            /* test if before the gap */
            if (tooth_counter_gap == teeth_till_gap - 1)
            {
                /* there is one more teeth till the gap */
                state = CRANK_TOOTH_BEFORE_GAP;
            }
            /* log tooth period */
            ToothArray_Log(tooth_period);
            /* open and close window using win_ratio_after_timeout */
            Window_NoReturn(win_ratio_after_timeout, tooth_period);
            break;

        case CRANK_COUNTING_TIMEOUT:  /* win_ratio_after_timeout */
            /**************************************************************
            * STATE: M8 - COUNTING_TIMEOUT
            * DESCRIPTION:
            *   Transition not detected in window after timeout, this is
            *   the second timeout, there has been one immediately before.
            *   Set ENG_POS_SEEK and IRQ, signal output functions and
            *   restart searching for the gap.
            **************************************************************/
            /* restart searching for the gap */
            Stall_NoReturn();
            break;

        case CRANK_TOOTH_BEFORE_GAP:  /* win_ratio_normal */
            /**************************************************************
            * STATE: M9 - TOOTH_BEFORE_GAP
            * DESCRIPTION:
            *   Transition not detected in normal window before gap.
            *   Set CRANK_ERR_TIMEOUT_BEFORE_GAP.
            *   Set ENG_POS_SEEK and IRQ, signal output functions and
            *   restart searching for the gap.
            **************************************************************/
            /* set error */
            error |= CRANK_ERR_TIMEOUT_BEFORE_GAP;
            /* restart searching for the gap */
            Stall_NoReturn();
            break;

        case CRANK_TOOTH_BEFORE_GAP_NOT_HRM:
            /**************************************************************
            * STATE: M10 - TOOTH_BEFORE_GAP_NOT_HRM
            * DESCRIPTION:
            *   On devices with ERRATA_2477, the MISSCNT must be written
            *   when EAC is out of high rate mode, which is now.
            *   Expect next transition within window across the gap.
            **************************************************************/
            /* write MISSCNT when not in High Rate Mode */
            /* set state */
            state = CRANK_TOOTH_AFTER_GAP;
            /* write MISSCNT */
            tpr |= misscnt_mask;
            /* open and close window using win_ratio_normal
               and win_ratio_across_gap */
            channel.TBSB = TBS_M1C2GE;
            erta = last_tooth_tcr1_time;
            WindowAcrossGap_NoReturn(last_tooth_period);
            break;

        case CRANK_TOOTH_AFTER_GAP:   /* long timeout over a possible gap */
            /**************************************************************
            * STATE: M11 - TOOTH_AFTER_GAP
            * DESCRIPTION:
            *   Transition not detected in window across gap.
            *   Set CRANK_ERR_TIMEOUT_AFTER_GAP.
            *   Set ENG_POS_SEEK and IRQ, signal output functions and
            *   restart searching for the gap.
            **************************************************************/
            /* set error */
            error |= CRANK_ERR_TIMEOUT_AFTER_GAP;
            /* restart searching for the gap */
            Stall_NoReturn();
            break;

        case CRANK_SEEK:
        case CRANK_BLANK_TEETH:
        case CRANK_FIRST_TRANS:
            /**************************************************************
            * STATE: M0, M2, M3 - SEEK, BLANK_TEETH, FIRST_TRANS
            * DESCRIPTION:
            *   Match detection should never happen in this state.
            *   Set CRANK_ERR_INVALID_MATCH.
            **************************************************************/
            error |= CRANK_ERR_INVALID_MATCH;
            break;

        default:
            error |= CRANK_ERR_INTERNAL;
            break;
        }
    }
}

/**************************************************************************
* THREAD NAME: CRANK_WITH_ADDITIONAL_TOOTH
* DESCRIPTION: A transition or a timeout, handling a crank wheel with an
*              additional tooth.
**************************************************************************/
_eTPU_thread CRANK::CRANK_WITH_ADDITIONAL_TOOTH(_eTPU_matches_enabled)
{
    uint24_t   tooth_period;
    uint24_t   half_window_width;

    if (cc.TDLA == 1)
    {
        /* A tooth transition detected */
        switch (state)
        {
        case CRANK_SEEK:
            /**************************************************************
            * STATE: T0 - SEEK
            * DESCRIPTION:
            *   First transition after INIT was detected.
            *   Wait for blank_time without detecting transitions.
            **************************************************************/
            channel.TDL = TDL_CLEAR;
            tcr2 = 0;
            /* set_state */
            state = CRANK_BLANK_TIME;
            /* do not detect transitions */
            channel.IPACA = IPAC_NO_DETECT;
            /* keep window opened, close window after blank_time */
            WindowCloseAt_NoReturn(erta + blank_time);
            break;

        case CRANK_BLANK_TEETH:
            /**************************************************************
            * STATE: T2 - BLANK_TEETH
            * DESCRIPTION:
            *   Downcount blank_teeth without tooth period measurement.
            **************************************************************/
            channel.TDL = TDL_CLEAR;
            tcr2 = 0;
            /* downcount blank_teeth */
            if (--blank_teeth <= 0)
            {
                /* set_state */
                state = CRANK_FIRST_TRANS;
            }
            break;

        case CRANK_FIRST_TRANS:
            /**************************************************************
            * STATE: T3 - FIRST_TRANS
            * DESCRIPTION:
            *   First transition after blank_teeth was detected.
            *   Record transition time.
            *   Next transition is expected within first_tooth_timeout.
            **************************************************************/
            channel.TDL = TDL_CLEAR;
            tcr2 = 0;
            /* set_state */
            state = CRANK_SECOND_TRANS;
            /* record last_tooth_tcr1_time */
            last_tooth_tcr1_time = erta;
            /* keep window opened, close window after first_tooth_timeout */
            WindowCloseAt_NoReturn(erta + first_tooth_timeout);
            break;

        case CRANK_SECOND_TRANS:
            /**************************************************************
            * STATE: T4A - SECOND_TRANS
            * DESCRIPTION:
            *   Second transition after blank_teeth was detected.
            *   Calculate tooth period and record transition time.
            *   Next transition is expected in normal window or earlier.
            **************************************************************/
            channel.TDL = TDL_CLEAR;
            tcr2 = 0;
            /* set_state */
            state = CRANK_TEST_POSSIBLE_GAP;
            /* record last_tooth_period and last_tooth_tcr1_time */
            tooth_period = erta - last_tooth_tcr1_time;
            last_tooth_tcr1_time = erta;
            last_tooth_period = tooth_period;
            /* keep window opened, close using win_ratio_normal */
            WindowClose_NoReturn(win_ratio_normal,
                tooth_period);
            break;

        case CRANK_TEST_POSSIBLE_GAP:
            /**************************************************************
            * STATE: T5A - TEST_POSSIBLE_GAP
            * DESCRIPTION:
            *   Transition detected, no synchronization yet.
            *   Calculate tooth period and record transition time.
            *   Test for a possible gap (AB of ABA test).
            *   If possible gap found, expect next transition in normal
            *     window from the previous tooth (not the additional one).
            *   Else, expect next transition in normal window or earlier.
            **************************************************************/
            channel.TDL = TDL_CLEAR;
            tcr2 = 0;
            /* calc tooth_period */
            tooth_period = erta - last_tooth_tcr1_time;
            /* test for a possible additional tooth (AB test) */
            if (muliur(last_tooth_period, gap_ratio)
               > tooth_period)
            { /* a possible additional tooth found */
                /* record additional_tooth_period */
                additional_tooth_period = tooth_period;
                /* set state */
                state = CRANK_VERIFY_GAP;
                /* open and close window using win_ratio_normal from
                   the previous tooth (not the additional one) */
                half_window_width = muliur(last_tooth_period, win_ratio_normal);
                WindowCloseAt_NoReturn(last_tooth_tcr1_time
                    + last_tooth_period + half_window_width);
            }
            else
            { /* gap not found */
                /* record last_tooth_tcr1_time */
                last_tooth_tcr1_time = erta;
                /* record last_tooth_period */
                last_tooth_period = tooth_period;
                last_tooth_period_norm = tooth_period;
                /* keep window opened, close using win_ratio_normal */
                WindowClose_NoReturn(win_ratio_normal,
                    tooth_period);
            }
            break;

        case CRANK_VERIFY_GAP:
            /**************************************************************
            * STATE: T6A - VERIFY_GAP
            * DESCRIPTION:
            *   Transition detected in window, after an additional tooth.
            *   Calculate tooth period and record transition time.
            *   Verify a possible gap (BA of ABA test).
            *   If gap verified, this is the first tooth after gap,
            *     start TCR2, set ENG_POS_FIRST_HALF_SYNC and IRQ,
            *     reset Cam log, expect next transition in normal window.
            *   Else, test possible gap again, expect next transition
            *     in normal window or earlier.
            **************************************************************/
            /* calc tooth_period and record last_tooth_tcr1_time */
            tooth_period = erta - last_tooth_tcr1_time;
            last_tooth_tcr1_time = erta;
            /* verify a possible additional tooth (BA portion of the ABA test) */
            if (muliur(tooth_period, gap_ratio)
               > additional_tooth_period)
            { /* additional tooth verified */
                // channel.TDL = TDL_CLEAR; - ONLY CLEAR TDL after the next window is set
                /* set states */
                state = CRANK_COUNTING;
                /* record last_tooth_period */
                last_tooth_period = tooth_period;
                last_tooth_period_norm = tooth_period;
                /* set tooth counters - the first tooth after gap */
                tooth_counter_gap = 1;
                tooth_counter_cycle = 1;
                /* set TRR and TICKS */
                tpr_str.TICKS = (uint16_t)tcr2_ticks_per_tooth - 1;
                Set_TRR(tooth_period);
                tcr2 = 0;
                /* set global eng_pos state and channel interrupt */
                eng_pos_state = ENG_POS_FIRST_HALF_SYNC;
                channel.CIRC = CIRC_INT_FROM_SERVICED;
                /* reset Cam log */
                Link4(link_cam);
                /* open and close window using win_ratio_normal */
                Window_NoReturn(win_ratio_normal, tooth_period);
            }
            else
            { /* additional tooth not verified */
                /* set state */
                channel.TDL = TDL_CLEAR;
                state = CRANK_TEST_POSSIBLE_GAP;
                /* correct tooth_period - it was not the additional tooth */
                tooth_period -= additional_tooth_period;
                /* record last_tooth_period */
                last_tooth_period = tooth_period;
                last_tooth_period_norm = tooth_period;
                /* keep window opened, close using win_ratio_normal */
                WindowClose_NoReturn(win_ratio_normal,
                    tooth_period);
            }
            break;

        case CRANK_COUNTING_TIMEOUT:
            /**************************************************************
            * STATE: T8 - COUNTING_TIMEOUT
            * DESCRIPTION:
            *   Transition detected within window after a single timeout.
            *   Recover and continue normal counting.
            **************************************************************/
            /* recover from counting timeout */
            /* set state */
            state = CRANK_COUNTING;
            /* continue to normal processing at CRANK_COUNTING */

        case CRANK_COUNTING:
            /**************************************************************
            * STATE: T7 - COUNTING
            * DESCRIPTION:
            *   Transition detected in normal window.
            *   Calculate tooth period and record transition time.
            *   Increment tooth counters.
            *   Check if the next tooth is the last before gap.
            *   Adjust TCR2 rate.
            *   Expect next transition in normal window.
            **************************************************************/
            // channel.TDL = TDL_CLEAR; - ONLY CLEAR TDL after the next window is set
            /* record last_tooth_period and last_tooth_tcr1_time */
            tooth_period = erta - last_tooth_tcr1_time;
            last_tooth_tcr1_time = erta;
            last_tooth_period = tooth_period;
            last_tooth_period_norm = tooth_period;
            /* increment tooth counters */
            tooth_counter_gap++;
            tooth_counter_cycle++;
            /* test if before the gap */
            if (tooth_counter_gap == teeth_till_gap - 1)
            {
                /* there is one more teeth till the gap */
                state = CRANK_TOOTH_BEFORE_GAP;
            }
            /* set TRR */
            Set_TRR(tooth_period);
            /* log tooth period */
            ToothArray_Log(tooth_period);
            /* open and close window using win_ratio_normal */
            Window_NoReturn(win_ratio_normal, tooth_period);
            break;

        case CRANK_TOOTH_BEFORE_GAP:
            /**************************************************************
            * STATE: T9A - TOOTH_BEFORE_GAP
            * DESCRIPTION:
            *   Transition detected in normal window, gap expected next.
            *   Calculate tooth period and record transition time.
            *   Increment tooth counters.
            *   Adjust TCR2 rate.
            *   Expect next transition in normal window or earlier.
            **************************************************************/
            channel.TDL = TDL_CLEAR;
            /* record last_tooth_period and last_tooth_tcr1_time */
            tooth_period = erta - last_tooth_tcr1_time;
            last_tooth_tcr1_time = erta;
            last_tooth_period = tooth_period;
            last_tooth_period_norm = tooth_period;
            /* increment tooth counters */
            tooth_counter_gap++;
            tooth_counter_cycle++;
            /* Set TICKS */
            tpr_str.TICKS = (uint16_t)tcr2_ticks_per_add_tooth - 1;
            /* set TRR */
            Set_TRR(tooth_period);
            /* log tooth period */
            ToothArray_Log(tooth_period);
            /* set state */
            state = CRANK_ADDITIONAL_TOOTH;
            /* keep window opened, close using win_ratio_normal */
            WindowClose_NoReturn(win_ratio_normal,
                tooth_period);
            break;

        case CRANK_ADDITIONAL_TOOTH:
            /**************************************************************
            * STATE: T10A - CRANK_ADDITIONAL_TOOTH
            * DESCRIPTION:
            *   Transition detected in additional-tooth window.
            *   Calculate additional tooth period.
            *   Verify the additional tooth (AB of ABA test).
            *   If tooth verified, expect next transition within normal
            *     window from the last tooth (not the additional tooth).
            *   Else, tooth not verified, set CRANK_ERR_ADD_TOOTH_NOT_FOUND,
            *     set ENG_POS_SEEK and IRQ, signal output functions and
            *     restart searching for the gap
            **************************************************************/
            channel.TDL = TDL_CLEAR;
            /* calc tooth_period */
            tooth_period = erta - last_tooth_tcr1_time;
            /* Set TICKS */
            tpr_str.TICKS = (uint16_t)(tcr2_ticks_per_tooth - tcr2_ticks_per_add_tooth) - 1;
            /* verify the additional tooth (AB test) */
            if (muliur(last_tooth_period, gap_ratio)
               > tooth_period)
            { /* additional tooth not verified */
                /* record additional_tooth_period */
                additional_tooth_period = tooth_period;
                /* set state */
                state = CRANK_TOOTH_AFTER_GAP;
                /* open and close window using win_ratio_normal from
                   the previous tooth (not the additional one) */
                half_window_width = muliur(last_tooth_period, win_ratio_normal);
                WindowCloseAt_NoReturn(last_tooth_tcr1_time
                    + last_tooth_period + half_window_width);
            }
            else
            { /* additional tooth not verified */
                /* record last_tooth_tcr1_time */
                last_tooth_tcr1_time = erta;
                /* record last_tooth_period */
                last_tooth_period = tooth_period;
                last_tooth_period_norm = tooth_period;
                /* set error */
                error |= CRANK_ERR_ADD_TOOTH_NOT_FOUND;
                /* restart searching for the gap */
                Stall_NoReturn();
            }
            break;

        case CRANK_TOOTH_AFTER_GAP:
            /**************************************************************
            * STATE: T11A - TOOTH_AFTER_GAP
            * DESCRIPTION:
            *   Transition detected in normal window from the last tooth.
            *   Calculate tooth period and record transition time.
            *   Verify the additional tooth (BA of ABA test).
            *   If additional tooth verified, adjust TCR2 rate and tooth
            *     counters, sync-cycle or engine-cycle is finished:
            *     In ENG_POS_FIRST_HALF_SYNC,
            *       ask CPU to decode the Cam log,
            *       reset TCR2, reset tooth_counter_cycle,
            *       set ENG_POS_PRE_FULL_SYNC and IRQ.
            *     In ENG_POS_PRE_FULL_SYNC, there was no response from CPU,
            *       reset Cam log, reset TCR2, reset tooth_counter_cycle,
            *       set ENG_POS_FIRST_HALF_SYNC and IRQ.
            *     In ENG_POS_FULL_SYNC,
            *       reset Cam log, reset tooth_counter_cycle,
            *       set IRQ (once per cycle in full sync)
            *       increment eng_cycle_tcr2_start by one cycle
            *     Expect next transition in normal window.
            *   Else, additional tooth not verified, set CRANK_ERR_TOOTH_IN_GAP,
            *     set ENG_POS_SEEK and IRQ, signal output functions and
            *     restart searching for the gap
            **************************************************************/
            /* Set TICKS */
            tpr_str.TICKS = (uint16_t)tcr2_ticks_per_tooth - 1;
            /* calc tooth_period and record last_tooth_tcr1_time */
            tooth_period = erta - last_tooth_tcr1_time;
            last_tooth_tcr1_time = erta;
            /* verify a possible additional tooth (BA portion of the ABA test) */
            if (muliur(tooth_period, gap_ratio)
               > additional_tooth_period)
            { /* additional tooth verified */
                // channel.TDL = TDL_CLEAR; - ONLY CLEAR TDL after the next window is set
                /* record last_tooth_period */
                last_tooth_period = tooth_period;
                last_tooth_period_norm = tooth_period;
                /* set TRR */
                Set_TRR(tooth_period);
                /* set state - if the second tooth after the gap times out then
                   the state machine will revert to FIRST_TRANS */
                state = CRANK_COUNTING_TIMEOUT;
                /* set tooth counters - first tooth after gap */
                tooth_counter_gap = 1;
                tooth_counter_cycle++;
                /* log tooth period */
                ToothArray_Log(tooth_period);
                /* when the sync-cycle or engine-cycle is finished */
                switch (eng_pos_state)
                {
                case ENG_POS_FIRST_HALF_SYNC:
                    /* if the sync cycle is finished */
                    if (tooth_counter_cycle >= teeth_per_sync)
                    { /* It is time to ask the CPU to decode which half-cycle it was */
                        /* set global eng_pos state and channel interrupt */
                        eng_pos_state = ENG_POS_PRE_FULL_SYNC;
                        channel.CIRC = CIRC_INT_FROM_SERVICED;
                        ToothTcr2Sync_NoReturn();
                    }
                    break;
                case ENG_POS_PRE_FULL_SYNC:
                    /* if the sync cycle is finished */
                    if (tooth_counter_cycle >= teeth_per_sync)
                    { /* no answer from the CPU has been received during the whole
                         sync cycle */
                         /* set global eng_pos state and channel interrupt */
                        eng_pos_state = ENG_POS_FIRST_HALF_SYNC;
                        channel.CIRC = CIRC_INT_FROM_SERVICED;
                        /* reset Cam log */
                        Link4(link_cam);
                        ToothTcr2Sync_NoReturn();
                    }
                    break;
                case ENG_POS_FULL_SYNC:
                    /* if the engine cycle is finished */
                    if (tooth_counter_cycle >= teeth_per_cycle)
                    {
                        /* set channel interrupt - once per cycle in full-sync */
#if defined(__TARGET_ETPU2__)
                        channel.CIRC = CIRC_BOTH_FROM_SERVICED;  /* on eTPU2, set also DMA request */
#else
                        channel.CIRC = CIRC_INT_FROM_SERVICED;
#endif
                        /* reset Cam log */
                        Link4(link_cam);
                        /* reset tooth_counter_cycle */
                        tooth_counter_cycle = 1;
                        /* collect diagnostic data */
                        tcr2_error_at_cycle_start = tcr2 - eng_cycle_tcr2_start - tcr2_adjustment;
                        /* increment eng_cycle_tcr2_start by one cycle */
                        eng_cycle_tcr2_start += eng_cycle_tcr2_ticks;
                    }
                    break;
                }
                /* open and close window using win_ratio_normal */
                Window_NoReturn(win_ratio_normal, tooth_period);
            }
            else
            { /* additional tooth not verified */
                /* correct tooth_period - it was not the additional tooth */
                tooth_period -= additional_tooth_period;
                /* record last_tooth_period */
                last_tooth_period = tooth_period;
                last_tooth_period_norm = tooth_period;
                /* set error */
                error |= CRANK_ERR_ADD_TOOTH_NOT_FOUND;
                /* restart searching for the gap */
                channel.TDL = TDL_CLEAR;
                Stall_NoReturn();
            }
            break;

        case CRANK_BLANK_TIME:
            /**************************************************************
            * STATE: T1 - BLANK_TIME
            * DESCRIPTION:
            *   Transition detection should never happen in this state.
            *   Set CRANK_ERR_INVALID_TRANS.
            **************************************************************/
            channel.TDL = TDL_CLEAR;
            error |= CRANK_ERR_INVALID_TRANS;
            break;

        default:
            channel.TDL = TDL_CLEAR;
            error |= CRANK_ERR_INTERNAL;
            break;
        }
    }
    else /* cc.MRLB == 1 */
    {
        /* A timeout detected */
        channel.MRLB = MRL_CLEAR;
        switch (state)
        {
        case CRANK_BLANK_TIME:
            /**************************************************************
            * STATE: M1 - BLANK_TIME
            * DESCRIPTION:
            *   Blank_time after the first transition has passed.
            *   Start to detect transitions.
            **************************************************************/
            tcr2 = 0;
            if (cc.FM0 == CRANK_FM0_USE_TRANS_RISING)
            {
                channel.IPACA = IPAC_RISING;
            }
            else
            {
                channel.IPACA = IPAC_FALLING;
            }
            /* open window immediately, do not close it */
            erta = tcr1;
            channel.MRLA = MRL_CLEAR;
            channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
            /* set next state */
            state = CRANK_BLANK_TEETH;
            if (blank_teeth == 0)
            {
                state = CRANK_FIRST_TRANS;
            }
            break;

        case CRANK_SECOND_TRANS:      /* first_tooth_timeout */
        case CRANK_TEST_POSSIBLE_GAP: /* win_ratio_normal */
        case CRANK_VERIFY_GAP:        /* win_ratio_normal */
            /**************************************************************
            * STATE: M4, M5, M6 - SECOND_TRANS, TEST_POSSIBLE_GAP,
            *                     VERIFY_GAP
            * DESCRIPTION:
            *   Transition not detected in a window, timeout happened
            *   while gap is not verified.
            *   Set CRANK_ERR_TIMEOUT.
            *   Open the acceptance window immediately and do not close it.
            **************************************************************/
            /* timeout happened while gap is not verified */
            tcr2 = 0;
            error |= CRANK_ERR_TIMEOUT;
            state = CRANK_FIRST_TRANS;
            /* open the acceptance window immediately and do not close it */
            erta = tcr1;
            channel.MRLA = MRL_CLEAR;
            channel.MRLB = MRL_CLEAR;
            channel.ERWA = ERW_WRITE_ERT_TO_MATCH;
            break;

        case CRANK_COUNTING:          /* win_ratio_normal */
            /**************************************************************
            * STATE: M7 - COUNTING
            * DESCRIPTION:
            *   Transition not detected in normal window, this is the first
            *   timeout, there has not been one immediately before.
            *   Set CRANK_ERR_TIMEOUT.
            *   Insert physical tooth, increment tooth counters.
            *   Expect next transition in window after timeout.
            **************************************************************/
            error |= CRANK_ERR_TIMEOUT;
            state = CRANK_COUNTING_TIMEOUT;
            /* approximate when the missed tooth should have happened */
            tooth_period = last_tooth_period;
            erta = last_tooth_tcr1_time + tooth_period;
            last_tooth_tcr1_time = erta;
            /* set IPH because one tooth was missing */
            tpr_str.IPH = 1;
            /* increment tooth counters */
            tooth_counter_gap++;
            tooth_counter_cycle++;
            /* test if before the gap */
            if (tooth_counter_gap == teeth_till_gap - 1)
            {
                /* there is one more teeth till the gap */
                state = CRANK_TOOTH_BEFORE_GAP;
            }
            /* log tooth period */
            ToothArray_Log(tooth_period);
            /* open and close window using win_ratio_after_timeout */
            Window_NoReturn(win_ratio_after_timeout, tooth_period);
            break;

        case CRANK_COUNTING_TIMEOUT:  /* win_ratio_after_timeout */
            /**************************************************************
            * STATE: M8 - COUNTING_TIMEOUT
            * DESCRIPTION:
            *   Transition not detected in window after timeout, this is
            *   the second timeout, there has been one immediately before.
            *   Set ENG_POS_SEEK and IRQ, signal output functions and
            *   restart searching for the gap.
            **************************************************************/
            /* restart searching for the gap */
            Stall_NoReturn();
            break;

        case CRANK_TOOTH_BEFORE_GAP:  /* win_ratio_normal */
            /**************************************************************
            * STATE: M9 - TOOTH_BEFORE_GAP
            * DESCRIPTION:
            *   Transition not detected in normal window before gap.
            *   Set CRANK_ERR_TIMEOUT_BEFORE_GAP.
            *   Set ENG_POS_SEEK and IRQ, signal output functions and
            *   restart searching for the gap.
            **************************************************************/
            /* set error */
            error |= CRANK_ERR_TIMEOUT_BEFORE_GAP;
            /* restart searching for the gap */
            Stall_NoReturn();
            break;

        case CRANK_ADDITIONAL_TOOTH:   /* win_ratio_normal */
            /**************************************************************
            * STATE: M10A - CRANK_ADDITIONAL_TOOTH
            * DESCRIPTION:
            **************************************************************/
            /* set error */
            error |= CRANK_ERR_ADD_TOOTH_NOT_FOUND;
            /* restart searching for the gap */
            Stall_NoReturn();
            break;

        case CRANK_TOOTH_AFTER_GAP:   /* win_ratio_normal */
            /**************************************************************
            * STATE: M11 - TOOTH_AFTER_GAP
            * DESCRIPTION:
            *   Transition not detected in window across gap.
            *   Set CRANK_ERR_TIMEOUT_AFTER_GAP.
            *   Set ENG_POS_SEEK and IRQ, signal output functions and
            *   restart searching for the gap.
            **************************************************************/
            /* set error */
            error |= CRANK_ERR_TIMEOUT_AFTER_GAP;
            /* restart searching for the gap */
            Stall_NoReturn();
            break;

        case CRANK_SEEK:
        case CRANK_BLANK_TEETH:
        case CRANK_FIRST_TRANS:
            /**************************************************************
            * STATE: M0, M2, M3 - SEEK, BLANK_TEETH, FIRST_TRANS
            * DESCRIPTION:
            *   Match detection should never happen in this state.
            *   Set CRANK_ERR_INVALID_MATCH.
            **************************************************************/
            error |= CRANK_ERR_INVALID_MATCH;
            break;

        default:
            error |= CRANK_ERR_INTERNAL;
            break;
        }
    }
}

/**************************************************************************
* THREAD NAME: CRANK_TOOTH_TCR2_SYNC_GAP
* DESCRIPTION: This thread fires when it is safe to reset the TCR2 counter
*              during synchronization (gapped wheel mode).
**************************************************************************/
_eTPU_thread CRANK::CRANK_TOOTH_TCR2_SYNC_GAP(_eTPU_matches_enabled)
{
    channel.MRLA = MRL_CLEAR;
    channel.MRLB = MRL_CLEAR;
    channel.FLAG1 = CRANK_FLAG1_NORMAL_MODE;
    /* set back to match on TCR1 */
    channel.TBSA = TBS_M1C1GE;
    channel.TBSB = TBS_M1C2GE;

    /* reset tooth_counter_cycle */
    tooth_counter_cycle = 1;
    /* reset TCR2 */
    tcr2 = 0;
#ifdef ERRATA_2477
    err2477_tcr2_target = 0;
#endif

    erta = last_tooth_tcr1_time;
    state = CRANK_COUNTING_TIMEOUT;

    /* log tooth period (after possible tooth_counter_cycle reset) */
    ToothArray_Log(last_tooth_period_norm);
    /* open and close window using win_ratio_after_gap */
    Window_NoReturn(win_ratio_after_gap, last_tooth_period_norm);
}

/**************************************************************************
* THREAD NAME: CRANK_TOOTH_TCR2_SYNC_ADD
* DESCRIPTION: This thread fires when it is safe to reset the TCR2 counter
*              during synchronization (additional tooth wheel mode).
**************************************************************************/
_eTPU_thread CRANK::CRANK_TOOTH_TCR2_SYNC_ADD(_eTPU_matches_enabled)
{
    channel.MRLA = MRL_CLEAR;
    channel.MRLB = MRL_CLEAR;
    channel.FLAG1 = CRANK_FLAG1_NORMAL_MODE;
    /* set back to match on TCR1 */
    channel.TBSA = TBS_M1C1GE;
    channel.TBSB = TBS_M1C2GE;

    /* reset tooth_counter_cycle */
    tooth_counter_cycle = 1;
    /* reset TCR2 */
    tcr2 = 0;

    erta = last_tooth_tcr1_time;
    state = CRANK_COUNTING_TIMEOUT;

    /* open and close window using win_ratio_normal */
    Window_NoReturn(win_ratio_normal, last_tooth_period_norm);
}


DEFINE_ENTRY_TABLE(CRANK, CRANK, alternate, inputpin, autocfsr)
{
    //           HSR    LSR M1 M2 PIN F0 F1 vector
    ETPU_VECTOR2(6,7,   x,  x, x, x,  x, x, INIT),

    //           HSR    LSR M1 M2 PIN F0 F1 vector
    ETPU_VECTOR3(1,4,5, x,  x, x, x,  x, x, ANGLE_ADJUST),

    //           HSR    LSR M1 M2 PIN F0 F1 vector
    ETPU_VECTOR1(0,     x, 1, 1,  0, 0, 0, CRANK_WITH_GAP),
    ETPU_VECTOR1(0,     x, 1, 1,  1, 0, 0, CRANK_WITH_GAP),

    //           HSR    LSR M1 M2 PIN F0 F1 vector
    ETPU_VECTOR1(0,     x, 1, 1,  0, 1, 0, CRANK_WITH_ADDITIONAL_TOOTH),
    ETPU_VECTOR1(0,     x, 1, 1,  1, 1, 0, CRANK_WITH_ADDITIONAL_TOOTH),

    //           HSR    LSR M1 M2 PIN F0 F1 vector
    ETPU_VECTOR1(0,     x, 1, 1,  0, 0, 1, CRANK_TOOTH_TCR2_SYNC_GAP),
    ETPU_VECTOR1(0,     x, 1, 1,  1, 0, 1, CRANK_TOOTH_TCR2_SYNC_GAP),

    //           HSR    LSR M1 M2 PIN F0 F1 vector
    ETPU_VECTOR1(0,     x, 1, 1,  0, 1, 1, CRANK_TOOTH_TCR2_SYNC_ADD),
    ETPU_VECTOR1(0,     x, 1, 1,  1, 1, 1, CRANK_TOOTH_TCR2_SYNC_ADD),

    // unused/invalid entries
    ETPU_VECTOR2(2,3,   x, x, x,  0, 0, x, _Error_handler_unexpected_thread),
    ETPU_VECTOR2(2,3,   x, x, x,  0, 1, x, _Error_handler_unexpected_thread),
    ETPU_VECTOR2(2,3,   x, x, x,  1, 0, x, _Error_handler_unexpected_thread),
    ETPU_VECTOR2(2,3,   x, x, x,  1, 1, x, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,     1, 0, 0,  0, x, x, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,     1, 0, 0,  1, x, x, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,     x, 1, 0,  0, 0, 0, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,     x, 1, 0,  0, 1, 0, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,     x, 1, 0,  0, 0, 1, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,     x, 1, 0,  0, 1, 1, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,     x, 1, 0,  1, 0, 0, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,     x, 1, 0,  1, 1, 0, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,     x, 1, 0,  1, 0, 1, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,     x, 1, 0,  1, 1, 1, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,     x, 0, 1,  0, 0, 0, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,     x, 0, 1,  0, 1, 0, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,     x, 0, 1,  0, 0, 1, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,     x, 0, 1,  0, 1, 1, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,     x, 0, 1,  1, 0, 0, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,     x, 0, 1,  1, 1, 0, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,     x, 0, 1,  1, 0, 1, _Error_handler_unexpected_thread),
    ETPU_VECTOR1(0,     x, 0, 1,  1, 1, 1, _Error_handler_unexpected_thread),
};


/*******************************************************************************
*  Export interface information to Host CPU program.
*******************************************************************************/
/* [MISRA 2004 Rule 3.4] usage of #pragma write documented in the Pragma Write
   Manual, see https://www.ashware.com/Manuals */
#pragma write h, (::ETPUfilename (cpu/etpu_crank_auto.h));
#pragma write h, (/**************************************************************** );
#pragma write h, (* WARNING: This file is automatically generated. DO NOT EDIT IT! );
#pragma write h, (*);
#pragma write h, (* FILE NAME: etpu_crank_auto.h  );
#if defined(__TARGET_ETPU1__)
#pragma write h, (* ARCHITECTURE: eTPU );
#else
#pragma write h, (* ARCHITECTURE: eTPU2 );
#endif
#pragma write h, (*);
#pragma write h, (* This file was generated by: __FILE__ on __DATE__, __TIME__ );
#pragma write h, (*);
#pragma write h, (* This file provides an interface between eTPU code and CPU       );
#pragma write h, (* code. All references to the eTPU function should be made with   );
#pragma write h, (* information in this file. This allows only symbolic             );
#pragma write h, (* information to be referenced which allows the eTPU code to be   );
#pragma write h, (* optimized without effecting the CPU code.                       );
#pragma write h, (*****************************************************************/);
#pragma write h, (#ifndef _ETPU_CRANK_AUTO_H_ );
#pragma write h, (#define _ETPU_CRANK_AUTO_H_ );
#pragma write h, ( );
#pragma write h, (/* Function Configuration Information */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_FUNCTION_NUMBER) ::ETPUfunctionnumber(CRANK) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_TABLE_SELECT) ::ETPUentrytype(CRANK) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_NUM_PARMS) ::ETPUram(CRANK) );
#pragma write h, ( );
#pragma write h, (/* Host Service Request Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_HSR_INIT)         CRANK_HSR_INIT );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_HSR_SET_SYNC)     CRANK_HSR_SET_SYNC );
#pragma write h, ( );
#pragma write h, (/* Function Mode Bit Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_FM0_USE_TRANS_RISING)  CRANK_FM0_USE_TRANS_RISING );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_FM0_USE_TRANS_FALLING) CRANK_FM0_USE_TRANS_FALLING );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_FM1_TOOTH_PERIODS_LOG_OFF) (0) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_FM1_TOOTH_PERIODS_LOG_ON)  (CRANK_FM1_LOG_TOOTH_PERIODS << 1) );
#pragma write h, ( );
#pragma write h, (/* Parameter Definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_BLANK_TIME              ) ::ETPUlocation (CRANK, blank_time              ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_TCR1_CLOCK_SOURCE_DIV1  ) ::ETPUlocation (CRANK, tcr1_clock_source_div1  ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_TCR2_TICKS_PER_TOOTH    ) ::ETPUlocation (CRANK, tcr2_ticks_per_tooth    ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_TCR2_TICKS_PER_ADD_TOOTH) ::ETPUlocation (CRANK, tcr2_ticks_per_add_tooth) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_LAST_TOOTH_TCR1_TIME    ) ::ETPUlocation (CRANK, last_tooth_tcr1_time    ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_LAST_TOOTH_PERIOD       ) ::ETPUlocation (CRANK, last_tooth_period       ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_LAST_TOOTH_PERIOD_NORM  ) ::ETPUlocation (CRANK, last_tooth_period_norm  ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_ADDITIONAL_TOOTH_PERIOD ) ::ETPUlocation (CRANK, additional_tooth_period ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_TCR2_ADJUSTMENT         ) ::ETPUlocation (CRANK, tcr2_adjustment         ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_GAP_RATIO               ) ::ETPUlocation (CRANK, gap_ratio               ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_WIN_RATIO_NORMAL        ) ::ETPUlocation (CRANK, win_ratio_normal        ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_WIN_RATIO_ACROSS_GAP    ) ::ETPUlocation (CRANK, win_ratio_across_gap    ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_WIN_RATIO_AFTER_GAP     ) ::ETPUlocation (CRANK, win_ratio_after_gap     ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_WIN_RATIO_AFTER_TIMEOUT ) ::ETPUlocation (CRANK, win_ratio_after_timeout ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_FIRST_TOOTH_TIMEOUT     ) ::ETPUlocation (CRANK, first_tooth_timeout     ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_LINK_CAM                ) ::ETPUlocation (CRANK, link_cam                ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_LINK_1                  ) ::ETPUlocation (CRANK, link_1                  ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_LINK_2                  ) ::ETPUlocation (CRANK, link_2                  ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_LINK_3                  ) ::ETPUlocation (CRANK, link_3                  ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_LINK_4                  ) ::ETPUlocation (CRANK, link_4                  ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_TEETH_TILL_GAP          ) ::ETPUlocation (CRANK, teeth_till_gap          ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_TEETH_IN_GAP            ) ::ETPUlocation (CRANK, teeth_in_gap            ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_MISSCNT_MASK            ) ::ETPUlocation (CRANK, misscnt_mask            ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_TEETH_PER_CYCLE         ) ::ETPUlocation (CRANK, teeth_per_cycle         ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_TEETH_PER_SYNC          ) ::ETPUlocation (CRANK, teeth_per_sync          ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_TOOTH_COUNTER_GAP       ) ::ETPUlocation (CRANK, tooth_counter_gap       ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_TOOTH_COUNTER_CYCLE     ) ::ETPUlocation (CRANK, tooth_counter_cycle     ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_BLANK_TEETH             ) ::ETPUlocation (CRANK, blank_teeth             ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_STATE                   ) ::ETPUlocation (CRANK, state                   ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_ERROR                   ) ::ETPUlocation (CRANK, error                   ) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_TOOTH_PERIOD_LOG        ) ::ETPUlocation (CRANK, tooth_period_log        ) );
#ifdef ERRATA_2477
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_OFFSET_ERR2477_TCR2_TARGET     ) ::ETPUlocation (CRANK, err2477_tcr2_target     ) );
#endif
#pragma write h, ( );
#pragma write h, (/* Globals definitions */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_OFFSET_ENG_POS_STATE                 )  ::ETPUlocation (eng_pos_state) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_OFFSET_ENG_CYCLE_TCR2_TICKS          )  ::ETPUlocation (eng_cycle_tcr2_ticks) );
#pragma write h, (::ETPUliteral(#define FS_ETPU_OFFSET_ENG_CYCLE_TCR2_START          )  ::ETPUlocation (eng_cycle_tcr2_start) );
#pragma write h, ( );
#pragma write h, (/* Errors */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_ERR_NO_ERROR           ) CRANK_ERR_NO_ERROR           );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_ERR_INVALID_TRANS      ) CRANK_ERR_INVALID_TRANS      );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_ERR_INVALID_MATCH      ) CRANK_ERR_INVALID_MATCH      );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_ERR_TIMEOUT            ) CRANK_ERR_TIMEOUT            );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_ERR_STALL              ) CRANK_ERR_STALL              );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_ERR_INTERNAL           ) CRANK_ERR_INTERNAL           );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_ERR_TIMEOUT_BEFORE_GAP ) CRANK_ERR_TIMEOUT_BEFORE_GAP );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_ERR_TIMEOUT_AFTER_GAP  ) CRANK_ERR_TIMEOUT_AFTER_GAP  );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_ERR_TOOTH_IN_GAP       ) CRANK_ERR_TOOTH_IN_GAP       );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_ERR_ADD_TOOTH_NOT_FOUND) CRANK_ERR_ADD_TOOTH_NOT_FOUND);
#pragma write h, ( );
#pragma write h, (/* Crank State values */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_SEEK                     ) CRANK_SEEK                     );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_BLANK_TIME               ) CRANK_BLANK_TIME               );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_BLANK_TEETH              ) CRANK_BLANK_TEETH              );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_FIRST_TRANS              ) CRANK_FIRST_TRANS              );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_SECOND_TRANS             ) CRANK_SECOND_TRANS             );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_TEST_POSSIBLE_GAP        ) CRANK_TEST_POSSIBLE_GAP        );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_VERIFY_GAP               ) CRANK_VERIFY_GAP               );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_COUNTING                 ) CRANK_COUNTING                 );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_COUNTING_TIMEOUT         ) CRANK_COUNTING_TIMEOUT         );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_TOOTH_BEFORE_GAP         ) CRANK_TOOTH_BEFORE_GAP         );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_TOOTH_BEFORE_GAP_NOT_HRM ) CRANK_TOOTH_BEFORE_GAP_NOT_HRM );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_ADDITIONAL_TOOTH         ) CRANK_ADDITIONAL_TOOTH         );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_TOOTH_AFTER_GAP          ) CRANK_TOOTH_AFTER_GAP          );
#pragma write h, (::ETPUliteral(#define FS_ETPU_CRANK_TOOTH_TCR2_SYNC          ) CRANK_TOOTH_TCR2_SYNC          );
#pragma write h, ( );
#pragma write h, (/* Global Engine Position State values */);
#pragma write h, (::ETPUliteral(#define FS_ETPU_ENG_POS_SEEK            ) ENG_POS_SEEK            );
#pragma write h, (::ETPUliteral(#define FS_ETPU_ENG_POS_FIRST_HALF_SYNC ) ENG_POS_FIRST_HALF_SYNC );
#pragma write h, (::ETPUliteral(#define FS_ETPU_ENG_POS_PRE_FULL_SYNC   ) ENG_POS_PRE_FULL_SYNC   );
#pragma write h, (::ETPUliteral(#define FS_ETPU_ENG_POS_FULL_SYNC       ) ENG_POS_FULL_SYNC       );
#pragma write h, ( );
#pragma write h, (#endif );

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
 *  Revision 1.2  2015/09/01  r54529
 *  Output parameter last_tooth_period_norm added.
 *
 *  Revision 1.11  2015/06/29  r54529
 *  eTPU compiler 10.2.2, minor change at line 579.
 *
 *  Revision 1.1  2014/10/27  r54529
 *  Crank with an additional tooth generated (teeth_till_gap + 1)*tcr2_ticks_per_tooth ticks per 360deg.
 *  Fixed, parameter tcr2_ticks_per_add_tooth added.
 *
 *  Revision 1.0  2014/03/16  r54529
 *  Bug fix - tooth_period_norm used for TRR calculation on tooth after gap.
 *  Minor comment and formating improvements. MISRA compliance checked.
 *  Ready for eTPU Engine Control Library release 1.0.
 *
 *  Revision 0.3  2013/11/27  r54529
 *  Full precision of TRR calculation.
 *  MISSCNT[2] supported - up to 7 missing teeth. teeth_in_gap replaced by misscnt_mask.
 *
 *  Revision 0.2  2013/08/14  r54529
 *  TCR2 not reset on stall.
 *
 *  Revision 0.1  2012/06/12  r54529
 *  Initial version.
 *
 *******************************************************************************/
