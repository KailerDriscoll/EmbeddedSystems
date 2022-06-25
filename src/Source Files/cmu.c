/**
 * @file cmu.c
 * @author KD
 * @date 9/9/21
 * @brief Responsible in enabling all oscillators and routing the clock tree for the application
 *
 */
//***********************************************************************************
// Include files
//***********************************************************************************
#include "cmu.h"
#include "brd_config.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// Private variables
//***********************************************************************************


//***********************************************************************************
// Private functions
//***********************************************************************************


//***********************************************************************************
// Global functions
//***********************************************************************************

/***************************************************************************//**
 * @brief enables all oscillators and CMU clock
 *
 *
 * @details Enables the High-frequency peripheral clock,
 * Sets Low-frequency RC oscillator to false,
 * Sets Low-frequency Crystal Oscillator to false,
 * Sets clock to Low-frequency A clock with ULFRCO cmu,
 * Enables Low-energy clock divided down from HFCLK
 *
 *
 * @note
 *
 *
 ******************************************************************************/

void cmu_open(void){

    CMU_ClockEnable(cmuClock_HFPER, true);

    // By default, LFRCO is enabled, disable the LFRCO oscillator
    // Disable the LFRCO oscillator
    // What is the enumeration required for LFRCO?
    // It can be found in the online HAL documentation
     CMU_OscillatorEnable(cmuOsc_LFRCO , false, false);

    // Disable the LFXO oscillator
    // What is the enumeration required for LFXO?
    // It can be found in the online HAL documentation
    CMU_OscillatorEnable(cmuOsc_LFXO , false, false);

    // No requirement to enable the ULFRCO oscillator.  It is always enabled in EM0-4H1

    // Route LF clock to the LF clock tree
    // What is the enumeration required to placed the ULFRCO onto the proper clock branch?
    // It can be found in the online HAL documentation
    CMU_ClockSelectSet(cmuClock_LFA , cmuSelect_ULFRCO);    // routing ULFRCO to proper Low Freq clock tree

    // What is the proper enumeration to enable the clock tree onto the LE clock branches?
    // It can be found in the Assignment 2 documentation
    CMU_ClockEnable(cmuClock_HFLE, true);


    CMU_HFRCOBandSet(MCU_HFXO_FREQ);// Defined in brd_config.h
    CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
    CMU_OscillatorEnable(cmuOsc_HFXO, false, false);

    //Lab 6
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);

}

