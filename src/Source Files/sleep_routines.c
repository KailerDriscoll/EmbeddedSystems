/**
 * @file sleep_routines.c
 * @author KD
 * @date 9/23/21
 * @brief Retains information about lowest possible sleep mode
 *
 */
 /***************************************************************************
 * @section License
 * <b>(C) Copyright 2015 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
*
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
**************************************************************************/

//******************************************************************************
// Include files
//******************************************************************************
#include "sleep_routines.h"


//******************************************************************************
// Private variables
//******************************************************************************
static int lowest_energy_mode[MAX_ENERGY_MODES];


//******************************************************************************
// Functions
//******************************************************************************
/*******************************************************************************
 * @brief Initializes global variable
 *
 *
 * @details Uses an AND, NOT Gate to remove the event from the register event_scheduled
 *
 *
 * @note Atomic State
 *
 * @param[in] event
 * interrupt event to remove from the list
 ******************************************************************************/
void sleep_open(void) {
  for (int i=0; i<MAX_ENERGY_MODES; i++) {
      lowest_energy_mode[i] = 0;
  }
}

/*******************************************************************************
 * @brief Updates lowest energy mode registry
 *
 *
 * @details Increases the number of peripherals at the lowest energy mode
 *
 *
 * @note Atomic State
 *
 * @param[in] EM
 * Lowest energy mode of peripheral
 ******************************************************************************/
void sleep_block_mode(uint32_t EM) {
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  lowest_energy_mode[EM]++;
  EFM_ASSERT(lowest_energy_mode[EM] < 5);
  CORE_EXIT_CRITICAL();
}

/*******************************************************************************
 * @brief Updates lowest energy mode registry
 *
 *
 * @details Decreases the number of peripherals at the lowest energy mode
 *
 *
 * @note Atomic State
 *
 * @param[in] EM
 * Lowest energy mode of peripheral
 ******************************************************************************/
void sleep_unblock_mode(uint32_t EM) {
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  lowest_energy_mode[EM]--;
  EFM_ASSERT(lowest_energy_mode[EM] >= 0);
  CORE_EXIT_CRITICAL();
}

/*******************************************************************************
 * @brief Sets the board to the lowest possible energy level
 *
 *
 * @details Checks lowest_energy_mode until it finds a level with peripherals >
 * 0, then it sets the energy mode to that level
 *
 *
 * @note Atomic State
 *
 ******************************************************************************/
void enter_sleep(void) {
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  if (lowest_energy_mode[EM0] > 0) {}
  else if (lowest_energy_mode[EM1] > 0) {}
  else if (lowest_energy_mode[EM2] > 0) {EMU_EnterEM1();}
  else if (lowest_energy_mode[EM3] > 0) {EMU_EnterEM2(true);}
  else {EMU_EnterEM3(true);}

  CORE_EXIT_CRITICAL();
}

/*******************************************************************************
 * @brief Returns lowest energy mode possible for the peripherals
 *
 *
 * @details Iterates through lowest_energy_mode and returns the energy mode
 * before the lowest energy mode that has 0 peripherals
 *
 *
 * @note
 *
 * @param[out] unint32_t
 * Lowest energy mode
 ******************************************************************************/
uint32_t current_block_energy_mode(void) {
  for (int i=0; i<MAX_ENERGY_MODES; i++) {
      if (lowest_energy_mode[i] != 0) {return i;}
  }
  return MAX_ENERGY_MODES-1;
}
