/**
 * @file scheduler.c
 * @author KD
 * @date 9/23/21
 * @brief Tracks the order and status of interrupts
 *
 */

/*
 * scheduler.c
 *
 *  Created on: Sep 16, 2021
 *      Author: kaidriscoll
 */

//******************************************************************************
// Include files
//******************************************************************************
#include "scheduler.h"


//******************************************************************************
// Private variables
//******************************************************************************
static unsigned int event_scheduled;


//******************************************************************************
// Functions
//******************************************************************************
/*******************************************************************************
 * @brief Initializer for scheduler
 *
 *
 * @details Initializes the static variable event_scheduled to 0
 *
 *
 * @note
 *
 *
 *
 ******************************************************************************/
void scheduler_open(void) {
  event_scheduled = 0;
}

/*******************************************************************************
 * @brief Adds an interrupt event to the list
 *
 *
 * @details Uses an OR gate to add the event to the register event_scheduled
 *
 *
 * @note Atomic State
 *
 *@param[in] event
 * interrupt event to add to the list
 ******************************************************************************/
void add_scheduled_event(uint32_t event) {
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  event_scheduled |= event;
  CORE_EXIT_CRITICAL();
}

/*******************************************************************************
 * @brief Removes an interrupt event from the list
 *
 *
 * @details uses an AND, NOT Gate to remove the event from the register event_scheduled
 *
 *
 * @note Atomic State
 *
 * @param[in] event
 * interrupt event to remove from the list
 ******************************************************************************/
void remove_scheduled_event(uint32_t event) {
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  event_scheduled &= ~event;
  CORE_EXIT_CRITICAL();
}

/*******************************************************************************
 * @brief Passes static variable of register of scheduled interrupt events
 *
 *
 * @details returns static event_schuled
 *
 *
 * @note
 *
 * @param[out] uint32_t
 * registry of the scheduled interrupts
 ******************************************************************************/
uint32_t get_scheduled_events(void) {
  return event_scheduled;
}
