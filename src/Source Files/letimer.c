/**
 * @file letimer.c
 * @author KD
 * @date 9/9/21
 * @brief Open LETIMER Peripheral and enables/disables specified LETIMER
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "letimer.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// Private variables
//***********************************************************************************
static uint32_t scheduled_comp0_cb;
static uint32_t scheduled_comp1_cb;
static uint32_t scheduled_uf_cb;

//***********************************************************************************
// Private functions
//***********************************************************************************


//***********************************************************************************
// Global functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 *   Driver to open an set an LETIMER peripheral in PWM mode
 *
 * @details
 *   This routine is a low level driver.  The application code calls this function
 *   to open one of the LETIMER peripherals for PWM operation to directly drive
 *   GPIO output pins of the device and/or create interrupts that can be used as
 *   a system "heart beat" or by a scheduler to determine whether any system
 *   functions need to be serviced.
 *
 * @note
 *   This function is normally called once to initialize the peripheral and the
 *   function letimer_start() is called to turn-on or turn-off the LETIMER PWM
 *   operation.
 *
 * @param[in] letimer
 *   Pointer to the base peripheral address of the LETIMER peripheral being opened
 *
 * @param[in] app_letimer_struct
 *   Is the STRUCT that the calling routine will use to set the parameters for PWM
 *   operation
 *
 ******************************************************************************/

void letimer_pwm_open(LETIMER_TypeDef *letimer, APP_LETIMER_PWM_TypeDef *app_letimer_struct){
	LETIMER_Init_TypeDef letimer_pwm_values;

	unsigned int period_cnt;
	unsigned int period_active_cnt;

	/*  Initializing LETIMER for PWM mode */
	/*  Enable the routed clock to the LETIMER0 peripheral */

	  if (letimer == LETIMER0) {CMU_ClockEnable(cmuClock_LETIMER0, true);}

  // Verify whether the LETIMER clock tree properly configured and enabled
  /* Use EFM_ASSERT statements to verify whether the LETIMER clock tree is properly
   * configured and enabled
   * You must select a register that utilizes the clock enabled to be tested
   * With the LETIMER registers being in the low frequency clock tree, you must
   * use a while SYNCBUSY loop to verify that the write of the register has propagated
   * into the low frequency domain before reading it. */

	  letimer_start(letimer,false);

	  letimer->CMD = LETIMER_CMD_START;
	  while (letimer->SYNCBUSY);
	  EFM_ASSERT(letimer->STATUS & LETIMER_STATUS_RUNNING);
	  letimer->CMD = LETIMER_CMD_STOP;
	  while (letimer->SYNCBUSY);

	// Must reset the LETIMER counter register since enabling the LETIMER to verify that
	// the clock tree has been correctly configured to the LETIMER may have resulted in
	// the counter counting down from 0 and underflowing which by default will load
	// the value of 0xffff.  To load the desired COMP0 value quickly into this
	// register after complete initialization, it must start at 0 so that the underflow
	// will happen quickly upon enabling the LETIMER loading the desired top count from
	// the COMP0 register.

  // Reset the Counter to a know value such as 0
  letimer->CNT = 0; // What is the register enumeration to use to specify the LETIMER Counter Register?

  // Initialize letimer for PWM operation
  // XXX are values passed into the driver via the input app_letimer_struct
  // ZZZ are values that you must specify for this PWM specific driver from the online HAL documentation
	letimer_pwm_values.bufTop = false;		// Comp1 will not be used to load comp0, but used to create an on-time/duty cycle
	letimer_pwm_values.comp0Top = true;		// load comp0 into cnt register when count register underflows enabling continuous looping
	letimer_pwm_values.debugRun = app_letimer_struct->debugRun;
	letimer_pwm_values.enable = app_letimer_struct->enable;
	letimer_pwm_values.out0Pol = false;			// While PWM is not active out, idle is DEASSERTED, 0
	letimer_pwm_values.out1Pol = false;			// While PWM is not active out, idle is DEASSERTED, 0
	letimer_pwm_values.repMode = letimerRepeatFree;	// Setup letimer for free running for continuous looping
	letimer_pwm_values.ufoa0 = letimerUFOAPwm;		// Using the HAL documentation, set to PWM mode
	letimer_pwm_values.ufoa1 = letimerUFOAPwm;		// Using the HAL documentation, set to PWM mode



	LETIMER_Init(letimer, &letimer_pwm_values);		// Initialize letimer

	while (letimer->SYNCBUSY);
  /* Calculate the value of COMP0 and COMP1 and load these control registers
   * with the calculated values
   */
	period_cnt = app_letimer_struct->period * LETIMER_HZ;
	period_active_cnt = app_letimer_struct->active_period * LETIMER_HZ;

	LETIMER_CompareSet(letimer, 0, period_cnt);				    // comp0 register is PWM period
	LETIMER_CompareSet(letimer, 1, period_active_cnt);		// comp1 register is PWM active period

  /* Set the REP0 mode bits for PWM operation directly since this driver is PWM specific.
   * Datasheets are very specific and must be read very carefully to implement correct functionality.
   * Sometimes, the critical bit of information is a single sentence out of a 30-page datasheet
   * chapter.  Look careful in the following section of the Mighty Gecko Reference Manual in the
   * notes section of Table 21.2. LETIMER Underflow Output Actions to learn how to correctly set the
   * REP0 and REP1 bits
   */
	letimer->REP0 = 1;
	letimer->REP1 = 1;


   /* Use the values from app_letimer_struct input argument for ROUTELOC0 register for both the
    * OUT0LOC and OUT1LOC fields */
	letimer->ROUTELOC0 = app_letimer_struct->out_pin_route0 | app_letimer_struct->out_pin_route1;

  /* Use the values from app_letimer_struct input argument to program the ROUTEPEN register for both
   * the OUT 0 Pin Enable (OUT0PEN) and the OUT 1 Pin Enable (OUT1PEN) in combination with the
   * enumeration of these pins utilizing boolean multiplication*/
	letimer->ROUTEPEN = (LETIMER_ROUTEPEN_OUT0PEN * app_letimer_struct->out_pin_0_en) | (LETIMER_ROUTEPEN_OUT1PEN * app_letimer_struct->out_pin_1_en);


  /* We are not enabling any interrupts at this tie.  If you were, you would enable them now */

	LETIMER0->IFC |= LETIMER_IFC_COMP0;
	LETIMER0->IFC |= LETIMER_IFC_COMP1;
	LETIMER0->IFC |= LETIMER_IFC_UF;

	LETIMER0->IEN |= app_letimer_struct->comp0_irq_enable * LETIMER_IEN_COMP0;
	LETIMER0->IEN |= app_letimer_struct->comp1_irq_enable * LETIMER_IEN_COMP1;
	LETIMER0->IEN |= app_letimer_struct->uf_irq_enable * LETIMER_IEN_UF;
	NVIC_EnableIRQ(LETIMER0_IRQn);


  scheduled_comp0_cb = app_letimer_struct->comp0_cb;
	scheduled_comp1_cb = app_letimer_struct->comp1_cb;
	scheduled_uf_cb = app_letimer_struct->uf_cb;


  /* We will not enable or turn-on the LETIMER0 at this time */
	if (letimer->STATUS && LETIMER_STATUS_RUNNING) {
	    sleep_block_mode(LETIMER_EM);
	}
}


/***************************************************************************//**
 * @brief
 *   Function to enable/turn-on or disable/turn-off the LETIMER specified
 *
 * @details
 *   letimer_start uses the lower level API interface of the EM libraries to
 *   directly interface to the LETIMER peripheral to turn-on or off its counter
 *
 * @note
 *   This function should only be called to enable/turn-on the LETIMER once the
 *   LETIMER peripheral has been completely configured via its open driver
 *
 * @param[in] letimer
 *   Pointer to the base peripheral address of the LETIMER peripheral being opened
 *
 * @param[in] enable
 *   Variable to turn-on the LETIMER if boolean value = true and turn-off the LETIMER
 *   if the boolean value = false
 *
 ******************************************************************************/

void letimer_start(LETIMER_TypeDef *letimer, bool enable){
  if (letimer->STATUS && LETIMER_STATUS_RUNNING && !enable) {
      sleep_unblock_mode(LETIMER_EM);
      LETIMER_Enable(letimer, enable);
      while(letimer->SYNCBUSY);
  }
  if (!letimer->STATUS && LETIMER_STATUS_RUNNING && enable) {
      sleep_block_mode(LETIMER_EM);
      LETIMER_Enable(letimer, enable);
      while(letimer->SYNCBUSY);
  }
}

/*******************************************************************************
 * @brief Handles Interrupts
 *
 *
 * @details Reads interrupts and adds the to the list of interrupt events
 *
 *
 * @note
 *
 *
 *
 ******************************************************************************/

void LETIMER0_IRQHandler(void) {
  uint32_t int_flag;
  int_flag = LETIMER0->IF & LETIMER0->IEN;
  LETIMER0->IFC = int_flag;

  if (int_flag & LETIMER_IF_UF) {
      EFM_ASSERT(!(LETIMER0->IF & LETIMER_IF_UF));
      add_scheduled_event(scheduled_uf_cb);
  }
  if (int_flag & LETIMER_IF_COMP0) {
      EFM_ASSERT(!(LETIMER0->IF & LETIMER_IF_COMP0));
      add_scheduled_event(scheduled_comp0_cb);
  }
  if (int_flag & LETIMER_IF_COMP1) {
      EFM_ASSERT(!(LETIMER0->IF & LETIMER_IF_COMP1));
      add_scheduled_event(scheduled_comp1_cb);
  }

}

// Lab 7
/*******************************************************************************
 * @brief Change LETIMER Period
 *
 *
 * @details Changes Letimer period by "change"
 *          Called by app.c
 *
 * @param[in] LETIMER_TypeDef *letimer
 * specifies LETIMER module
 *
 * @param[in] int change
 * Specifies how much to change LETIMER period by
 *
 * @note Ensures LETIMER is disabled before changing the period
 *
 ******************************************************************************/
void change_LETIMER_period(LETIMER_TypeDef *letimer, int change) {
  // If letimer is enabled, disable it
  if (letimer->STATUS && LETIMER_STATUS_RUNNING) {
      //sleep_block_mode(LETIMER_EM);
      LETIMER_Enable(letimer, false);
      while(letimer->SYNCBUSY);
      uint32_t period = letimer->COMP0;
      int outcome = (int) period + change;
      letimer->COMP0 = outcome;
      LETIMER_Enable(letimer, true);
  } else {
      while(letimer->SYNCBUSY);
      uint32_t period = letimer->COMP0;
      int outcome = (int) period + change;
      letimer->COMP0 = outcome;
      LETIMER_Enable(letimer, true);
  }


}
