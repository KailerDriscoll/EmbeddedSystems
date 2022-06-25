/**
 * @file leuart.c
 * @author
 * @date
 * @brief Contains all the functions of the LEUART peripheral
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************

//** Standard Library includes
#include <string.h>

//** Silicon Labs include files
#include "em_gpio.h"
#include "em_cmu.h"

//** Developer/user include files
#include "leuart.h"
#include "scheduler.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// private variables
//***********************************************************************************
uint32_t	rx_done_evt;
uint32_t	tx_done_evt;
bool		leuart0_tx_busy;
static char result[80];

/***************************************************************************//**
 * @brief LEUART driver
 * @details
 *  This module contains all the functions to support the driver's state
 *  machine to transmit a string of data across the LEUART bus.  There are
 *  additional functions to support the Test Driven Development test that
 *  is used to validate the basic set up of the LEUART peripheral.  The
 *  TDD test for this class assumes that the LEUART is connected to the HM-18
 *  BLE module.  These TDD support functions could be used for any TDD test
 *  to validate the correct setup of the LEUART.
 *
 ******************************************************************************/

//***********************************************************************************
// Private functions
//***********************************************************************************
static LEUART_STATE_MACHINE leuart_sm_write;
static LEUART_STATE_MACHINE leuart_sm_read;

void leuart_txbl(void);
void leuart_txc(void);

// Lab 7
static void leuart_rx_tdd(LEUART_TypeDef *leuart);
void leuart_rxdatav(void);
void leuart_startf(void);
void leuart_sigf(void);



//***********************************************************************************
// Global functions
//***********************************************************************************

/***************************************************************************//**
 * @brief Enables required registers to use the LEUART0
 *
 * @details Enables clock tree,
 *          Sets STARTFRAME,
 *          Enables TXEN & RXEN Registers
 *          Routes ROUTELOC0 & ROUTEPEN
 *          Initializes & Enables LEUART0
 *          Enables IRQ
 *
 * @param[in] LEUART_TypeDef leuart
 * specifies leuart module
 *
 * @param[in] LEUART_OPEN_STRUCT leuart_settings
 * contains settings for static leuart
 ******************************************************************************/

void leuart_open(LEUART_TypeDef *leuart, LEUART_OPEN_STRUCT *leuart_settings){
  if (leuart == LEUART0) {
        CMU_ClockEnable(cmuClock_LEUART0, true);
  } else {
        EFM_ASSERT(false);
  }

    leuart->STARTFRAME = 0x01;
    while (leuart->SYNCBUSY & LEUART_SYNCBUSY_CMD);
    EFM_ASSERT(leuart->STARTFRAME & 0x01);
    leuart->STARTFRAME &= 0x00;


    // Check for Transmit Buffer
    leuart->CMD = LEUART_CMD_TXEN;
    while (leuart->SYNCBUSY & LEUART_SYNCBUSY_CMD);
    while (!(leuart->STATUS & LEUART_STATUS_TXENS));
    EFM_ASSERT(leuart->STATUS & LEUART_STATUS_TXENS);
    leuart->CMD = LEUART_CMD_TXDIS;
    while (leuart->SYNCBUSY & LEUART_SYNCBUSY_CMD);
    while (!(leuart->STATUS & LEUART_STATUS_TXENS));

    // Check for Read Buffer
    leuart->CMD = LEUART_CMD_RXEN;
    while (leuart->SYNCBUSY & LEUART_SYNCBUSY_CMD);
    while (!(leuart->STATUS & LEUART_STATUS_RXENS));
    EFM_ASSERT(leuart->STATUS & LEUART_STATUS_RXENS);
    leuart->CMD = LEUART_CMD_RXDIS;
    while (leuart->SYNCBUSY & LEUART_SYNCBUSY_CMD);
    while (!(leuart->STATUS & LEUART_STATUS_RXENS));


    LEUART_Init_TypeDef leuart_values;
    leuart_values.refFreq = leuart_settings->refFreq;
    //leuart_values.enable = leuart_settings->enable;
    leuart_values.baudrate = leuart_settings->baudrate;
    leuart_values.databits = leuart_settings->databits;
    leuart_values.parity = leuart_settings->parity;
    leuart_values.stopbits = leuart_settings->stopbits;

    leuart->ROUTELOC0 = leuart_settings->rx_loc | leuart_settings->tx_loc;
    leuart->ROUTEPEN = (LEUART0_RX_ROUTE * leuart_settings->rx_en) | ((LEUART0_TX_ROUTE * leuart_settings->tx_en));

    //clear buffers
    leuart->CMD = LEUART_CMD_CLEARRX;
    leuart->CMD = LEUART_CMD_CLEARTX;

    LEUART_Init(leuart, &leuart_values);
    while (leuart->SYNCBUSY);
    LEUART_Enable(leuart, HM10_ENABLE);

    while (leuart->SYNCBUSY & LEUART_SYNCBUSY_CMD);
    while (!(leuart->STATUS & LEUART_STATUS_RXENS));
    EFM_ASSERT(leuart->STATUS & LEUART_STATUS_RXENS);
    while (leuart->SYNCBUSY & LEUART_SYNCBUSY_CMD);
    while (!(leuart->STATUS & LEUART_STATUS_TXENS));
    EFM_ASSERT(leuart->STATUS & LEUART_STATUS_TXENS);



// Lab 7
    leuart->STARTFRAME = leuart_settings->startframe;
    leuart->SIGFRAME = leuart_settings->sigframe;


    leuart->IFC |= LEUART_IFC_TXC;
    leuart->IEN |= LEUART_IEN_STARTF | LEUART_IEN_SIGF | LEUART_IEN_RXDATAV;
    leuart->CMD |= LEUART_CMD_RXBLOCKEN;
    while(leuart->SYNCBUSY & LEUART_SYNCBUSY_CMD);
    leuart->CTRL |= LEUART_CTRL_SFUBRX;
    while(leuart->SYNCBUSY & LEUART_SYNCBUSY_CTRL);

    NVIC_EnableIRQ(LEUART0_IRQn);

    sleep_block_mode(LEUART_TX_EM<LEUART_RX_EM ? LEUART_TX_EM : LEUART_RX_EM);
    leuart_rx_tdd(leuart);
}


/***************************************************************************//**
 * @brief TDD for LEUART Read capability
 *
 * @details Tests Loopback with STARTFRAMES and SIGFRAMES
 *          Sends String into TX and receives it to RX
 *
 * @param[in] LEUART_TypeDef leuart
 * specifies leuart module
 ******************************************************************************/
static void leuart_rx_tdd(LEUART_TypeDef *leuart) {
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  uint32_t Save_IEN = leuart->IEN; // Save current state of the machine
  leuart->IEN = 0;

  leuart->CTRL |= LEUART_CTRL_LOOPBK; // Enable LoopBack
  while (leuart->SYNCBUSY);

  uint32_t local_STARTF, local_SIGF;  // Local Memory of STARTFRAME and SIGFRAME
  local_STARTF = leuart->STARTFRAME;
  local_SIGF = leuart->SIGFRAME;


  // 1) Test whether LEUART blocks a non-Start Frame Character
  leuart->TXDATA = ~local_STARTF;
  timer_delay(TWO_MS);
  EFM_ASSERT(!(leuart->IF & LEUART_IF_RXDATAV));

  // 2) Test whether LEUART receives a Start Frame Character
  leuart->TXDATA = local_STARTF;
  timer_delay(TWO_MS);
  EFM_ASSERT(leuart->IF & LEUART_IF_RXDATAV);
  EFM_ASSERT(local_STARTF == leuart->RXDATA);

  // 3) Test whether any additional character is received
  EFM_ASSERT(!(leuart->IF & LEUART_IF_RXDATAV));

  // 4) Test whether LEUART properly indicates a Signal Frame
  leuart->TXDATA = local_SIGF;
  timer_delay(TWO_MS);
  EFM_ASSERT(leuart->IF & LEUART_IF_SIGF);
  EFM_ASSERT(local_SIGF == leuart->RXDATA);

  leuart->CMD |= LEUART_CMD_RXBLOCKEN; // Re-enable RXBLOCKEN
  while (leuart->SYNCBUSY);


  leuart->IFC |= LEUART_IFC_STARTF | LEUART_IFC_SIGF;


  CORE_EXIT_CRITICAL(); // Currently no State Machine to check Read


  char str[20] = "123";
  str[strlen(str)] = local_STARTF;
  str[strlen(str)] = 0;
  strcat(str, "abc");
  str[strlen(str)] = local_SIGF;
  str[strlen(str)] = 0;
  strcat(str, "xyz");
  str[strlen(str)] = 0;


  leuart->CMD |= LEUART_CMD_RXBLOCKEN;
  while(leuart->SYNCBUSY & LEUART_SYNCBUSY_CMD);
  leuart->IEN = Save_IEN;

  leuart_start(leuart, str, strlen(str), RX_TDD_TEST);

  timer_delay(30);
  EFM_ASSERT(!strcmp("#abc!", result));
  leuart->IEN = Save_IEN;

  leuart->CMD |= LEUART_CMD_RXBLOCKEN;
  while (leuart->SYNCBUSY & LEUART_SYNCBUSY_CMD);
  EFM_ASSERT(leuart->STATUS & LEUART_STATUS_RXBLOCK);

  leuart->CTRL &= ~LEUART_CTRL_LOOPBK;
  while (leuart->SYNCBUSY & LEUART_SYNCBUSY_CTRL);
  remove_scheduled_event(RX_TDD_TEST);

}





/***************************************************************************//**
 * @brief Interrupt handler for the LEUART0
 *
 * @details Organizes and executes commands for TXBL, TXC, STARTF, SIGF, RXDATAV
 *
 ******************************************************************************/

void LEUART0_IRQHandler(void){
  uint32_t int_flag;
  int_flag = LEUART0->IF & LEUART0->IEN;
  LEUART0->IFC = int_flag;

  if (int_flag & LEUART_IF_TXBL) {
      //EFM_ASSERT(!(LEUART0->IF & LEUART_IF_TXBL));
      leuart_txbl();
  }
  if (int_flag & LEUART_IF_TXC) {
      EFM_ASSERT(!(LEUART0->IF & LEUART_IF_TXC));
      leuart_txc();
  }
  if (int_flag & LEUART_IF_STARTF) {
      EFM_ASSERT(!(LEUART0->IF & LEUART_IF_STARTF));
      leuart_startf();
  }
  if (int_flag & LEUART_IF_SIGF) {
      EFM_ASSERT(!(LEUART0->IF & LEUART_IF_SIGF));
      leuart_sigf();
  }
  if (int_flag & LEUART_IF_RXDATAV) {
      //EFM_ASSERT(!(LEUART0->IF & LEUART_IF_RXDATAV));
      leuart_rxdatav();
  }
}

/***************************************************************************//**
 * @brief TXBL Interrupt Routine
 *
 * @details Transfers data to TX Buffer while there are still more characters to
 *be read
 *          Enables TXC Interrupt once done writing to BLE
 *
 ******************************************************************************/
void leuart_txbl(void) {
  switch (leuart_sm_write.current_state) {
    case WRITE_DATA:
      if (leuart_sm_write.count < leuart_sm_write.string_len) {
          leuart_sm_write.leuart->TXDATA = leuart_sm_write.string[leuart_sm_write.count];
          leuart_sm_write.count++;
      } else {
          leuart_sm_write.current_state = END;
          leuart_sm_write.leuart->IEN |= LEUART_IEN_TXC;
          leuart_sm_write.leuart->IEN &= ~LEUART_IEN_TXBL;
      }
      break;
    case END:
      EFM_ASSERT(false);
      break;
    default:
      EFM_ASSERT(false);
      break;
  }
}

/***************************************************************************//**
 * @brief TXC Interrupt Routine
 *
 * @details Unblocks Sleep Mode
 *          Signals Callback Event
 *          Sets busy to false
 *
 ******************************************************************************/
void leuart_txc(void) {
  switch (leuart_sm_write.current_state) {
    case WRITE_DATA:
      EFM_ASSERT(false);
      break;
    case END:
      leuart_sm_write.busy = false;
      leuart_sm_write.leuart->IEN &= ~LEUART_IEN_TXC;
      break;
    default:
      EFM_ASSERT(false);
      break;
  }

}

/***************************************************************************//**
 * @brief RXDATAV Interrupt Routine
 *
 * @details Reads data from RX Buffer while there are still more characters to
 *          be read
 *          Does not read startframe or sigframe
 *
 ******************************************************************************/
void leuart_rxdatav(void) {
  switch (leuart_sm_read.current_state) {
    case WRITE_DATA:
      if ((leuart_sm_read.leuart->RXDATA != '!') & (leuart_sm_read.leuart->RXDATA != '#')) {
        result[leuart_sm_read.count] = leuart_sm_read.leuart->RXDATA;
        leuart_sm_read.count++;
      }

      break;
    case END:
      EFM_ASSERT(false);
      break;
    default:
      EFM_ASSERT(false);
      break;
  }
}

/***************************************************************************//**
 * @brief STARTF Interrupt Routine
 *
 * @details Waits for start frame, and reads it
 *
 ******************************************************************************/
void leuart_startf(void) {
  switch (leuart_sm_read.current_state) {
    case WRITE_DATA:
      result[leuart_sm_read.count] = leuart_sm_read.leuart->RXDATA;
      leuart_sm_read.count++;
      break;
    case END:
      EFM_ASSERT(false);
      break;
    default:
      EFM_ASSERT(false);
      break;
  }
}

/***************************************************************************//**
 * @brief SIGF Interrupt Routine
 *
 * @details Waits for Sigframe and reads it
 *          Re-enables RXBLOCK
 *          Adds scheduled callback
 *
 ******************************************************************************/
void leuart_sigf(void) {
  switch (leuart_sm_read.current_state) {
      case WRITE_DATA:
          result[leuart_sm_read.count] = leuart_sm_read.leuart->RXDATA;
          leuart_sm_read.count++;
          leuart_sm_read.busy = false;
          leuart_sm_read.leuart->CMD |= LEUART_CMD_RXBLOCKEN;
          while (leuart_sm_read.leuart->SYNCBUSY & LEUART_SYNCBUSY_CMD);
          add_scheduled_event(leuart_sm_read.sleep_mode_event);
        break;
      case END:
        EFM_ASSERT(false);
        break;
      default:
        EFM_ASSERT(false);
        break;
    }
}

/***************************************************************************//**
 * @brief Starts leuart
 *
 * @details Initilaizes global struct containing string and status information
 *          Enables TXBL Interrupt
 *
 * @note Uses atomic state
 *
 ******************************************************************************/

void leuart_start(LEUART_TypeDef *leuart, char *string, uint32_t string_len, uint32_t cb){
  while(leuart_sm_write.busy);
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

//  sleep_block_mode(LEUART_TX_EM<LEUART_RX_EM ? LEUART_TX_EM : LEUART_RX_EM);


  leuart_sm_write.current_state = WRITE_DATA;
  leuart_sm_write.busy = true;

  leuart_sm_write.sleep_mode_event = cb;

  strcpy(leuart_sm_write.string, string);
  leuart_sm_write.string_len = string_len;
  leuart_sm_write.leuart = leuart;
  leuart_sm_write.count = 0;

  leuart_sm_read.current_state = WRITE_DATA;
  leuart_sm_read.busy = true;
  leuart_sm_read.sleep_mode_event = cb;
  strcpy(leuart_sm_read.string, string);
  leuart_sm_read.string_len = string_len;
  leuart_sm_read.leuart = leuart;
  leuart_sm_read.count = 0;

  //leuart->IEN |= LEUART_IEN_TXBL | LEUART_IEN_RXDATAV | LEUART_IEN_STARTF | LEUART_IEN_SIGF;
  //leuart->IEN |= LEUART_IEN_TXBL | LEUART_IEN_STARTF | LEUART_IEN_SIGF;
  leuart->IEN |= LEUART_IEN_TXBL;


  CORE_EXIT_CRITICAL();
}

/***************************************************************************//**
 * @brief Does nothing
 *
 ******************************************************************************/

//bool leuart_tx_busy(LEUART_TypeDef *leuart){
//
//}

/***************************************************************************//**
 * @brief Transfers data from read operation to other files
 *
 * @details Copies result string to input string
 *          Resets result string
 *
 ******************************************************************************/
void receive_data(char *very_stupid_string) {

  strcpy(very_stupid_string, result);

  for (int i=0; i < (int) strlen(result); i++) {
      result[i] = '\0';
  }
}

/***************************************************************************//**
 * @brief
 *   LEUART STATUS function returns the STATUS of the peripheral for the
 *   TDD test
 *
 * @details
 * 	 This function enables the LEUART STATUS register to be provided to
 * 	 a function outside this .c module.
 *
 * @param[in] *leuart
 *   Defines the LEUART peripheral to access.
 *
 * @return
 * 	 Returns the STATUS register value as an uint32_t value
 *
 ******************************************************************************/

uint32_t leuart_status(LEUART_TypeDef *leuart){
	uint32_t	status_reg;
	status_reg = leuart->STATUS;
	return status_reg;
}

/***************************************************************************//**
 * @brief
 *   LEUART CMD Write sends a command to the CMD register
 *
 * @details
 * 	 This function is used by the TDD test function to program the LEUART
 * 	 for the TDD tests.
 *
 * @note
 *   Before exiting this function to update  the CMD register, it must
 *   perform a SYNCBUSY while loop to ensure that the CMD has by synchronized
 *   to the lower frequency LEUART domain.
 *
 * @param[in] *leuart
 *   Defines the LEUART peripheral to access.
 *
 * @param[in] cmd_update
 * 	 The value to write into the CMD register
 *
 ******************************************************************************/

void leuart_cmd_write(LEUART_TypeDef *leuart, uint32_t cmd_update){

	leuart->CMD = cmd_update;
	while(leuart->SYNCBUSY);
}

/***************************************************************************//**
 * @brief
 *   LEUART IF Reset resets all interrupt flag bits that can be cleared
 *   through the Interrupt Flag Clear register
 *
 * @details
 * 	 This function is used by the TDD test program to clear interrupts before
 * 	 the TDD tests and to reset the LEUART interrupts before the TDD
 * 	 exits
 *
 * @param[in] *leuart
 *   Defines the LEUART peripheral to access.
 *
 ******************************************************************************/

void leuart_if_reset(LEUART_TypeDef *leuart){
	leuart->IFC = 0xffffffff;
}

/***************************************************************************//**
 * @brief
 *   LEUART App Transmit Byte transmits a byte for the LEUART TDD test
 *
 * @details
 * 	 The BLE module will respond to AT commands if the BLE module is not
 * 	 connected to the phone app.  To validate the minimal functionality
 * 	 of the LEUART peripheral, write and reads to the LEUART will be
 * 	 performed by polling and not interrupts.
 *
 * @note
 *   In polling a transmit byte, a while statement checking for the TXBL
 *   bit in the Interrupt Flag register is required before writing the
 *   TXDATA register.
 *
 * @param[in] *leuart
 *   Defines the LEUART peripheral to access.
 *
 * @param[in] data_out
 *   Byte to be transmitted by the LEUART peripheral
 *
 ******************************************************************************/

void leuart_app_transmit_byte(LEUART_TypeDef *leuart, uint8_t data_out){
	while (!(leuart->IF & LEUART_IF_TXBL));
	leuart->TXDATA = data_out;
}


/***************************************************************************//**
 * @brief
 *   LEUART App Receive Byte polls a receive byte for the LEUART TDD test
 *
 * @details
 * 	 The BLE module will respond to AT commands if the BLE module is not
 * 	 connected to the phone app.  To validate the minimal functionality
 * 	 of the LEUART peripheral, write and reads to the LEUART will be
 * 	 performed by polling and not interrupts.
 *
 * @note
 *   In polling a receive byte, a while statement checking for the RXDATAV
 *   bit in the Interrupt Flag register is required before reading the
 *   RXDATA register.
 *
 * @param[in] leuart
 *   Defines the LEUART peripheral to access.
 *
 * @return
 * 	 Returns the byte read from the LEUART peripheral
 *
 ******************************************************************************/

uint8_t leuart_app_receive_byte(LEUART_TypeDef *leuart){
	uint8_t leuart_data;
	while (!(leuart->IF & LEUART_IF_RXDATAV));
	leuart_data = leuart->RXDATA;
	return leuart_data;
}
