/**
 * @file i2c.c
 * @author KD
 * @date 9/28/21
 * @brief I2C peripherals
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "i2c.h"

//***********************************************************************************
// defined files
//***********************************************************************************
static void i2c_bus_reset(I2C_TypeDef *i2c);
static void i2c_ack_sm(I2C_STATE_MACHINE *i2c_sm);
static void stop_interrupt_func(I2C_STATE_MACHINE *i2c_sm);
static void rxdatav_interrupt(I2C_STATE_MACHINE *i2c_sm);
//***********************************************************************************
// global variables
//***********************************************************************************
static I2C_STATE_MACHINE i2c0_sm;
static I2C_STATE_MACHINE i2c1_sm;


/***************************************************************************//**
   * @brief Configures I2C in preparation for state machine implementation
   *
   *
   *
   * @details Enables proper clock depending on peripheral,
   * initializes local struct,
   * routes pins,
   * and enables interrupts
   *
   *
   * @note
   *
   * @param[in] I2C_TypeDef *i2c
   *  Passes in library defined type in order to enable functionality for I2C1 and
   *  I2C0
   *
   * @param[in] I2C_OPEN_STRUCT *i2c_setup
   *  Passes in type I2C_OPEN_STRUCT for initialization
 ******************************************************************************/
void i2c_open(I2C_TypeDef *i2c, I2C_OPEN_STRUCT *i2c_setup) {
  if (i2c == I2C0) {
      CMU_ClockEnable(cmuClock_I2C0, true);
      i2c0_sm.busy = false;
  } else if (i2c == I2C1) {
      CMU_ClockEnable(cmuClock_I2C1, true);
      i2c1_sm.busy = false;
  } else {
      EFM_ASSERT(false);
  }

  if((i2c->IF & 0x01) == 0) {
      i2c->IFS = 0x01;
      EFM_ASSERT(i2c->IF & 0x01);
      i2c->IFC = 0x01;
  } else {
      i2c->IFC = 0x01;
      EFM_ASSERT(!(i2c->IF & 0x01));
  }


  I2C_Init_TypeDef i2c_values;

  i2c_values.enable = i2c_setup->enable;
  i2c_values.master = i2c_setup->master;
  i2c_values.refFreq = i2c_setup->refFreq;
  i2c_values.freq = i2c_setup->freq;
  i2c_values.clhr = i2c_setup->clhr;

  EFM_ASSERT(i2c_values.freq != 0);

  I2C_Init(i2c, &i2c_values);
  //i2c->ROUTEPEN = (I2C_ROUTEPEN_SCLPEN * i2c_setup->i2c_scl_pin_en) | (I2C_ROUTEPEN_SDAPEN * i2c_setup->i2c_sda_pin_en);
  //i2c->ROUTELOC0 = i2c_setup->i2c_scl_pin | i2c_setup->i2c_sda_pin;
 // i2c->ROUTELOC0 = I2C_ROUTELOC0_SCLLOC_LOC17 & I2C_ROUTELOC0_SDALOC_LOC17;
  i2c->ROUTELOC0 = i2c_setup->i2c_scl_pin | i2c_setup->i2c_sda_pin;
  i2c->ROUTEPEN = (I2C_ROUTEPEN_SCLPEN * i2c_setup->i2c_scl_pin_en) | (I2C_ROUTEPEN_SDAPEN * i2c_setup->i2c_sda_pin_en); //STILL UNSURE
// May be wrong ^

  i2c_bus_reset(i2c);

  i2c->IFC = I2C_IFC_ACK | I2C_IFC_MSTOP;
  i2c->IEN = I2C_IEN_ACK | I2C_IEN_MSTOP | I2C_IEN_RXDATAV;


  if(i2c == I2C0) {NVIC_EnableIRQ(I2C0_IRQn);}
  if(i2c == I2C1) {NVIC_EnableIRQ(I2C1_IRQn);}
}

/***************************************************************************//**
 * @brief Checks whether i2c is busy
 *
 *
 * @details Returns if the i2c is busy
 *
 *
 * @note
 *
 * @param[in] I2C_TypeDef *i2c
 * Struct of I2C_TypeDef denoting the I2C peripheral
 *
 * @param[out] bool
 * True if i2c is busy, false otherwise
 ******************************************************************************/
bool isBusy(I2C_TypeDef *i2c) {
  if (i2c == I2C1) {
      return i2c1_sm.busy;
  } else {
      return i2c0_sm.busy;
  }
  EFM_ASSERT(false);
}


//***********************************************************************************
// functions
//***********************************************************************************

/***************************************************************************//**
   * @brief Resets state machine for new read or write command
   *
   *
   *
   * @details Disables interrupts, clears interrupts, clears transmit buffer,
   * resets state machine, and resets state of interrupt enable register to temporary
   * variable initialized at beginning of the function
   *
   *
   * @note i2c_IEN is temporary variable to save state of Interrupt Enable Register
   *
   *@param[in] I2C_TypeDef *i2c
   *  Passes in library defined type in order to enable functionality for I2C1 and
   *  I2C0
   ******************************************************************************/
static void i2c_bus_reset(I2C_TypeDef *i2c){
  I2C_IntClear(i2c, I2C_IFC_ACK | I2C_IFC_NACK | I2C_IFC_MSTOP);
  I2C_IntEnable(i2c, I2C_IEN_ACK | I2C_IEN_RXDATAV | I2C_IEN_MSTOP);

  i2c->CMD = I2C_CMD_ABORT;

  uint32_t i2c_IEN;
  i2c_IEN = i2c->IEN; // temp storage of IEN
  i2c->IEN &= ~i2c_IEN; //clearing IEN interrupts
  i2c->IFC = i2c->IF;// clearing IF register before reset
  //EFM_ASSERT(i2c->IFC == 0);
  i2c->TXDATA &= ~i2c->TXDATA; // Clearing I2C transmit buffer

  i2c->CMD |= I2C_CMD_CLEARTX;       // Reset operation sequence
  i2c->CMD |= I2C_CMD_START | I2C_CMD_STOP;
  while(!(i2c->IF & I2C_IF_MSTOP)); // Check for reset operation complete

  i2c->IFC = i2c->IF;
  i2c->CMD |= I2C_CMD_ABORT; //Lastly, reset the micro-controller I2C peripheral
  //state machine itself which is accomplished by writing the ABORT bit in the I2C CMD register
  i2c->IEN |= i2c_IEN;
}

/***************************************************************************//**
   * @brief Uses conditionals to initiate calls to correct interrupt functions
   *
   *
   *
   * @details Checks if interrupt flag is true, interrupts are enabled, and which interrupt
   * was received, then calls correct function
   *
   *
   * @note This implementation is for I2C0
   *
   ******************************************************************************/
void I2C0_IRQHandler(void) {
    uint32_t int_flag;
    int_flag = I2C0->IF & I2C0->IEN;
    I2C0->IFC = int_flag;

   if (int_flag & I2C_IF_ACK) {
       EFM_ASSERT(!(I2C0->IF & I2C_IF_ACK));
       i2c_ack_sm(&i2c0_sm);
   }
   if (int_flag & I2C_IF_RXDATAV) {
       rxdatav_interrupt(&i2c0_sm);
   }
   if (int_flag & I2C_IF_MSTOP) {
       EFM_ASSERT(!(I2C0->IF & I2C_IF_MSTOP));
       stop_interrupt_func(&i2c0_sm);
   }

}

/***************************************************************************//**
   * @brief Uses conditionals to initiate calls to correct interrupt functions
   *
   *
   *
   * @details Checks if interrupt flag is true, interrupts are enabled, and which interrupt
   * was received, then calls correct function
   *
   *
   * @note This implementation is for I2C1
   *
   ******************************************************************************/
void I2C1_IRQHandler(void) {

    uint32_t int_flag;
    int_flag = I2C1->IF & I2C1->IEN;
    I2C1->IFC = int_flag;

    if (int_flag & I2C_IF_ACK) {
        EFM_ASSERT(!(I2C1->IF & I2C_IF_ACK));
        i2c_ack_sm(&i2c1_sm);
    }
    if (int_flag & I2C_IF_RXDATAV) {
        rxdatav_interrupt(&i2c1_sm);
    }
    if (int_flag & I2C_IF_MSTOP) {
        EFM_ASSERT(!(I2C1->IF & I2C_IF_MSTOP));
        stop_interrupt_func(&i2c1_sm);
    }

}

/***************************************************************************//**
   * @brief Initializes conditions before flowchart is iterated through
   *
   * @details Initializes state machine struct and configures settings for Initialize state
   * which is the first in the flowchart
   *
   *
   * @note
   *
   * @param[in] I2C_TypeDef *i2c
   *  Required to determine if using I2C0 or I2C1
   *
   *  @param[in] uint32_t byte_count
   *  Parameter used to check if there is data left to read in RXDATA
   *
   * @param[in] uint32_t cb
   *  Call back event to be loaded into sleep_mode_event state machine struct element
   *
   * @param[in] uint32_t request_address
   *  Address of SI1133
   *
   *  @param[in] device_address
   *  PART_ID for SI1133
   *
   * @param[in] uint32_t* result
   *  address of si1133_read_result
   ******************************************************************************/

void i2c_start(I2C_TypeDef *i2c,uint32_t cb,uint32_t byte_count,uint32_t slave_address,uint32_t register_address,bool cmd,uint32_t *read_result) {


    if (i2c == I2C0) {

        while(i2c0_sm.busy);
        EFM_ASSERT((I2C0->STATE & _I2C_STATE_STATE_MASK) == I2C_STATE_STATE_IDLE);
        sleep_block_mode(I2C_EM_BLOCK);

        i2c0_sm.busy = true;
        i2c0_sm.i2c = i2c;
        i2c0_sm.current_state = INITIALIZATION;
        i2c0_sm.slave_address = slave_address;
        i2c0_sm.command = cmd;
        i2c0_sm.byte_count = byte_count;
        i2c0_sm.sleep_mode_event = cb;
        i2c0_sm.register_address = register_address;
        i2c0_sm.buffer = read_result;

        i2c->CMD = I2C_CMD_START;
        i2c->TXDATA = (i2c0_sm.slave_address << 1) | WRITE;

    } else if (i2c == I2C1) {

        while(i2c1_sm.busy);
        EFM_ASSERT((I2C1->STATE & _I2C_STATE_STATE_MASK) == I2C_STATE_STATE_IDLE);
        sleep_block_mode(I2C_EM_BLOCK);

        i2c1_sm.busy = true;
        i2c1_sm.i2c = i2c;
        i2c1_sm.current_state = INITIALIZATION;
        i2c1_sm.slave_address = slave_address;
        i2c1_sm.command = cmd;
        i2c1_sm.byte_count = byte_count;
        i2c1_sm.sleep_mode_event = cb;
        i2c1_sm.register_address = register_address;
        i2c1_sm.buffer = read_result;

        i2c->CMD = I2C_CMD_START;
        i2c->TXDATA = (i2c1_sm.slave_address << 1) | WRITE;

    } else {

        EFM_ASSERT(false);

    }

}


/***************************************************************************//**
   * @brief Handles state changes and commands for ACK interrupt
   *
   *
   *
   * @details Sets appropriate commands and performs state changes depending on current state
   *
   *
   * @note Sets read bit
   *
   * @param[in] I2C_STATE_MACHINE *i2c_sm
   *  Needed in order to change states and define commands
   ******************************************************************************/

void i2c_ack_sm(I2C_STATE_MACHINE *i2c_sm) {
  switch(i2c_sm->current_state) {
      case INITIALIZATION:  // Progresses State Machine

        i2c_sm->current_state = WRITE_TO_SLAVE;
        i2c_sm->i2c->TXDATA = (i2c_sm->register_address);

        break;
      case WRITE_TO_SLAVE: // Progresses State Machine

        if (i2c_sm->command == READ) {
          i2c_sm->current_state = READ_FROM_SLAVE;
          i2c_sm->i2c->CMD = I2C_CMD_START;
          //i2c_sm->command = READ;
          i2c_sm->i2c->TXDATA = i2c_sm->slave_address << 1 | READ;
        } else {
            if (i2c_sm->byte_count == 0) {
                i2c_sm->current_state = STOP;
                i2c_sm->i2c->CMD = I2C_CMD_STOP;
            } else {
                i2c_sm->i2c->TXDATA = *(i2c_sm->buffer) << (8*((i2c_sm->byte_count)-1));
                i2c_sm->byte_count--;
            }
        }


        break;
      case READ_FROM_SLAVE:  // Progresses State Machine

        i2c_sm->current_state = RECEIVE_DATA;

        break;
      case RECEIVE_DATA:  // Incorrect if ACK is called here
        //Nothing
        break;
      case STOP:  // Incorrect if ACK is called here
        EFM_ASSERT(false);
        break;
      default:  // Base Case
        EFM_ASSERT(false);
        break;
  }
}

/***************************************************************************//**
   * @brief Handles state changes and commands for RXDATAV interrupt
   *
   *
   *
   * @details Reads data into buffer element within state machine struct by iterating
   * through bits one by one using byte_count. State is changed to stop, all other instances
   * assert false.
   *
   *
   * @note
   *
   * @param[in] I2C_STATE_MACHINE *i2c_sm
   *  Needed in order to change states and define commands
   ******************************************************************************/

void rxdatav_interrupt(I2C_STATE_MACHINE *i2c_sm) {
  switch(i2c_sm->current_state) {
      case INITIALIZATION:  // Incorrect if RXDATAV is called here
        EFM_ASSERT(false);
        break;
      case WRITE_TO_SLAVE:  // Incorrect if RXDATAV is called here
        //EFM_ASSERT(false);  //stuck
        break;
      case READ_FROM_SLAVE:  // Incorrect if RXDATAV is called here
       EFM_ASSERT(false);
       break;
      case RECEIVE_DATA:  // Progresses if data is not valid
        if (i2c_sm->command == READ) {
        *(i2c_sm->buffer) |= i2c_sm->i2c->RXDATA << (8*((i2c_sm->byte_count)-1));
        i2c_sm->byte_count--;
        if (i2c_sm->byte_count > 0) {i2c_sm->i2c->CMD = I2C_CMD_ACK;}
          if (i2c_sm->byte_count == 0) {
              i2c_sm->current_state = STOP;
              i2c_sm->i2c->CMD = I2C_CMD_NACK;
              i2c_sm->i2c->CMD = I2C_CMD_STOP;
          }
        }
        break;
      case STOP:  // Incorrect if RXDATAV is called here
        EFM_ASSERT(false);
        break;
      default:  // Base Case
        EFM_ASSERT(false);
        break;
  }
}

/***************************************************************************//**
   * @brief Handles state changes and commands for MSTOP interrupt
   *
   *
   *
   * @details Sets appropriate commands and performs state changes depending on current state
   *
   *
   * @note Unblocks sleep mode, adds scheduled call back event and frees up state machine
   *
   * @param[in] I2C_STATE_MACHINE *i2c_sm
   *  Needed in order to change states and define commands
   ******************************************************************************/


void stop_interrupt_func(I2C_STATE_MACHINE *i2c_sm)
{
  switch(i2c_sm->current_state) {
      case INITIALIZATION:  // Incorrect if MSTOP is called here
        EFM_ASSERT(false);
        break;
      case WRITE_TO_SLAVE:  // Incorrect if MSTOP is called here
        EFM_ASSERT(false);
        break;
      case READ_FROM_SLAVE:  // Incorrect if MSTOP is called here
        EFM_ASSERT(false);
        break;
      case RECEIVE_DATA:  // Incorrect if MSTOP is called here
        EFM_ASSERT(false);
        break;
      case STOP:  // Progresses State Machine

        i2c_sm->busy = false;
        sleep_unblock_mode(I2C_EM_BLOCK);
        add_scheduled_event(i2c_sm->sleep_mode_event);

        break;
      default:  // Base Case
        EFM_ASSERT(false);
        break;
  }
}
