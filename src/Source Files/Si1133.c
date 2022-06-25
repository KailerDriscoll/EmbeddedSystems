/**
 * @file Si1133.c
 * @author KD
 * @date 10/5/21
 * @brief Si1133 sensor Driver
 *
 */


//***********************************************************************************
// Include files
//***********************************************************************************
#include "Si1133.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************

static uint32_t si1133_read_result;
static uint32_t si1133_write_data;


//***********************************************************************************
// functions
//***********************************************************************************

/***************************************************************************//**
 * @brief Sets time delay and initializes local struct to begin I2C setup
 *
 *
 * @details Sets a time delay to wait peripheral powers on
 * Initializes a local I2C_OPEN_STRUCT which is passed to I2C to initialize
 * another struct that sets up to the I2C for communication
 *
 *
 * @note
 *
 *
 ******************************************************************************/
void si1133_i2c_open(void) {
  I2C_OPEN_STRUCT i2c_setup;
  timer_delay(TIME_DELAY_MS);
  i2c_setup.i2c_scl_pin = I2C_ROUTELOC0_SCLLOC_LOC17;
  i2c_setup.i2c_sda_pin = I2C_ROUTELOC0_SDALOC_LOC17;
  i2c_setup.i2c_scl_pin_en = true;
  i2c_setup.i2c_sda_pin_en = true;
  i2c_setup.enable = true;
  i2c_setup.master = true;  // Slave
  i2c_setup.refFreq = 0;  // Only applicable in master mode
  i2c_setup.freq = I2C_FREQ_FAST_MAX;  // Only applicable in master mode
  i2c_setup.clhr = i2cClockHLRAsymetric;


  i2c_open(I2C1, &i2c_setup);
  si1133_configure(I2C1, NULL_CB);
}

/***************************************************************************//**
 * @brief Initializes Local Struct responsible for setting up private structs
 * in the I2C with information to read from the si1133
 *
 *
 * @details Sends information to Initialize an I2C_STATE_MACHINEE that is passed to I2C via i2c_start
 *
 *
 * @param[in] I2C_TypeDef *i2c
 * Struct of I2C_TypeDef denoting the I2C peripheral
 *
 * @param[in] uint32_t reg_address
 * Address of the register to read from
 *
 * @param[in] uint32_t* data
 * Location to store read result
 *
 * @param[in] uint32_t byte_count
 * The number of bytes expected to be read by the si1133 function
 *
 * @param[in] uint32_t cb
 * Callback function to call at the end of the STOP function in I2C
 *
 * @note
 * i2c start is passed a private variable si1133_read_result which is passed by
 * reference so that its value can be changed by I2C to receive the si1133's
 * PORT_ID
 *
 ******************************************************************************/
void si1133_read(I2C_TypeDef* i2c, uint32_t reg_address, uint32_t *data, uint32_t byte_count, uint32_t cb) { //request the read of the Si1133 Part ID
  si1133_read_result = 0;
  i2c_start(i2c, cb, byte_count, SLAVE_ADDRESS, reg_address, READ, data);
}


/***************************************************************************//**
 * @brief Initializes Local Struct responsible for setting up private structs
 * in the I2C that with information to write to the si1133
 *
 *
 * @details Initializes an I2C_STATE_MACHINE that is passed to I2C via i2c_start
 *
 *
 * @param[in] I2C_TypeDef *i2c
 * Struct of I2C_TypeDef denoting the I2C peripheral
 *
 * @param[in] uint32_t reg_address
 * Address of the register to write to
 *
 * @param[in] uint32_t write_data
 * Information to write to the address
 *
 * @param[in] uint32_t byte_count
 * The number of bytes expected to be read by the si1133 function
 *
 * @param[in] uint32_t cb
 * Callback function to call at the end of the STOP function in I2C
 *
 * @note
 *
 ******************************************************************************/
void si1133_write(I2C_TypeDef* i2c, uint32_t reg_address, uint32_t write_data, uint32_t byte_count, uint32_t cb) { //request the read of the Si1133 Part ID
  si1133_write_data = write_data;
  i2c_start(i2c, cb, byte_count, SLAVE_ADDRESS, reg_address, WRITE, &si1133_write_data);
}

/***************************************************************************//**
 * @brief Initializes si1133 to be read to read from the light sensor
 *
 *
 * @details Sends a series of commands to prepare to retrieve data from the
 * light sensor of the si1133
 *
 * @param[in] I2C_TypeDef *i2c
 * Struct of I2C_TypeDef denoting the I2C peripheral
 *
 * @param[in] uint32_t null_cb
 * Callback that is null because it does not need to be referenced later
 *
 * @note
 *
 ******************************************************************************/
void si1133_configure(I2C_TypeDef *i2c, uint32_t null_cb) {
   si1133_read_result = 0; // Reset Read Result
   si1133_write(i2c, COMMAND, RESET_CMD_CTR, ONE_BYTE, null_cb); // Reset

   //---------------------------------------------------------------------------
   // Obtain current command count
   si1133_read(i2c, RESPONSE0, &si1133_read_result, ONE_BYTE, null_cb);
   while(isBusy(i2c));
   uint32_t result = si1133_read_result; // Store RESPONSE0
   uint32_t CMD_CTR = result & CMD_CTR_BITS; // Isolate CMD_CTR
   //---------------------------------------------------------------------------

   //---------------------------------------------------------------------------
   // Write value to set ADCMUX to WHITE in INPUT0 register
   si1133_write(i2c, INPUT0, WHITE_ADCMUX, ONE_BYTE, null_cb);
   while(isBusy(i2c));
   //---------------------------------------------------------------------------

   //---------------------------------------------------------------------------
   // Specify Parameter Table Write command ORed w/ ADCCONFIG0 while writing to COMMAND Register
   //si1133_write(i2c, COMMAND, RESET_CMD_CTR, ONE_BYTE, null_cb); // Reset
   si1133_write(i2c, COMMAND, PARAM_SET | ADCCONFIG0, ONE_BYTE, null_cb);
   while(isBusy(i2c));
   //---------------------------------------------------------------------------

   //---------------------------------------------------------------------------
   // Verify if COMMAND register executed successfully
   si1133_read(i2c, RESPONSE0, &si1133_read_result, ONE_BYTE, null_cb);
   while(isBusy(i2c));
   result = si1133_read_result;
   uint32_t CMD_NEW = result & CMD_CTR_BITS;
   CMD_CTR = (CMD_CTR+1) & CMD_CTR_BITS;
   if (CMD_CTR != CMD_NEW)
   {
       EFM_ASSERT(false); // CMD Write Failed
   }
   //---------------------------------------------------------------------------

   //---------------------------------------------------------------------------
   // Write 0b000001 to INPUT0 to set channel 0 as active
   si1133_write(i2c, INPUT0, CHANNEL0, ONE_BYTE, null_cb);
   while(isBusy(i2c));
   //---------------------------------------------------------------------------

   //---------------------------------------------------------------------------
   //si1133_write(i2c, COMMAND, RESET_CMD_CTR, ONE_BYTE, null_cb); // Reset
   // Specify Parameter Table Write command ORed w/ CHAN_LIST while writing to COMMAND
   si1133_write(i2c, COMMAND, PARAM_SET | CHAN_LIST, ONE_BYTE, null_cb); // What is this supposed to be ORed with
   while(isBusy(i2c));
   //---------------------------------------------------------------------------

   //---------------------------------------------------------------------------
   // Verify if COMMAND register executed successfully
   si1133_read(i2c, RESPONSE0, &si1133_read_result, ONE_BYTE, null_cb);
   while(isBusy(i2c));
   result = si1133_read_result;
   CMD_NEW = result & CMD_CTR_BITS;
   CMD_CTR = (CMD_CTR+1) & CMD_CTR_BITS;
   if (CMD_CTR != CMD_NEW) {EFM_ASSERT(false);}
   //---------------------------------------------------------------------------

}
/***************************************************************************//**
 * @brief Prepared light sensor to be read
 *
 *
 * @details Sends force command to prepare si1133 to be read.
 *
 *
 * @param[in] I2C_TypeDef *i2c
 * Struct of I2C_TypeDef denoting the I2C peripheral
 *
 * @note
 *
 ******************************************************************************/
   void send_force_cmd(I2C_TypeDef* i2c) {
     si1133_write(i2c, COMMAND, FORCE, ONE_BYTE, NULL_CB);
   }

 /***************************************************************************//**
  * @brief Read from si1133
  *
  *
  * @details Reads from HOSTOUT0 register to receive light sensor data
  *
  *
  * @param[in] I2C_TypeDef *i2c
  * Struct of I2C_TypeDef denoting the I2C peripheral
  *
  * @param[in] uint32_t cb
  * Callback to initiate LEDS based on light reading
  *
  * @note
  *
  ******************************************************************************/
   void pass_sensing_result(I2C_TypeDef *i2c, uint32_t cb) {
      si1133_read(i2c, HOSTOUT0, &si1133_read_result, TWO_BYTES, cb);
   }
/***************************************************************************//**
 * @brief Passes a private variable
 *
 *
 * @details Returns the private si1133_read_result for other files to have
 * access to it
 *
 *
 * @param[out] uint32_t si1133_read_result
 * Private variable containing Port information for the si1133 peripheral
 *
 * @note
 *
 *
 ******************************************************************************/
uint32_t si1133_pass_read_result() {
  return si1133_read_result;
}
