//***********************************************************************************
// Include files
//***********************************************************************************
#ifndef HEADER_FILES_Si1133_H_
#define HEADER_FILES_Si1133_H_

/* System include statements */


/* Silicon Labs include statements */
#include "em_gpio.h"

/* The developer's include statements */
#include "HW_delay.h"
#include "i2c.h"

//***********************************************************************************
// defined files
//***********************************************************************************
//#define I2C_FREQ_STANDARD_MAX 100000   // Ratio is 4:4
//#define I2C_FREQ_FAST_MAX     400000   // Ratio is 6:3
//#define I2C_FREQ_FASTPLUS_MAX 1000000  // Ratio is 11:3

#define TIME_DELAY_MS  25
#define SLAVE_PART_ID  0x33
#define SLAVE_ADDRESS  0x55
#define REG_ADD        0x00
#define ONE_BYTE       1
#define TWO_BYTES      2

// Si1133 Registers
#define RESPONSE0    0x11
#define INPUT0       0x0A
#define COMMAND      0x0B
#define CHAN_LIST    0x01
#define ADCCONFIG0   0x02
#define WHITE_ADCMUX 0x0B //01011, value to set ADCMUX to White Photo Diode
#define PARAM_SET    0x80
#define HOSTOUT0     0x13

#define CMD_CTR_BITS 0x0f
#define CHANNEL0     0b000001
#define NULL_CB      0x00
#define FORCE        0x11

#define RESET_CMD_CTR 0x00

//***********************************************************************************
// private variables
//***********************************************************************************

//***********************************************************************************
// function prototypes
//***********************************************************************************
void si1133_i2c_open(void);
void si1133_read(I2C_TypeDef* i2c, uint32_t reg_address, uint32_t *data, uint32_t byte_count, uint32_t cb);
void si1133_write(I2C_TypeDef* i2c, uint32_t reg_address, uint32_t write_data, uint32_t byte_count, uint32_t cb);
void si1133_configure(I2C_TypeDef *i2c, uint32_t null_cb);
uint32_t si1133_pass_read_result(void);
void send_force_cmd(I2C_TypeDef* i2c);
void pass_sensing_result(I2C_TypeDef *i2c, uint32_t cb);

#endif
