//***********************************************************************************
// Include files
//***********************************************************************************
#ifndef HEADER_FILES_I2C_H_
#define HEADER_FILES_I2C_H_

/* System include statements */


/* Silicon Labs include statements */
#include "em_assert.h"
#include "em_i2c.h"
#include "em_cmu.h"

/* The developer's include statements */
//#include "Si1133.h"
#include "sleep_routines.h"
#include "scheduler.h"
#include "si1133.h"


//***********************************************************************************
// defined files
//***********************************************************************************
#define I2C_EM_BLOCK EM2
#define WRITE 0
#define READ 1
//***********************************************************************************
// global variables
//***********************************************************************************
typedef struct {
  I2C_TypeDef *i2c;

  uint32_t                    i2c_sda_pin;
  uint32_t                    i2c_scl_pin;
  bool                        i2c_scl_pin_en;
  bool                        i2c_sda_pin_en;
  bool                        enable;
  bool                        master;
  uint32_t                    refFreq;
  uint32_t                    freq;
  I2C_ClockHLR_TypeDef        clhr;

} I2C_OPEN_STRUCT ;


typedef enum {
    INITIALIZATION,
    WRITE_TO_SLAVE,
    READ_FROM_SLAVE,
    RECEIVE_DATA,
    STOP

} DEFINED_STATES ;

typedef struct {
    DEFINED_STATES current_state;
    uint32_t device_address;
    uint32_t slave_address;
    uint32_t register_address;
    bool command; // write or read
    volatile bool busy;
    uint32_t* buffer; // store read result or get write data
    uint32_t byte_count;
    uint32_t sleep_mode_event;
    I2C_TypeDef* i2c;


} I2C_STATE_MACHINE ;




//***********************************************************************************
// function prototypes
//***********************************************************************************

void i2c_open(I2C_TypeDef *I2C_TypeDef, I2C_OPEN_STRUCT *I2C_open);
void I2C0_IRQHandler(void);
void I2C1_IRQHandler(void);
void i2c_start(I2C_TypeDef *i2c,uint32_t cb,uint32_t byte_count,uint32_t slave_address,uint32_t register_address,bool cmd,uint32_t *read_result);
bool isBusy(I2C_TypeDef *i2c);

#endif /* HEADER_FILES_I2C_H_ */
