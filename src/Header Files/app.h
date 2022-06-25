//***********************************************************************************
// Include files
//***********************************************************************************
#ifndef APP_HG
#define APP_HG

/* System include statements */


/* Silicon Labs include statements */
#include "em_cmu.h"
#include "em_assert.h"
#include <stdio.h>

/* The developer's include statements */
#include "cmu.h"
#include "gpio.h"
#include "letimer.h"
#include "brd_config.h"
#include "sleep_routines.h"
#include "LEDs_thunderboard.h"
#include "Si1133.h"
#include "ble.h"
#include "HW_delay.h"

// Lab 7
#include "leuart.h"


//***********************************************************************************
// defined files
//***********************************************************************************
#define   PWM_PER         2.0   // PWM period in seconds
#define   PWM_ACT_PER     0.002  // PWM active period in seconds

// Application scheduled events
#define LETIMER0_COMP0_CB 0x01  //0b0001
#define LETIMER0_COMP1_CB 0x02  //0b0010
#define LETIMER0_UF_CB    0x04  //0b0100

#define SI1133_REG_READ_CB 0x08
#define SI1133_LIGHT_READ_CB 0x10

#define SI1133_PART_ID 51



// Lab 6
#define BOOT_UP_CB       0x20
#define  SYSTEM_BLOCK_EM  EM3
#define rx_event         0x40
#define tx_event         0x80
#define BLE_TX_DONE_CB   0x100

// Lab 7
#define LEUART_READ_CB   0x400

//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
void app_peripheral_setup(void);
void scheduled_letimer0_uf_cb(void);
void scheduled_letimer0_comp0_cb(void);
void scheduled_letimer0_comp1_cb(void);
void si1133_light_read_call_back(void);
// LAB 6
void scheduled_boot_up_cb(void);
void ble_tx_done_cb(void);

// Lab 7
void leuart_read_cb(void);

#endif
