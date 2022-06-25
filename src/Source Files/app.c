/**
 * @file app.c
 * @author KD
 * @date 9/7/21
 * @brief Initializes peripherals and pwm
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "app.h"


//***********************************************************************************
// defined files
//***********************************************************************************
//#define BLE_TEST_ENABLED
#define RX_TDD

//***********************************************************************************
// Private variables
//***********************************************************************************
//static int led_color_variable=0;
  static uint32_t x = 3;
  static float y = 0;
  static char stupid_str[80];
//***********************************************************************************
// Private functions
//***********************************************************************************

static void app_letimer_pwm_open(float period, float act_period, uint32_t out0_route, uint32_t out1_route);

//***********************************************************************************
// Global functions
//***********************************************************************************

/***************************************************************************//**
 * @brief Initializes the micro-controller
 *
 *
 * @details Initializes cmu, gpio, interrupt scheduler, sleep mode levels
 * Initializes a new letimer pwm struct and passes parameters and initialzes
 * the RGB LED
 * Start LETIMER0 and letimer PWM
 * Calls si1133 to open
 *
 *
 * @note
 *
 *
 ******************************************************************************/
void app_peripheral_setup(void){
  cmu_open(); // Must be first
  gpio_open();
  scheduler_open(); // Initializes static variable for events scheduled
  sleep_open(); // Set array of lowest energy modes to 0
  rgb_init(); // Clear LED Pins

  app_letimer_pwm_open(PWM_PER, PWM_ACT_PER, PWM_ROUTE_0, PWM_ROUTE_1);

  si1133_i2c_open();

  // Lab 6
  add_scheduled_event(BOOT_UP_CB);
  ble_open(rx_event, tx_event);
  sleep_block_mode(SYSTEM_BLOCK_EM);
}

/*******************************************************************************
 * @brief Creates an instance of a struct that holds information about routing
 *
 *
 * @details Contains debug states, pin routing, and periods
 *
 *
 * @note
 *
 *
 * @param[in] period
 * float for period of LED in off state
 *
 *
 * @param[in] act_period
 * float for period of LED in on (active) state
 *
 * @param[in] out0_route
 * routes to gpio out pin 0
 *
 * @param[in] out1_route
 * routes to gpio out pin 1
 *
 *
 ******************************************************************************/
void app_letimer_pwm_open(float period, float act_period, uint32_t out0_route, uint32_t out1_route){
  // Initializing LETIMER0 for PWM operation by creating the
  // letimer_pwm_struct and initializing all of its elements
  // APP_LETIMER_PWM_TypeDef is defined in letimer.h
  APP_LETIMER_PWM_TypeDef   letimer_pwm_struct;
  letimer_pwm_struct.debugRun = false;
  letimer_pwm_struct.enable = false;
  letimer_pwm_struct.out_pin_route0 = out0_route;
  letimer_pwm_struct.out_pin_route1 = out1_route;
  letimer_pwm_struct.out_pin_0_en = false;
  letimer_pwm_struct.out_pin_1_en = false;
  letimer_pwm_struct.period = period;
  letimer_pwm_struct.active_period = act_period;
  //EFM_ASSERT(false);

  letimer_pwm_struct.comp0_irq_enable = false;
  letimer_pwm_struct.comp0_cb = LETIMER0_COMP0_CB;
  letimer_pwm_struct.comp1_irq_enable = true;
  letimer_pwm_struct.comp1_cb = LETIMER0_COMP1_CB;
  letimer_pwm_struct.uf_irq_enable = true;
  letimer_pwm_struct.uf_cb = LETIMER0_UF_CB;

  letimer_pwm_open(LETIMER0, &letimer_pwm_struct);

}
/*******************************************************************************
 * @brief Sends values to print to BLE connected device
 *
 *
 * @details Calculates values for z to be printed
 * Sends read commands to si1133 light sensor
 *
 *
 * @note
 * Previously incremented the next light to be turned it
 *
 *
 ******************************************************************************/
void scheduled_letimer0_uf_cb(void) {
  EFM_ASSERT(!(get_scheduled_events() & LETIMER0_UF_CB));

  /*

  if (led_color_variable == 0) {
      leds_enabled(RGB_LED_1, COLOR_RED, false);
      led_color_variable++;
  } else if (led_color_variable == 1) {
      leds_enabled(RGB_LED_1, COLOR_GREEN, false);
      led_color_variable++;
  } else if (led_color_variable == 2) {
      leds_enabled(RGB_LED_1, COLOR_BLUE, false);
      led_color_variable = 0;
  }
  */

  pass_sensing_result(I2C1, SI1133_LIGHT_READ_CB);


  float z;
  x = x + 3;
  y = y + 1;
  z = x / y;

  char str[20];
  sprintf(str, "\nz = %1.3f\n", z);
  ble_write(str, LEUART_READ_CB);

}
/*******************************************************************************
 * @brief Asserts false
 *
 *
 * @details EFM_ASSERT(false)
 *
 *
 * @note
 * This function was built such that it should not yet be used
 *
 *
 ******************************************************************************/
void scheduled_letimer0_comp0_cb(void) {
  //EFM_ASSERT(false);
  //EFM_ASSERT(!(get_scheduled_events() & LETIMER0_COMP0_CB));
}
/*******************************************************************************
 * @brief calls send_force_cmd
 *
 *
 * @details Calls to send FORCE command to si1133
 *
 *
 * @note
 * Previously turned on LED
 *
 *
 ******************************************************************************/
void scheduled_letimer0_comp1_cb(void) {
  EFM_ASSERT(!(get_scheduled_events() & LETIMER0_COMP1_CB));
  /*
  if (led_color_variable == 0) {leds_enabled(RGB_LED_1, COLOR_RED, true);}
  if (led_color_variable == 1) {leds_enabled(RGB_LED_1, COLOR_GREEN, true);}
  if (led_color_variable == 2) {leds_enabled(RGB_LED_1, COLOR_BLUE, true);}
  */

  //si1133_read(I2C1,SI1133_LIGHT_READ_CB, ONE_BYTE);
  send_force_cmd(I2C1);
}

/*******************************************************************************
 * @brief Retrieves light read data from si1133
 *
 *
 * @details If the light data is less than 20, blue LED turns on
 * Otherwise blue LED turns off
 *
 * @note
 *
 ******************************************************************************/
void si1133_light_read_call_back(void) {
/*
  EFM_ASSERT(!(get_scheduled_events() & SI1133_LIGHT_READ_CB));
  if (si1133_pass_read_result() == SI1133_PART_ID) {
      leds_enabled(RGB_LED_1, COLOR_GREEN, true);
  } else {
      leds_enabled(RGB_LED_1, COLOR_RED, true);
  }
*/

  uint32_t result = si1133_pass_read_result();
  if (result < 20) {leds_enabled(RGB_LED_1, COLOR_BLUE, true);}
  else {leds_enabled(RGB_LED_1, COLOR_BLUE, false);}
}


//Lab 6
/*******************************************************************************
 * @brief Starts Letimer0 and writes "Hello World" to connected BLE device
 *
 *
 * @details Uses ble_test() to change the name of the peripheral if
 *          BLE_TEST_ENABLED is defined
 * Uses ble_write to print Hello WOrld
 *
 * @note
 *
 ******************************************************************************/
void scheduled_boot_up_cb(void) {
  EFM_ASSERT(!(get_scheduled_events() & BOOT_UP_CB));

  letimer_start(LETIMER0, true);  //This command will initiate the start of the LETIMER0


  #ifdef BLE_TEST_ENABLED
  char* mod_name = "KD_BLE";
  EFM_ASSERT(ble_test(mod_name));
  timer_delay(TWO_SECONDS_MS);
  #endif
  ble_write("\nHello World\n", BLE_TX_DONE_CB);


  #ifdef RX_TDD

  #endif

}

/*******************************************************************************
 * @brief Call back for bluetooth write
 *
 *
 * @details Does nothing
 *
 * @note
 *
 ******************************************************************************/
void ble_tx_done_cb(void) {

}

/*******************************************************************************
 * @brief Call back for LEUART read operation
 *
 *
 * @details Requests read result, if it contains the command "#U+xyz" to increase
 *          or "U-xyz" to decrease, where xyz is change in letimer period
 *          and sends command to letimer
 *
 * @note string comes in as ASCII value, so it 0x30 is subtracted to find
 *       correct number
 *
 ******************************************************************************/
void leuart_read_cb(void) {
  int change;
  receive_data(stupid_str);

  if (stupid_str[2] == '+') {
      change = ((stupid_str[3]-0x30)*100) + ((stupid_str[4]-0x30)*10) + (stupid_str[5]-0x30);
      change_LETIMER_period(LETIMER0, change);
  } else if (stupid_str[2] == '-') {
      change = -1 * (((stupid_str[3]-0x30)*100) + ((stupid_str[4]-0x30)*10) + (stupid_str[5]-0x30));
      change_LETIMER_period(LETIMER0, change);
  }
}
