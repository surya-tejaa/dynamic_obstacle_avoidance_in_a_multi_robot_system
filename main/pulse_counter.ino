//Header Files
#include "driver/pcnt.h"
#include "soc/pcnt_struct.h"

//Pins
#define rightenc 32
#define leftenc 33 //
#define led 2



//Global Variables
//int16_t rightWheel = 0;
//int16_t leftWheel = 0;

volatile byte state_tmr = 0;


//PCNT Parameters
#define PCNT_TEST_UNIT_1 PCNT_UNIT_0
#define PCNT_TEST_UNIT_2 PCNT_UNIT_1
#define PCNT_H_LIM_VAL 30
#define PCNT_L_LIM_VAL 0

void IRAM_ATTR TimerInt()
{
  portENTER_CRITICAL(&mux);

  switch (state_tmr) {

    case 0 :
      //      pcnt_counter_clear(PCNT_TEST_UNIT_1);
      pcnt_counter_resume(PCNT_TEST_UNIT_1);
      //      pcnt_counter_clear(PCNT_TEST_UNIT_2);
      pcnt_counter_resume(PCNT_TEST_UNIT_2);
      state_tmr = 1;
      break;

    case 1 :
      pcnt_counter_pause(PCNT_TEST_UNIT_1);
      pcnt_counter_pause(PCNT_TEST_UNIT_2);
      pcnt_get_counter_value(PCNT_TEST_UNIT_1, &leftWheel);
      pcnt_get_counter_value(PCNT_TEST_UNIT_2, &rightWheel);
      lenc = leftWheel;
      renc = rightWheel;
      leftWheel = 0;
      rightWheel = 0;
      //Serial.println((renc+lenc)/2);
      if (((lenc+renc)/2) >= 13)
      {
        val_ready = 1;
        pcnt_counter_clear(PCNT_TEST_UNIT_1);
        pcnt_counter_resume(PCNT_TEST_UNIT_1);
        pcnt_counter_clear(PCNT_TEST_UNIT_2);
        pcnt_counter_resume(PCNT_TEST_UNIT_2);
      }
      state_tmr = 2;
      break;

    case 2 :
      state_tmr = 0;
      break;

    default:
      break;
  }
  portEXIT_CRITICAL(&mux);
}

void pulse_counter_init()
{
  //Timer Configuration
  timer = timerBegin(0, 80, true); //(Use timer 0,80Mhz crystal thus set divider of 80 -> (1/(80MHz/80)))
  timerAttachInterrupt(timer, &TimerInt, true); //Attach timer function to timer
  timerAlarmWrite(timer, 100000, true); //Set alarm to call timer fucntion every 100 millisecond since 1 tick is 1 microsecond, third parameter is for it to repeat it
  timerAlarmEnable(timer);//enable timer

  //I/O
  pinMode(rightenc, INPUT_PULLUP);
  pinMode(leftenc, INPUT_PULLUP);

  initPcnt();
}

void initPcnt()
{
  pcnt_config_t pcnt_config_l = {
    leftenc, // Pulse input gpio_num, if you want to use gpio16, pulse_gpio_num = 16, a negative value will be ignored
    PCNT_PIN_NOT_USED, // Control signal input gpio_num, a negative value will be ignored
    PCNT_MODE_KEEP, // PCNT low control mode
    PCNT_MODE_KEEP, // PCNT high control mode
    PCNT_COUNT_INC, // PCNT positive edge count mode
    PCNT_COUNT_DIS, // PCNT negative edge count mode
    PCNT_H_LIM_VAL, // Maximum counter value
    PCNT_L_LIM_VAL, // Minimum counter value
    PCNT_TEST_UNIT_1, // PCNT unit number
    PCNT_CHANNEL_0, // the PCNT channel
  };
  pcnt_config_t pcnt_config_r = {
    rightenc, // Pulse input gpio_num, if you want to use gpio16, pulse_gpio_num = 16, a negative value will be ignored
    PCNT_PIN_NOT_USED, // Control signal input gpio_num, a negative value will be ignored
    PCNT_MODE_KEEP, // PCNT low control mode
    PCNT_MODE_KEEP, // PCNT high control mode
    PCNT_COUNT_INC, // PCNT positive edge count mode
    PCNT_COUNT_DIS, // PCNT negative edge count mode
    PCNT_H_LIM_VAL, // Maximum counter value
    PCNT_L_LIM_VAL, // Minimum counter value
    PCNT_TEST_UNIT_2, // PCNT unit number
    PCNT_CHANNEL_0, // the PCNT channel
  };
  if (pcnt_unit_config(&pcnt_config_l) == ESP_OK) //init unit
    Serial.println("Config Unit_0 = ESP_OK");
  if (pcnt_unit_config(&pcnt_config_r) == ESP_OK) //init unit
    Serial.println("Config Unit_0 = ESP_OK");
  pcnt_filter_enable(PCNT_TEST_UNIT_1);

  pcnt_intr_disable(PCNT_TEST_UNIT_1);
  pcnt_event_disable(PCNT_TEST_UNIT_1, PCNT_EVT_L_LIM);
  pcnt_event_disable(PCNT_TEST_UNIT_1, PCNT_EVT_H_LIM);
  pcnt_event_disable(PCNT_TEST_UNIT_1, PCNT_EVT_THRES_0);
  pcnt_event_disable(PCNT_TEST_UNIT_1, PCNT_EVT_THRES_1);
  pcnt_event_disable(PCNT_TEST_UNIT_1, PCNT_EVT_ZERO);

  pcnt_counter_pause(PCNT_TEST_UNIT_1);
  pcnt_counter_clear(PCNT_TEST_UNIT_1);
  pcnt_intr_enable(PCNT_TEST_UNIT_1);
  pcnt_counter_resume(PCNT_TEST_UNIT_1);

  pcnt_filter_enable(PCNT_TEST_UNIT_2);

  pcnt_intr_disable(PCNT_TEST_UNIT_2);
  pcnt_event_disable(PCNT_TEST_UNIT_2, PCNT_EVT_L_LIM);
  pcnt_event_disable(PCNT_TEST_UNIT_2, PCNT_EVT_H_LIM);
  pcnt_event_disable(PCNT_TEST_UNIT_2, PCNT_EVT_THRES_0);
  pcnt_event_disable(PCNT_TEST_UNIT_2, PCNT_EVT_THRES_1);
  pcnt_event_disable(PCNT_TEST_UNIT_2, PCNT_EVT_ZERO);

  pcnt_counter_pause(PCNT_TEST_UNIT_2);
  pcnt_counter_clear(PCNT_TEST_UNIT_2);
  pcnt_intr_enable(PCNT_TEST_UNIT_2);
  pcnt_counter_resume(PCNT_TEST_UNIT_2);
}
