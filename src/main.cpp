#define TEST_MODE
#ifdef TEST_MODE
#include "test.h"

float BLE_Enabled = true;
float laser_timeout = 10.0; 
float screen_timeout = 5;
float device_poweroff_timeout = 300;
float long_hold_time = 5;

void setup()
{
  test_main();
}

void loop(){}

#else
#include <Arduino.h>
#include "sensors/Sensors.h"
#include "utils/interrupts.h"
enum state {STATE_IDLE, STATE_B1_WAITING, STATE_B2_WAITING, STATE_B1_SHORT, STATE_B2_SHORT, STATE_B1_LONG, STATE_B2_LONG, STATE_B1B2_SHORT, STATE_B1B2_LONG, STATE_B1B2_WAITING};

enum b1_enum {B1_ENABLE_LASER, B1_TAKE_SHOT, B1_GET_ALIGN};

enum b1b2_enum
enum menu_enum {}

state previous_state = STATE_IDLE;
state current_state = STATE_IDLE;
state next_state = current_state;
b1_enum b1_mode = B1_ENABLE_LASER;

float current_time;
float b1_start_time;
float b2_start_time;
float b1b2_start_time;
SensorHandler sh;

void setup() {

  // put your setup code here, to run once:
}


void TakeShot(){
  Vector3f shot_data;
  shot_data = sh.get_measurement();
  sh.DisableLaser();
  // TODO: Save shot data
}

void EnableLaser(){
  sh.EnableLaser();
}

void StateB1Waiting(){
  next_state = STATE_B1_WAITING;
  if (previous_state != STATE_B1_WAITING)
  {
    b1_start_time = current_time;
  }
  else if(button1_released_flag && !button2_pressed_flag)
  {
    next_state = STATE_B1_SHORT;
  } 
  else if (button1_pressed_flag && !button2_pressed_flag && current_time > b1_start_time + long_hold_time)
  {
    next_state = STATE_B1_LONG;
  }
  else if (button1_pressed_flag && button2_pressed_flag && current_time >  b1_start_time + long_hold_time)
  {
    next_state = STATE_B1B2_WAITING;
  }
}

void StateB2Waiting(){
  next_state = STATE_B2_WAITING;
  if (previous_state != STATE_B2_WAITING)
  {
    b2_start_time = current_time;
  }
  else if(button2_released_flag && !button1_pressed_flag)
  {
    next_state = STATE_B2_SHORT;
  } 
  else if (button2_pressed_flag && !button1_pressed_flag && current_time > b2_start_time + long_hold_time)
  {
    next_state = STATE_B2_LONG;
  }
  else if (button2_pressed_flag && button1_pressed_flag && current_time >  b2_start_time + long_hold_time)
  {
    next_state = STATE_B1B2_WAITING;
  }
}

void StateB1B2Waiting()
{
  current_time = millis();
  if (previous_state != STATE_B1B2_WAITING)
  {
    b1b2_start_time = current_time;
  }
  else if (button1_released_flag || button2_released_flag)
  {
    next_state = STATE_B1B2_SHORT;
  }
  else if (current_time > b1b2_start_time + long_hold_time)
  {
    next_state = STATE_B1B2_LONG;
  }
}

void StateIdle(){
  previous_state = current_state;
  next_state = STATE_IDLE;
}

void StateB1ShortHold(){
  switch(b1_mode)
  {
    case(B1_ENABLE_LASER):
    b1_mode = B1_TAKE_SHOT;
    EnableLaser();
    break;

    case(B1_TAKE_SHOT):
    b1_mode = B1_ENABLE_LASER;
    TakeShot();
    break;

    case(B1_GET_ALIGN):
    if (sh.CollectAlignmentData())
    {
      sh.AlignLaser();
      b1_mode = B1_ENABLE_LASER;
    }
    break;
  }
}

void StateB1LongHold(){}
void StateB2ShortHold(){}
void StateB2LongHold(){}
void StateB1B2ShortHold(){}
void StateB1B2LongHold(){}


void loop()
{
  current_time = millis();
  current_state = next_state;
}

#endif