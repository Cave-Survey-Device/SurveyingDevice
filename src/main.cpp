//#define TEST_MODE
#ifdef TEST_MODE
//#include "test.h"
#include "Arduino.h"
//#include "test.h"

void setup()
{
  Serial.begin(9600);
  delay(1000);  
  Serial.print("STARTING BAYBEE\n\n");
}

void loop(){
  delay(1000);  
  Serial.print("FINISHED BAYBEE\n\n");
  Serial.print("FUCK EMBEDDED SOFTWARE 2\n\n");
  //test_main();
}


#else
#include <Arduino.h>
#include "sensors/Sensors.h"
#include "utils/interrupts.h"

enum state_enum {STATE_IDLE, STATE_B1_WAITING, STATE_B2_WAITING, STATE_B1_SHORT, STATE_B2_SHORT, STATE_B1_LONG, STATE_B2_LONG, STATE_B1B2_SHORT, STATE_B1B2_LONG, STATE_B1B2_WAITING};
enum mode_enum {MODE_MENU, MODE_IDLE, MODE_LASER_ENA, MODE_ALIGN, MODE_CALIBRATE};
enum menu_enum {MENU_CALIBRATE, MENU_ALIGN};

float BLE_Enabled = true;
float laser_timeout = 10.0; 
float screen_timeout = 5;
float device_poweroff_timeout = 300;
float long_hold_time = 5;

state_enum previous_state = STATE_IDLE;
state_enum current_state = STATE_IDLE;
state_enum next_state = STATE_IDLE;

mode_enum current_mode = MODE_IDLE;
mode_enum next_mode = MODE_IDLE;
mode_enum previous_mode = MODE_IDLE;

menu_enum menu_selection = MENU_CALIBRATE;

float current_time;
float b1_start_time;
float b2_start_time;
float b1b2_start_time;
SensorHandler sh;

void setup() {

  // put your setup code here, to run once:
}

void MenuForward()
{
  switch(menu_selection)
  {
    case(MENU_CALIBRATE):
    menu_selection = MENU_ALIGN;
    break;

    case(MENU_ALIGN):
    menu_selection = MENU_CALIBRATE;
    break;
  }
}
void MenuBackward()
{
  switch(menu_selection)
  {
    case(MENU_CALIBRATE):
    menu_selection = MENU_ALIGN;
    break;

    case(MENU_ALIGN):
    menu_selection = MENU_CALIBRATE;
    break;
  }
}
void MenuSelect()
{
  switch(menu_selection)
  {
    case(MENU_CALIBRATE):
    next_mode = MODE_CALIBRATE;
    break;

    case(MENU_ALIGN):
    next_mode = MODE_ALIGN;
    break;
  }
}
void MenuExit(){
  next_mode = MODE_IDLE;
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
  next_state = STATE_IDLE;
}

// Enable laser, take shot, align shot, forwards in menu
void StateB1ShortHold(){
  switch(current_mode)
  {
    case(MODE_IDLE):
    next_mode = MODE_LASER_ENA;
    EnableLaser();
    break;

    case(MODE_LASER_ENA):
    next_mode = MODE_IDLE;
    TakeShot();
    break;

    case(MODE_CALIBRATE):
    if (previous_mode != MODE_CALIBRATE)
    {
      sh.ResetCalibration();
    }
    if (sh.CollectCalibrationData())
    {
      sh.CalibrateInertial();
      next_mode = MODE_IDLE;
    }
    break;

    case(MODE_ALIGN):
    if (previous_mode != MODE_ALIGN)
    {
      sh.ResetAlignment();
    }
    if (sh.CollectAlignmentData())
    {
      sh.AlignLaser();
      next_mode = MODE_IDLE;
    }
    break;
    
    case(MODE_MENU):
    MenuForward();
    break;
  }
}

// Select in menu
void StateB1LongHold(){
  switch(current_mode)
  {
    case(MODE_MENU):
    MenuSelect();
    break;
  }
}

// Back in menu
void StateB2ShortHold(){
  switch(current_mode)
  {
    case(MODE_MENU):
    MenuBackward();
    break;
  }
}
void StateB2LongHold(){
  // Return
  switch(current_mode)
  {
    case(MODE_MENU):
    MenuExit();
    break;
  }
  
}

// Enter menu
void StateB1B2ShortHold(){
  next_mode = MODE_MENU;
}

// Reset
void StateB1B2LongHold(){
  next_mode = MODE_IDLE;
  next_state = STATE_IDLE;
  // RESET
}


void loop()
{
  current_time = millis();
  previous_state = current_state;
  current_state = next_state;
  previous_mode = current_mode;
  current_mode = next_mode;
}

#endif