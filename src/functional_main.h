#include <ArduinoEigenDense.h>

#include <NumericalMethods_csd.h>
#include <config_csd.h>
#include <interrupts_csd.h>
#include <utility_csd.h>

#include <sensorhandler.h>

#include <SCA3300SensorConnection.h>
#include <RM3100SensorConnection.h>

#include <accelerometer_csd.h>
#include <magnetometer_csd.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <OLED.h>

/* ----------------------------------- OPERATION STATE ENUMS -----------------------------------*/

enum state_enum {
    STATE_IDLE,
    STATE_B1_WAITING,
    STATE_B2_WAITING,
    STATE_B1_SHORT,
    STATE_B2_SHORT,
    STATE_B1_LONG,
    STATE_B2_LONG,
    STATE_B1B2_SHORT,
    STATE_B1B2_LONG,
    STATE_B1B2_WAITING
};

enum mode_enum {
    MODE_MENU,
    MODE_IDLE,
    MODE_LASER_ENA,
    MODE_ALIGN,
    MODE_CALIBRATE
};

enum menu_enum {
    MENU_CALIBRATE,
    MENU_ALIGN,
    MENU_SETTINGS
};
const static int MENU_ENUM_MAX = 2;
inline menu_enum& operator++(menu_enum& menu)
{
    if (static_cast<int>(menu) + 1 > MENU_ENUM_MAX)
    {
        return menu = static_cast<menu_enum>(MENU_ENUM_MAX);
    } else {
        return menu = static_cast<menu_enum>( static_cast<int>(menu)+ 1 ); 
    }
}
menu_enum operator++(menu_enum& menu, int)
{
  menu_enum tmp(menu);
  ++menu;
  return tmp;
}

inline  menu_enum& operator--(menu_enum& menu)
{
    
    if (static_cast<int>(menu) - 1 < 0)
    {
        return menu = static_cast<menu_enum>(MENU_ENUM_MAX);
    } else {
        return menu = static_cast<menu_enum>( static_cast<int>(menu) - 1 ); 
    }
}
menu_enum operator--(menu_enum& menu, int)
{
  menu_enum tmp(menu);
  --menu;
  return tmp;
}
/* ----------------------------------- CONTROL VARIABLE DECLERATION -----------------------------------*/

static float BLE_Enabled = true;
static float laser_timeout = 10.0; 
static float screen_timeout = 5;
static float device_poweroff_timeout = 300;
static float long_hold_time = 5;

static state_enum previous_state = STATE_IDLE;
static state_enum current_state = STATE_IDLE;
static state_enum next_state = STATE_IDLE;

static mode_enum current_mode = MODE_IDLE;
static mode_enum next_mode = MODE_IDLE;
static mode_enum previous_mode = MODE_IDLE;

static menu_enum menu_selection = MENU_CALIBRATE;

static RM3100 rm3100;
static SCA3300 sca3300;

static RM3100SensorConnection mag_sc(&rm3100);
static SCA3300SensorConnection acc_sc(&sca3300);

static Magnetometer mag(&mag_sc);
static Accelerometer acc(&acc_sc);
static SensorHandler sh(&acc, &mag);

static float current_time;
static float b1_start_time;
static float b2_start_time;
static float b1b2_start_time;

/* ----------------------------------- OPERATION FUNCTION DEFINITIONS -----------------------------------*/

void takeShot(){
    debug(DEBUG_ALWAYS,"Take shot");
    Vector3f shot_data;
    shot_data = sh.get_measurement();
    sh.disableLaser();
    // TODO: Save shot data
}

void enableLaser(){
  sh.enableLaser();
}

void stateB1Waiting(){
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

void stateB2Waiting(){
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

void stateB1B2Waiting()
{
    debug(DEBUG_ALWAYS,"stateB1B2Waiting()");
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

void stateIdle(){
    debug(DEBUG_ALWAYS,"stateIdle()");
  previous_state = current_state;
  next_state = STATE_IDLE;
}

// Enable laser, take shot, align shot, forwards in menu
void stateB1ShortHold(){
  switch(current_mode)
  {
    debug(DEBUG_ALWAYS,"stateB1ShortHold()");
    case(MODE_IDLE):
    next_mode = MODE_LASER_ENA;
    enableLaser();
    break;

    case(MODE_LASER_ENA):
    next_mode = MODE_IDLE;
    takeShot();
    break;

    case(MODE_CALIBRATE):
    if (previous_mode != MODE_CALIBRATE)
    {
        sh.resetCalibration();
    }
    if (sh.collectInertialAlignmentData())
    {
        sh.calibrateInertial();
        next_mode = MODE_IDLE;
    }
    break;

    case(MODE_ALIGN):
    if (previous_mode != MODE_ALIGN)
    {
        sh.resetAlignment();
    }
    if (sh.collectLaserAlignmentData())
    {
        sh.alignLaser();
        next_mode = MODE_IDLE;
    }
    break;

    case(MODE_MENU): menu_selection++; 
    break;

    break;
  }
}

// Select in menu
void stateB1LongHold()
{
    debug(DEBUG_ALWAYS,"stateB1LongHold()");
    switch(current_mode)
    {
    case(MODE_MENU):

    }
}

// Back in menu
void stateB2ShortHold(){
    debug(DEBUG_ALWAYS,"stateB2ShortHold()");
    switch(current_mode)
    {
    case(MODE_MENU): menu_selection--; 
    break;
    }
}
void stateB2LongHold(){
    debug(DEBUG_ALWAYS,"stateB2LongHold()");
  // Return
}

// Enter menu
void stateB1B2ShortHold(){
    debug(DEBUG_ALWAYS,"stateB1B2ShortHold()");
    next_mode = MODE_MENU;
}

// Reset
void stateB1B2LongHold(){
    debug(DEBUG_ALWAYS,"stateB1B2LongHold()");
    next_mode = MODE_IDLE;
    next_state = STATE_IDLE;
    // RESET
}

void mainloop(void* parameter)
{
  current_time = millis();
  previous_state = current_state;
  current_state = next_state;
  previous_mode = current_mode;
  current_mode = next_mode;
}

/* ----------------------------------- SETUP AND LOOP FOR ARDUINO -----------------------------------*/

void setup(){
    TaskHandle_t hardware_handle;
    xTaskCreatePinnedToCore(
        mainloop, /* Function to implement the task */
        "main", /* Name of the task */
        50000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        tskIDLE_PRIORITY ,  /* Priority of the task */
        &hardware_handle,  /* Task handle. */
        0); /* Core where the task should run */
}

void loop(){}
