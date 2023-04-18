#define TEST_MODE
#ifdef TEST_MODE
#include "test.h"

void setup()
{
  test_main();
}

void loop(){}

#else
#include <Arduino.h>
#include "sensors/Sensors.h"
#include "utils/interrupts.h"
enum state {STATE_CALIBRATE, STATE_ALIGN, STATE_IDLE, D};

state current_state = STATE_IDLE;
state next_state = current_state;

SensorHandler sh;

void setup() {

  // put your setup code here, to run once:
}

void state_calibration()
{
  if (external_button_interrupt_flag)
  {
    external_button_interrupt_flag = false;
    if(sh.CollectCalibrationData())
    {
        sh.CalibrateInertial();
        sh.AlignInertial();
        next_state = STATE_ALIGN;
    }
  }

}

void state_align()
{
  if (external_button_interrupt_flag)
  {
    external_button_interrupt_flag = false;
    if(sh.CollectAlignmentData())
    {
      next_state = STATE_IDLE;
    }
  }
}

void state_BLE()
{

}

void loop() {
  current_state = next_state;
  switch(current_state)
  {
    case (STATE_CALIBRATE):
    state_calibration();
    break;

    case (STATE_ALIGN):
      state_align();
    break;

    case (STATE_IDLE):
    if (internal_button1_interrupt_flag)
    {
      internal_button1_interrupt_flag = false;
      next_state = STATE_CALIBRATE;
      sh.ResetCalibration();
    }
    break;
  }
}

#endif