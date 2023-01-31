#include "interrupts.h"

bool interrupt_button_released = false;
bool interrupt_button_pressed = false;
bool interrupt_get_shot = false;

void init_interrupts()
{
    
    // if (not interrupts_initialised)
    // {
        // interrupts_initialised = true;
        init_shot_timer();

        // Enable pin A0 as an input
        pinMode(A0, INPUT);

        // Set A0 as shot interrupt pin
        attachInterrupt(A0, &ISR_BUTTON_CHANGE, CHANGE);
    // }
}

void reset_flow_interrupt_flags()
{
    interrupt_button_pressed = false;
    interrupt_button_released = false;
    interrupt_get_shot = false;
}

void IRAM_ATTR ISR_BUTTON_CHANGE()
{
    if (digitalRead(A0))
    {
        interrupt_button_pressed = true;
        interrupt_button_released = false;
    } else {
        interrupt_button_pressed = false;
        interrupt_button_released = true;
    }
}

void IRAM_ATTR ISR_GET_SHOT()
{
  interrupt_get_shot = true;
}



void init_shot_timer()
{
    shot_timer = timerBegin(2, 80, true);
    timerAttachInterrupt(shot_timer, &ISR_GET_SHOT, false);
    timerAlarmWrite(shot_timer, 1000000, false);
}

void start_shot_interrupt_timer()
{
    // Serial.println("Starting shot timer...");
    timerRestart(shot_timer);
    timerAlarmEnable(shot_timer);
    timerStart(shot_timer);
}

void stop_shot_interrupt_timer()
{
    // Serial.println("Stopping shot timer...");
    interrupt_get_shot = false;
    timerStop(shot_timer);
    timerAlarmDisable(shot_timer);
}

void disable_shot_interrupt()
{
    timerDetachInterrupt(shot_timer);
}

void enable_shot_interrupt()
{
    timerAttachInterrupt(shot_timer, &ISR_GET_SHOT, true);
}