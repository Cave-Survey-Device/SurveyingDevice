#ifndef HEADER_INTERRUPTS
#define HEADER_INTERRUPTS

#include <Arduino.h>
#include "config_csd.h"

bool button1_pressed_flag = false;
bool button2_pressed_flag = false;
bool button1_released_flag = false;
bool button2_released_flag = false;

void IRAM_ATTR ISR_button1_change()
{
    if (digitalRead(PIN_BUTTON1))
    {
    button1_released_flag = false;
    button1_pressed_flag = true;
    } else {
    button1_released_flag = true;
    button1_pressed_flag = false;
    }
}
void IRAM_ATTR ISR_button2_change()
{
    if (digitalRead(PIN_BUTTON2))
    {
    button2_released_flag = false;
    button2_pressed_flag = true;
    } else {
    button2_released_flag = true;
    button2_pressed_flag = false;
    }
}

void init_interrupts()
{
    attachInterrupt(PIN_BUTTON1,ISR_button1_change,CHANGE);
    attachInterrupt(PIN_BUTTON2,ISR_button2_change,FALLING);
}

#endif