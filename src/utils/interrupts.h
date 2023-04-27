#ifndef HEADER_INTERRUPTS
#define HEADER_INTERRUPTS

#include <Arduino.h>
#include "utils/config.h"

bool button1_pressed_flag = false;
bool button2_pressed_flag = false;
bool button1_released_flag = false;
bool button2_released_flag = false;

void ISR_button1_pressed()
{
    button1_released_flag = false;
    button1_pressed_flag = true;
}

void ISR_button1_released()
{
    button1_pressed_flag = false;
    button1_released_flag = true;
}

void ISR_button2_pressed()
{
    button2_released_flag = false;
    button2_pressed_flag = true;
}

void ISR_button2_released()
{
    button2_pressed_flag = false;
    button2_released_flag = true;
}

void init_interrupts()
{
    attachInterrupt(PIN_EXTERNAL_BUTTON,ISR_button1_pressed,FALLING);
    attachInterrupt(PIN_EXTERNAL_BUTTON,ISR_button1_released,RISING);

    attachInterrupt(PIN_INTERNAL_BUTTON1,ISR_button2_pressed,FALLING);
    attachInterrupt(PIN_INTERNAL_BUTTON1,ISR_button2_released,RISING);
    //attachInterrupt(PIN_INTERNAL_BUTTON2,ISR_internal_button2,FALLING);
}

#endif