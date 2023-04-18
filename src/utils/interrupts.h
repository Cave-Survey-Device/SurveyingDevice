#ifndef HEADER_INTERRUPTS
#define HEADER_INTERRUPTS

#include <Arduino.h>
#include "utils/config.h"

// Internal button 1
bool internal_button1_interrupt_flag = false;
void IRAM_ATTR ISR_internal_button1()
{
    internal_button1_interrupt_flag = true;
}

// // Internal button 2
bool internal_button2_interrupt_flag = false;
void IRAM_ATTR ISR_internal_button2()
{
    internal_button2_interrupt_flag = true;
}

// External button
bool external_button_interrupt_flag = false;
void IRAM_ATTR ISR_external_button()
{
    external_button_interrupt_flag = true;
}

void init_interrupts()
{
    attachInterrupt(PIN_EXTERNAL_BUTTON,ISR_external_button,FALLING);
    attachInterrupt(PIN_INTERNAL_BUTTON1,ISR_internal_button1,FALLING);
    //attachInterrupt(PIN_INTERNAL_BUTTON2,ISR_internal_button2,FALLING);
}

#endif