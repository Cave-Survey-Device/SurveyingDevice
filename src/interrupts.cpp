#include "interrupts.h"

bool interrupt_button_released = false;
bool interrupt_button_pressed = false;
bool interrupt_uart_timeout = false;
bool interrupt_get_shot = false;



void init_interrupts()
{
    
    // if (not interrupts_initialised)
    // {
        // interrupts_initialised = true;
        init_shot_timer();
        init_uart_read_timer();

        // Enable pin A0 as an input
        pinMode(A0, INPUT);

        // Set A0 as shot interrupt pin
        attachInterrupt(A0, &ISR_BUTTON_CHANGE, CHANGE);
    // }
}

void IRAM_ATTR ISR_BUTTON_CHANGE()
{
    if (digitalRead(A0))
    {
        interrupt_button_pressed = true;
    } else {
        interrupt_button_released = true;
    }
}

void IRAM_ATTR ISR_GET_SHOT()
{
  interrupt_get_shot = true;
}

void init_uart_read_timer()
{
    uart_read_timer = timerBegin(1, 80, true);
    timerAttachInterrupt(uart_read_timer, &ISR_UART_TIMEOUT, true);
    timerAlarmWrite(uart_read_timer, 3000000, true);
}

void init_shot_timer()
{
    shot_timer = timerBegin(2, 80, true);
    timerAttachInterrupt(shot_timer, &ISR_GET_SHOT, true);
    timerAlarmWrite(shot_timer, 1500000, true);
}

void start_shot_interrupt_timer()
{
    timerRestart(shot_timer);
    timerAlarmEnable(shot_timer);
    timerStart(shot_timer);
}

void start_uart_read_timer()
{
    timerRestart(uart_read_timer);
    timerAlarmEnable(uart_read_timer);
    timerStart(uart_read_timer);
}

void stop_shot_interrupt_timer()
{
    timerStop(uart_read_timer);
    timerAlarmDisable(uart_read_timer);
}

void stop_uart_read_timer()
{
    timerStop(uart_read_timer);
    timerAlarmDisable(uart_read_timer);
}

void disable_shot_interrupt()
{
    timerDetachInterrupt(shot_timer);
}

void enable_shot_interrupt()
{
    timerAttachInterrupt(shot_timer, &ISR_GET_SHOT, true);
}