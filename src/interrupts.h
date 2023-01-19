#ifndef HEADER_INTERRUPTS
#define HEADER_INTERRUPTS

#include <Arduino.h>

// Has the shot interrupt triggered?
static bool interrupts_initialised = false;
extern bool interrupt_button_released;
extern bool interrupt_button_pressed;

void reset_flow_interrupt_flags();

void IRAM_ATTR ISR_BUTTON_CHANGE();

extern bool interrupt_get_shot;
static hw_timer_t* shot_timer = NULL;
void IRAM_ATTR ISR_GET_SHOT();

extern bool interrupt_uart_timeout;
static hw_timer_t* uart_read_timer = NULL;
void IRAM_ATTR ISR_UART_TIMEOUT();

void init_shot_timer();
void init_uart_read_timer();

void start_shot_interrupt_timer();
void start_uart_read_timer();

void stop_shot_interrupt_timer();
void stop_uart_read_timer();

void init_interrupts();

void disable_shot_interrupt();
void enable_shot_interrupt();


#endif