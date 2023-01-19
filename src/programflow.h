
#include "interrupts.h"

enum flow_state {IDLE, MASTER_PRESS, TAKE_SHOT, ACCEL_CALIB, MAG_CALIB, TOGGLE_LASER, LASER_ALIGN};
static flow_state current_state;
static flow_state next_state;

void flow_handler();

void state_idle();

void state_master_button_press();

bool state_accel_calibration();

bool state_mag_calibration();

void state_take_shot();

void state_toggle_laser();

void state_laser_align();

void state_reset();