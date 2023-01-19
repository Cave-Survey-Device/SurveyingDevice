// #ifndef HEADER_PROGRAMFLOW
// #define HEADER_PROGRAMFLOW

// #include <Arduino.h>
// // #include <Adafruit_BNO055.h>

// #include <iostream>
// #include <math.h>
// #include <nvs_flash.h>
// #include <string>

// #include <Adafruit_BNO055.h>
// #include <Adafruit_Sensor.h>
// #include <BNO055_support.h>
// #include <Wire.h>

// #include "filefuncs.h"
// #include "magnetometer.h"
// #include "accelerometer.h"
// #include "OLED.h"
// #include "config.h"
// #include "lidar.h"
// #include "unified.h"
// #include "interrupts.h"
// #include "BLE.h"
// #include "sensorhandler.h"

// static Adafruit_SSD1306 display;

// // Define sensor structs
// static struct bno055_t myBNO;
// static struct bno055_gravity myGravityData;
// static struct bno055_mag myMagData;

// // Static global objects
// Magnetometer magnetometer(&myMagData);
// Accelerometer accelerometer(&myGravityData);
// Lidar lidar;
// BLEHandler blehandler;
// SensorHandler sensorhandler(&accelerometer, &magnetometer, &lidar);

// static int shot_ID = 0; // Current shot ID
// static char current_file_name[] = "test.survey"; // File used to store all survey data in
// static double heading_correction, inclination_correction = 0;


// enum flow_state {IDLE, MASTER_PRESS, TAKE_SHOT, ACCEL_CALIB, MAG_CALIB, TOGGLE_LASER, LASER_ALIGN};
// static flow_state current_state = IDLE;
// static flow_state next_state = IDLE;


// void flow_handler();

// void state_idle();

// void state_master_button_press();

// void state_accel_calibration();

// void state_mag_calibration();

// void state_take_shot();

// void state_toggle_laser();

// void state_laser_align();

// void state_reset();

// void save_shot_to_BLE();

// void save_shot_to_flash();


// void init_bno();
// void setup_BLE();
// void setup_hw();

// #endif