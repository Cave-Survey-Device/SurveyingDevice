#ifndef HEADER_CONFIG
#define HEADER_CONFIG

#include "Arduino.h"
// Board config
#define PIN_LASER_ENA PIN_A1
#define PIN_EXTERNAL_BUTTON PIN_A2
#define PIN_INTERNAL_BUTTON1 PIN_A3

const static bool DEBUG = false;
const static bool DEBUG_LASER_CAL = true;

const static float DISTO_LEN = 0.1;
const static bool DEBUG_FILE_ENA = false;
const static bool DEBUG_LIDAR_ENA = false;
const static bool DEBUG_OLED_ENA = false;
const static bool DEBUG_MAG_ENA = false;
const static bool DEBUG_ACCEL_ENA = true;
const static bool DEBUG_MAIN_ENA = true;
const static bool DEBUG_LIDAR_EXTENDED_ENA = false;
const static bool DEBUG_SENSOR_ENA = true;

const static unsigned int DEBUG_ALWAYS = 1;
const static unsigned int DEBUG_FILE = 1;
const static unsigned int DEBUG_LIDAR = 2;
const static unsigned int DEBUG_OLED = 3;
const static unsigned int DEBUG_MAG = 4;
const static unsigned int DEBUG_ACCEL = 5;
const static unsigned int DEBUG_MAIN = 6;
const static unsigned int DEBUG_LIDAR_EXTENDED = 7;
const static unsigned int DEBUG_SENSOR = 8;

const static bool DEBUG_BOOL_ARR[8] = {DEBUG_FILE_ENA, DEBUG_LIDAR_ENA, DEBUG_OLED_ENA, DEBUG_MAG_ENA, DEBUG_ACCEL_ENA, DEBUG_MAIN_ENA, DEBUG_LIDAR_EXTENDED_ENA, DEBUG_SENSOR_ENA};
const static char DEBUG_STR_ARR[8][6] = {"FILE ", "LIDAR", "OLED ", "MAG  ", "ACCEL", "MAIN ", "LIDEX", "SENSR"};

#endif