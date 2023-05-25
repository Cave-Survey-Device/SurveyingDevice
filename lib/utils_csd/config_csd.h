#ifndef HEADER_CONFIG
#define HEADER_CONFIG

#include "Arduino.h"
// Board config
#define ESP32
#ifdef ESP32
#define PIN_LASER_ENA (uint8_t)13U
#define PIN_BUTTON1 (uint8_t)12U
#define PIN_BUTTON2 (uint8_t)27U
#else
#ifdef SEEED_XIAO_SENSE
#define PIN_LASER_ENA PIN_A1
#define PIN_EXTERNAL_BUTTON PIN_A2
#define PIN_INTERNAL_BUTTON1 PIN_A3
#endif
#endif




const static float DISTO_LEN = 0.1;

const static bool DEBUG = true;

const static bool DEBUG_FILE_ENA = false;
const static bool DEBUG_LASER_ENA = false;
const static bool DEBUG_LASER_EXTENDED_ENA = false;
const static bool DEBUG_INERTIALSENSOR_ENA = false;
const static bool DEBUG_SENSORHANDLER_ENA = true;
const static bool DEBUG_MAIN_ENA = true;

const static bool DEBUG_INERTIAL_ALIGN_ENA = false;
const static bool DEBUG_INERTIAL_CALIB_ENA = false;
const static bool DEBUG_LASER_ALIGN_ENA = false;


enum debug_type {
    DEBUG_ALWAYS, 
    DEBUG_FILE, 
    DEBUG_LASER, 
    DEBUG_LASER_EXTENDED, 
    DEBUG_INERTIALSENSOR,
    DEBUG_SENSORHANDLER, 
    DEBUG_MAIN, 
    DEBUG_INERIAL_ALIGN,
    DEBUG_INERTIAL_CALIB, 
    DEBUG_LASER_ALIGN};

const static bool DEBUG_BOOL_ARR[9] = {
    DEBUG_FILE_ENA, 
    DEBUG_LASER_ENA, 
    DEBUG_LASER_EXTENDED_ENA, 
    DEBUG_INERTIALSENSOR_ENA,
    DEBUG_SENSORHANDLER_ENA, 
    DEBUG_MAIN_ENA, 
    DEBUG_INERTIAL_ALIGN_ENA, 
    DEBUG_INERTIAL_CALIB_ENA, 
    DEBUG_LASER_ALIGN_ENA};

const static char DEBUG_STR_ARR[9][7] = {
    "FILE  ", 
    "LASER ", 
    "LASER+", 
    "SENSOR", 
    "SHNDLR", 
    "MAIN  ", 
    "IALIGN", 
    "ICALIB",
    "LALIGN"};

#endif