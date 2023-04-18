#include <Arduino.h>
// #include <Adafruit_BNO055.h>

#include <iostream>
#include <math.h>
#include <nvs_flash.h>
#include <string>

#include <Wire.h>

#include "filefuncs.h"
#include "magnetometer.h"
#include "accelerometer.h"
#include "config.h"
#include "lidar.h"
#include "utility.h"
#include "interrupts.h"
#include "BLE.h"
#include "sensorhandler.h"
#include "RM3100.h"
#include "LDK_2M.h"
#include "SCA3300.h"


// Define sensor structs
static RM3100 magnetometer;
static LDK_2M lidar;
static SCA3300 accelerometer;

BLEHandler blehandler;
SensorHandler sensorhandler(&accelerometer, &magnetometer, &lidar);
/****************************************************
*   All global variables are now declared. Following
*   declerations are for general mainloop functions.
*****************************************************/


// Setup for ESP32 - ALWAYS RUNS ONCE, followed by execution of "void loop()"
void setup()
{
    Serial.begin(9600);
    accelerometer.test_calibration();
}

// Runs aafter setup
void loop()
{
    
}
