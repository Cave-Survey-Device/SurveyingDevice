#include <Arduino.h>
// #include <Adafruit_BNO055.h>

#include <iostream>
#include <math.h>
#include <nvs_flash.h>
#include <string>

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <BNO055_support.h>
#include <Wire.h>

#include "filefuncs.h"
#include "magnetometer.h"
#include "accelerometer.h"
#include "OLED.h"
#include "config.h"
#include "lidar.h"
#include "unified.h"
#include "interrupts.h"
#include "BLE.h"
#include "sensorhandler.h"
#include "programflow.h"


static Adafruit_SSD1306 display;

// Define sensor structs
static struct bno055_t myBNO;
static struct bno055_gravity myGravityData;
static struct bno055_mag myMagData;
unsigned char accelCalibStatus = 0;		//Variable to hold the calibration status of the Accelerometer
unsigned char magCalibStatus = 0;		//Variable to hold the calibration status of the Magnetometer
unsigned char gyroCalibStatus = 0;		//Variable to hold the calibration status of the Gyroscope
unsigned char sysCalibStatus = 0;		//Variable to hold the calibration status of the System (BNO055's MCU)

// Static global objects
static Magnetometer magnetometer(&myMagData);
static Accelerometer accelerometer(&myGravityData);
static Lidar lidar;
static BLEHandler blehandler;
static SensorHandler sensorhandler(&accelerometer, &magnetometer, &lidar);

// Current shot ID
static int shot_ID = 0;
// Calibration flag
static bool flag_calibrate = false;

// File used to store all survey data in
static char current_file_name[] = "test.survey";

static double heading_correction, inclination_correction = 0;

/****************************************************
*   All global variables are now declared. Following
*   declerations are for general mainloop functions.
*****************************************************/

// Initialises BNO sensor
void init_bno(){
  //Initialize I2C communication
  Wire.begin();
 
  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device
 
  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
 
  delay(1);
}

void setup_BLE()
{
  debug(DEBUG_MAIN,"Starting BLE...");
  blehandler.start();
}

void setup_hw(){
  // Initialise serial and UARTs
  Serial.begin(9600);
  Serial1.begin(9600);

  // Set GPIO_14 as LIDAR enable
  pinMode(GPIO_NUM_14, OUTPUT);
  digitalWrite(GPIO_NUM_14,LOW);

  // Initialise OLED displays
  display = init_OLED(0x3C);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.setCursor(0,0);
  display.display();

  // Initialise data objects and sensors
  debug(DEBUG_MAIN,"Initialising sensor objects");
  init_bno();
  magnetometer.init();

  init_interrupts();
  try
  {
    lidar.init();
  }
  catch(const char* e)
  {
    Serial.println(e);
  }

  flag_calibrate = false;
  debug(DEBUG_MAIN,"Finished hw setup...");
}

// Task to execute on 1st core
void Core1Task(void * parameter)
{
  setup_hw();
  while(true)
  {
    flow_handler();
    delay(1);
  } 
}

// Task to execute on 2nd core
void Core2Task(void * parameter)
{
  setup_BLE();
  while(true)
  {
    delay(1);
  }
}

// Setup for ESP32 - ALWAYS RUNS ONCE, followed by execution of "void loop()"
void setup()
{
  TaskHandle_t BLE_handle;
  xTaskCreatePinnedToCore(
      Core2Task, /* Function to implement the task */
      "BLE", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &BLE_handle,  /* Task handle. */
      1); /* Core where the task should run */

  TaskHandle_t hardware_handle;
  xTaskCreatePinnedToCore(
      Core1Task, /* Function to implement the task */
      "Hardware", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &hardware_handle,  /* Task handle. */
      0); /* Core where the task should run */
}

// void setup()
// {
//   // Initialise serial and UARTs
//   Serial.begin(9600);
//   Serial1.begin(9600);

//   // Set GPIO_14 as LIDAR enable
//   pinMode(GPIO_NUM_14, OUTPUT);
//   digitalWrite(GPIO_NUM_14,LOW);

//   // Initialise data objects and sensors
//   debug(DEBUG_MAIN,"Initialising sensor objects");
//   init_bno();
//   magnetometer.init();

//   init_interrupts();
//   try
//   {
//     lidar.init();
//   }
//   catch(const char* e)
//   {
//     Serial.println(e);
//   }

//   flag_calibrate = false;
// }


// Runs aafter setup
void loop(){
  // TODO: Execute BLE commands which should have a struct made for them.
  // After command executed ack should be sent back such that commands aren't spammed during operation.

  // TODO: Move BLE onto CPU2 and operate all main HW stuff on CPU1. Make sure to init all interrupts on CPU1

  // interrupt_loop();
  // delay(500);

  // Magnetometer update code
  // magnetometer.update();
  // accelerometer.update();
  // magnetometer.add_calibration_data();
  // Serial.println(magnetometer.check_calibration_progress());
  // delay(50);

  
  // sensorhandler.update();
  // Serial.print("Calculated direction: ");
  // Serial.println(RAD_TO_DEG * sensorhandler.get_heading());
  // Serial.printf("Magnetometer X %f   Y %f    Z %f\n", (double)myMagData.x, (double)myMagData.y, (double)myMagData.z);
    
  // bno055_get_syscalib_status(&sysCalibStatus);
  // Serial.print("System Calibration Status: ");			//To read out the Magnetometer Calibration Status (0-3)
  // Serial.println(sysCalibStatus);
  
  // Serial.println();				
  // delay(250);
}
