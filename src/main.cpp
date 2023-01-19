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

// Mainloop flow state enum
enum shot_status {IDLE, WAITING, SPLAY, BASE};
// Holds current state to act on in after interrupt triggered
static shot_status current_state = IDLE;
// Holds next state to act on in after interrupt triggered
static shot_status next_state = IDLE;

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

// Code to save a splay to flash
void save_splay()
{
  Vector3d splay_data;
  splay_data = sensorhandler.get_shot_data();

  // Node struct to hold data to be saved
  node *n = (struct node*)malloc(sizeof(node));

  char str_id[4];
  // Populate node object
  debug(DEBUG_MAIN, "Assigning values to node object");
  n->id = shot_ID;
  n->heading = splay_data(0);
  n->inclination = splay_data(1);
  n->distance = splay_data(2);

  // Populate str_id with string version of int id
  debug(DEBUG_MAIN, "Writing to file");
  sprintf(str_id,"%d",shot_ID);
  write_to_file(current_file_name,str_id,n);
  
  debug(DEBUG_MAIN, "Finished saving data, returning...");
}

// Waiting state
void idle_state()
{
  if (interrupt_button_pressed)
  {
    interrupt_button_pressed = false;
    interrupt_get_shot = false;
    next_state = WAITING;
    // debug(DEBUG_MAIN,"\n\n---------------------------------------");
    // debug(DEBUG_MAIN,"Starting shot timer...");
    start_shot_interrupt_timer();
  }
}

// State which decides between toggling laser and taking shot
void waiting_state()
{
  if (interrupt_button_released)
  {
    //debug(DEBUG_MAIN,"Stopping shot timer...");
    stop_shot_interrupt_timer();
    interrupt_button_pressed = false;
    interrupt_button_released = false;
    interrupt_get_shot = false;
    //Serial.println("Toggle laser");
    next_state = IDLE;
    lidar.toggle_laser();
  } else if (interrupt_get_shot)
  {
    //debug(DEBUG_MAIN,"Stopping shot timer...");
    stop_shot_interrupt_timer();
    interrupt_button_pressed = false;
    interrupt_button_released = false;
    interrupt_get_shot = false;
    //Serial.println("Get shot...");
    next_state = SPLAY;
    //debug(DEBUG_MAIN, "Received shot interrupt");
    try
    {
      sensorhandler.update();
      debug(DEBUG_MAIN, "Succesfully got measurement");
    }
    catch(const char* e)
    {
      debug(DEBUG_MAIN, "Failed to got measurement");
      laser_on = false;
      Serial.println(e);
      next_state = IDLE;
    }
  } 
}

// State in which a splay is saved and BLE hdanler is updated
void splay_state()
{
  disable_shot_interrupt();
  next_state = IDLE;
  save_splay();

  debug(DEBUG_MAIN,"Reading splay and sending to BLE...");
  node n1;
  char str_id[4];
  snprintf(str_id,sizeof(str_id),"%d",shot_ID);
  read_from_file(current_file_name,str_id,&n1);
  blehandler.shared_bledata.write_data(&n1);
  blehandler.update();

  shot_ID += 1;
  enable_shot_interrupt();
}

// State in which calibration of the LIDAR takes plcace
void calibrate_state()
{
  bool completed;
  disable_shot_interrupt();
  next_state = IDLE;

  completed = sensorhandler.calibrate();
  if (completed)
  {
    flag_calibrate = false;
  }

  debug(DEBUG_MAIN, "Finished adding calibration data, returning...");
  enable_shot_interrupt();
}

// Main loop in which interrupt causes break from IDLE state - main FSM of the system
void interrupt_loop()
{
  current_state = next_state;
  switch (current_state){
    case IDLE:
      idle_state();
      break;

    case WAITING:
      waiting_state();
      break;

    case SPLAY:
      // maybe set a flag when cmd has changed to decrease processing?
      char cmd[20];
      blehandler.shared_bledata.read_command(cmd);
      if (strcmp(cmd, "calibrate") == 0 )
      {
        blehandler.shared_bledata.write_command("no command");
        flag_calibrate = true;
      }

      if (flag_calibrate != true)
      {
        debug(DEBUG_MAIN,"Splay state");
        splay_state();
      } else {
        debug(DEBUG_MAIN,"Calibration state");
        char cmd[50];
        blehandler.shared_bledata.read_command(cmd);
        Serial.println(cmd);

        calibrate_state();
      }
      break;
  }
}

void setup_BLE()
{
  debug(DEBUG_MAIN,"Starting BLE...");
  blehandler.start();
}
// Runs on power on
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

// Task to execute on 2nd core
void Core1Task(void * parameter)
{
  setup_hw();
  while(true)
  {
    interrupt_loop();
    delay(100);
  } 
}

// Task to execute on 1st core
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
      0); /* Core where the task should run */

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
