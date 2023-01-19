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

// Static global objects
Magnetometer magnetometer(&myMagData);
Accelerometer accelerometer(&myGravityData);
Lidar lidar;
BLEHandler blehandler;
SensorHandler sensorhandler(&accelerometer, &magnetometer, &lidar);

static int shot_ID = 0; // Current shot ID
static char current_file_name[] = "test.survey"; // File used to store all survey data in
static double heading_correction, inclination_correction = 0;

enum flow_state {IDLE, MASTER_PRESS, TAKE_SHOT, ACCEL_CALIB, MAG_CALIB, TOGGLE_LASER, LASER_ALIGN};
static flow_state current_state = IDLE;
static flow_state next_state = IDLE;
/****************************************************
*   All global variables are now declared. Following
*   declerations are for general mainloop functions.
*****************************************************/

void save_shot_to_flash()
{
    Vector3d shot_data; // Variable to hold shot data
    node *n = (struct node*)malloc(sizeof(node)); // Node struct to hold data to be saved
    char str_id[4]; // String formatted shot id

    // Get the shot data
    shot_data = sensorhandler.get_shot_data();

    // Populate node object
    debug(DEBUG_MAIN, "Assigning values to node object");
    n->id = shot_ID;
    n->heading = shot_data(0);
    n->inclination = shot_data(1);
    n->distance = shot_data(2);

    // Populate str_id with string version of int id
    debug(DEBUG_MAIN, "Writing to file");
    sprintf(str_id,"%d",shot_ID);
    write_to_file(current_file_name,str_id,n);
    
    debug(DEBUG_MAIN, "Finished saving data, returning...");
}

void save_shot_to_BLE()
{
    Vector3d shot_data; // Variable to hold shot data
    node *n = (struct node*)malloc(sizeof(node)); // Node struct to hold data to be saved
    char str_id[4]; // String formatted shot id

    // Get the shot data
    shot_data = sensorhandler.get_shot_data();

    // Write data to BLE
    snprintf(str_id,sizeof(str_id),"%d",shot_ID);
    read_from_file(current_file_name,str_id,n);
    blehandler.shared_bledata.write_data(n);
    blehandler.update();
}

void state_idle(){
  char cmd[30];
  blehandler.shared_bledata.read_command(cmd);
  if (strcmp(cmd, "calibrate accel") == 0 )
  {
    next_state = ACCEL_CALIB;
  } else if (strcmp(cmd, "calibrate mag") == 0 )
  {
    magnetometer.init();
    next_state = MAG_CALIB;
  } else if (strcmp(cmd, "align laser") == 0 )
  {
    next_state = LASER_ALIGN;
  } else if (interrupt_button_pressed)
  {
    next_state = MASTER_PRESS;
    reset_flow_interrupt_flags();
    start_shot_interrupt_timer();
  }
}

void state_master_button_press()
{
    if (interrupt_button_released)
    {
        stop_shot_interrupt_timer();
        reset_flow_interrupt_flags();
        next_state = TOGGLE_LASER;
    } else if (interrupt_get_shot)
    {
        stop_shot_interrupt_timer();
        reset_flow_interrupt_flags();
        next_state = TAKE_SHOT;
    }
}

void state_accel_calibration()
{
    // if (!CALIB_FUNC())
    // {
    //     next_state = ACCEL_CALIB;
    // }
}

void state_mag_calibration()
{
    magnetometer.update();
    magnetometer.add_calibration_data();
    if (touchRead(4))
    {
        next_state = IDLE;
        magnetometer.calibrate();
    } else{
        next_state = MAG_CALIB;
    }
}

void state_take_shot()
{
  sensorhandler.update();
  save_shot_to_flash();
  save_shot_to_BLE();
  shot_ID += 1;
}

void state_laser_align(){
    if (interrupt_button_released)
    {
        stop_shot_interrupt_timer();
        reset_flow_interrupt_flags();
        lidar.toggle_laser();
    } else if (interrupt_get_shot)
    {
        stop_shot_interrupt_timer();
        reset_flow_interrupt_flags();
        sensorhandler.update();
    }

    if (!sensorhandler.add_laser_calibration())
    {
        next_state = LASER_ALIGN;
    }
}

void state_toggle_laser()
{
    lidar.toggle_laser();
}

void state_reset()
{
    next_state = IDLE;
}

void flow_handler()
{
    current_state = next_state;
    switch(current_state)
    {
        case(IDLE):
            state_idle();
            break;

        case(MASTER_PRESS):
            next_state = MASTER_PRESS;
            state_master_button_press();
            break;

        case(TOGGLE_LASER):
            next_state = IDLE;
            state_toggle_laser();
            break;

        case(TAKE_SHOT):
            next_state = IDLE;
            state_take_shot();
            break;

        case(ACCEL_CALIB):
            next_state = IDLE;
            state_accel_calibration();
            break;

        case(MAG_CALIB):
            next_state = IDLE;
            state_mag_calibration();
            break;

        case(LASER_ALIGN):
            next_state = IDLE;
            state_laser_align();
            break;
    }
}


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

  debug(DEBUG_MAIN,"Finished hw setup...");
}


// Task to execute on 1st core
void Core1Task(void * parameter)
{
  setup_hw();
  reset_flow_interrupt_flags();
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

// Runs aafter setup
void loop()
{
}
