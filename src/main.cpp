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
#include "lasercalibration.h"
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

void save_splay()
{
  // Node struct to hold data to be saved
  node *n = (struct node*)malloc(sizeof(node));

  char str_id[4];
  // Populate node object
  debug(DEBUG_MAIN, "Assigning values to node object");
  n->id = shot_ID;
  n->heading = sensorhandler.get_heading();
  n->inclination = sensorhandler.get_inclination();
  n->distance = sensorhandler.get_distance();

  // Populate str_id with string version of int id
  debug(DEBUG_MAIN, "Writing to file");
  sprintf(str_id,"%d",shot_ID);
  write_to_file(current_file_name,str_id,n);
  
  debug(DEBUG_MAIN, "Finished saving data, returning...");
}

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

void calibrate_state()
{
  bool completed;
  disable_shot_interrupt();
  next_state = IDLE;

  completed = sensorhandler.calibrate();
  if (completed)
  {
    flag_calibrate = false;
  } else {
    blehandler.shared_bledata.write_command("no command");
  }

  debug(DEBUG_MAIN, "Finished adding calibration data, returning...");
  enable_shot_interrupt();
}

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
        flag_calibrate = true;
      } else{
        flag_calibrate = false;
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


void Core1Task(void * parameter)
{
  setup_hw();
  while(true)
  {
    interrupt_loop();
    delay(100);
  } 
}

void Core2Task(void * parameter)
{
  setup_BLE();
  while(true)
  {
    delay(1);
  }
}

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
  delay(2000);

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

}