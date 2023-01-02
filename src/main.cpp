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

// Defining global variables
static Adafruit_SSD1306 display;

// Define sensor structs
static struct bno055_t myBNO;
static struct bno055_gravity myGravityData;
static struct bno055_mag myMagData;

// Global magnetometer object
static Magnetometer magnetometer(&myMagData);

// Global accelerometer object
static Accelerometer accelerometer(&myGravityData);

// Global LIDAR object
static Lidar lidar;

// ID of the current base station
static unsigned int current_base_id = 0;

// ID of the next node to be created
static unsigned int new_node_id = 1;

// File used to store all survey data in
static char current_file_name[] = "test.survey";

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

// Mainloop flow state enum
enum shot_status {IDLE, WAITING, SPLAY, BASE};

// Holds current state to act on in after interrupt triggered
static shot_status current_state = IDLE;

// Holds next state to act on in after interrupt triggered
static shot_status next_state = IDLE;

// First distance measurement from lidar
static double distance = 0;

// Has the shot interrupt triggered?
static bool shot_interrupt = false;


void IRAM_ATTR ISR_GET_SHOT()
{
  shot_interrupt = true;
}

void save_splay(double distance,bool base=false)
{
  // Initialise local variables
  // Node n to hold the node to be saved
  //node n;
  node *n = (struct node*)malloc(sizeof(node));
  // string format of the int id of the node to be saved
  char str_id[4];
  // Heading of the disto (radians)
  double heading;
  // Inclination of the disto (radians)
  double inclination;

  debug(DEBUG_MAIN, "Updating sensors");
  // Get data from magnetometer
  magnetometer.update();
  heading = magnetometer.get_heading();

  // Get data from accelerometer
  accelerometer.update();
  inclination = accelerometer.get_inclination();

  // Populate node object
  debug(DEBUG_MAIN, "Assigning values to node object");
  n->id = new_node_id;
  n->previous = current_base_id;
  n->heading = heading;
  n->inclination = inclination;

  // Populate str_id with string version of int id
  debug(DEBUG_MAIN, "Writing to file");
  sprintf(str_id,"%d",n->id);
  write_to_file(current_file_name,str_id,(const node*)&n);

  // read from file and print result as a debug
  // debug(DEBUG_MAIN, "Reading from file");
  // read_from_file(current_file_name,str_id,&n);
  // read_from_file(current_file_name,str_id,n);
  // debug(DEBUG_MAIN, "Finished saving data, returning...");
}

void interrupt_loop()
{
  current_state = next_state;

  switch (current_state){

    case IDLE:
      if (shot_interrupt)
      {
        shot_interrupt = false;
        next_state = SPLAY;
        debug(DEBUG_MAIN, "Received shot interrupt");
        try
        {
          distance = lidar.get_measurement();
          debug(DEBUG_MAIN, "Succesfully got measurement");
        }
        catch(const char* e)
        {
          debug(DEBUG_MAIN, "Failed to got measurement");
          Serial.println(e);
          next_state = IDLE;
        }
      }
      break;


    case SPLAY:
      next_state = IDLE;
      save_splay(distance);
      new_node_id += 1;
      break;
  }
}

// Runs on power on
void setup(){
  // Initialise serial and UARTs
  Serial.begin(9600);
  Serial1.begin(9600);

  // Set A0 as shot interrupt pin
  pinMode(A0, INPUT);
  attachInterrupt(A0, &ISR_GET_SHOT, RISING);
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
  try
  {
    lidar.init();
  }
  catch(const char* e)
  {
    Serial.println(e);
  }
}

// Runs aafter setup
void loop(){
  interrupt_loop();
  delay(5);
}