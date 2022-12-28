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
static double distance1 = 0;

// Seconds distance measurement from lidar (only used for base station creation)
static double distance2 = 0;

// Has the shot interrupt triggered?
static bool interrupted = false;

// Has the timeout for the second shot for base station been exceeded?
bool timedout = false;

void save_splay(double distance,bool base=false)
{
  // Initialise local variables
  // Node n to hold the node to be saved
  node n;
  // string format of the int id of the node to be saved
  char str_id[4];
  // Heading of the disto (radians)
  double heading;
  // Inclination of the diston (radians)
  double inclination;

  // Get data from magnetometer
  magnetometer.update();
  heading = magnetometer.get_heading();

  // Get data from accelerometer
  accelerometer.update();
  inclination = accelerometer.get_inclination();

  // Populate node object
  n.id = new_node_id;
  n.previous = current_base_id;
  n.vector_to_prev = generate_vector(distance, heading, inclination);

  // Populate str_id with string version of int id
  sprintf(str_id,"%d",n.id);
  write_to_file(current_file_name,str_id,(const node*)&n);

  // read from file and print result as a debug
  read_from_file(current_file_name,str_id,&n);
}

void interrupt_loop()
{
  current_state = next_state;
  debug(DEBUG_MAIN, "Entering interrupt loop");

  switch (current_state){
    case IDLE:
      debug(DEBUG_MAIN, "Currently in IDLE state");
      next_state = SPLAY;
      try
      {
        distance1 = lidar.get_measurement();
        debug(DEBUG_MAIN, "Succesfully got measurement");
      }
      catch(const std::exception& e)
      {
        debug(DEBUG_MAIN, "Failed to got measurement");
        Serial.println(e.what());
        next_state = IDLE;
      }
      break;
      
      
      // Serial.println("BEGINNING TIMER");
      // timerRestart(My_timer);
      // timerAlarmWrite(My_timer, 3000000, true);
      // timerAlarmEnable(My_timer);
    
    case WAITING:
      debug(DEBUG_MAIN, "Currently in WAITING state");
      //timerAlarmDisable(My_timer);
      if (timedout = false)
      {
        next_state = BASE;
      } else {
        next_state = SPLAY;
      }
      break;

    case SPLAY:
      debug(DEBUG_MAIN, "Currently in SPLAY state");
      next_state = IDLE;
      save_splay(distance1);
      new_node_id += 1;
      break;
      
    case BASE:
      debug(DEBUG_MAIN, "Currently in BASE state");
      next_state = IDLE;
      try
      {
        debug(DEBUG_MAIN, "Succesfully got measurement");
        distance2 = lidar.get_measurement();
      }
      catch(const std::exception& e)
      {
        debug(DEBUG_MAIN, "Failed to got measurement");
        Serial.println(e.what());
        break;
      }

      if (fabs(distance1 - distance2) / (distance1 + distance2) > 1e-3)
      {
        display.clearDisplay();
        display.setCursor(0,0);
        debug(DEBUG_MAIN, "Error in distance greater than 1%, please retry!");
        //display.write("Error in distance greater than 1%, please retry!");
      }
      else {
        debug(DEBUG_MAIN, "Saving splay");
        save_splay(0.5*(distance1 + distance2),true);
        new_node_id += 1;
      }
      break;
  }
}

void IRAM_ATTR ISR_GET_SHOT()
{
  interrupted = true;
}

void setup(){
  // Initialise serial and UARTs
  Serial.begin(9600);
  Serial1.begin(9600);

  // Set A0 as shot interrupt pin
  pinMode(A0, INPUT);
  attachInterrupt(A0, ISR_GET_SHOT, RISING);
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
  init_bno();
  magnetometer.init();
  lidar.init();

  // TODO: Initialise timer - currently causing heap corruption!
  // My_timer = timerBegin(0, 80, true);
  // timerAttachInterrupt(My_timer, ISR_GET_SHOT, true);
  // timerAlarmWrite(My_timer, 3000000, true);
}

void loop(){
  if (interrupted)
  {
    debug(DEBUG_MAIN, "\nInterrupt!");
    interrupt_loop();
    interrupted = false;
  }
}