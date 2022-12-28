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
static float tilt_arr[3];
static float orientation_arr[3];

static struct bno055_t myBNO;
static struct bno055_gravity myGravityData;
static struct bno055_mag myMagData;

static Magnetometer magnetometer(&myMagData);
static Accelerometer accelerometer(&myGravityData);
static Lidar lidar;

static unsigned int current_base_id = 0;
static unsigned int new_node_id = 1;

static char current_file_name[] = "test.survey";


void test_lasercalibration(){
  char buffer[150];

  Serial.begin(9600);
  Serial.println("");
  Serial.println("Beginning!");
  MatrixXd g_vec(3,9);
  g_vec <<  9.25185853854297e-18, 0.3420201433256687, 0.24184476264797528, 3.019455222692793e-17, -0.24184476264797522, -0.3420201433256687, -0.2418447626479753, -5.35762225266119e-17, 0.2418447626479752,
            0.0, 0.0, 0.7071067811865476, 1.0, 0.7071067811865476, 1.2246467991473532e-16, -0.7071067811865475, -1.0, -0.7071067811865477,
            3.700743415417188e-17, 0.9396926207859084, 0.6644630243886748, 9.454701216556439e-17, -0.6644630243886747, -0.9396926207859084, -0.6644630243886749, -1.3561129988000566e-16, 0.6644630243886746;
  // g_vec << 1.0,2.0,3.0,
  //           4.0,5.0,6.0,
  //           7.0,8.0,9.0;
  Serial.println("SVD time!");
  Vector3d normal = calc_SVD(g_vec);

  Serial.println("Initialising true_vec vars...");
  double disto_len = 0.1;
  VectorXd laser_distances(3);
  laser_distances << 20,21,19;

  Serial.println("Beginning true_vec calcs...");
  Vector3d true_vec = calc_true_vec(normal, laser_distances, disto_len);

  sprintf(buffer, "\nX: %f \nY: %f \nZ: %f\n", true_vec[0], true_vec[1], true_vec[2]);
  Serial.printf(buffer);
}

void init_bno(){
  //Initialize I2C communication
  Wire.begin();
 
  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device
 
  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
 
  delay(1);
}

enum shot_status {IDLE, WAITING, SPLAY, BASE};
static shot_status current_state = IDLE;
static shot_status next_state = IDLE;
static double distance = 0;
static double distance1 = 0;
static double distance2 = 0;
static bool interrupted = false;
// static hw_timer_t *My_timer = NULL;
bool timedout = false;


void save_splay(double distance,bool base=false)
{
  node n;
  char str_id[4];
  double heading;
  double inclination;

  magnetometer.update();
  heading = magnetometer.get_heading();
  
  accelerometer.update();
  inclination = accelerometer.get_inclination();

  n.id = new_node_id;
  n.previous = current_base_id;
  n.vector_to_prev = generate_vector(distance, heading, inclination);
  sprintf(str_id,"%d",n.id);
  write_to_file(current_file_name,str_id,(const node*)&n);
  new_node_id += 1;

  int i;
  struct node* n1 = (struct node*)malloc(sizeof(struct node));
  Serial.println("Getting file data");
  preferences.begin(current_file_name, true);
  for (i=1;i<=n.id;i++)
  {
    Serial.printf("Getting file data for name %d\n",i);
    sprintf(str_id,"%d",i);
    preferences.getBytes(str_id,n1,sizeof(struct node));
    Serial.println("Succesfully read line!");
    Serial.printf("Read line: %s\n ID: %d\n Previous ID: %d\n Prev vec: %f %f %f\n",str_id,n1->id,n1->previous,n1->vector_to_prev(0),n1->vector_to_prev(1),n1->vector_to_prev(2));
  }
  preferences.end();

}

void interrupt_loop()
{
  current_state = next_state;
  switch (current_state){
    case IDLE:
      next_state = SPLAY;
      try
      {
        distance1 = lidar.get_measurement();
      }
      catch(const std::exception& e)
      {
        Serial.println(e.what());
        next_state = IDLE;
      }
      break;
      
      
      // Serial.println("BEGINNING TIMER");
      // timerRestart(My_timer);
      // timerAlarmWrite(My_timer, 3000000, true);
      // timerAlarmEnable(My_timer);
    
    case WAITING:
    //timerAlarmDisable(My_timer);
      if (timedout = false)
      {
        next_state = BASE;
      } else {
        next_state = SPLAY;
      }
      break;

    case SPLAY:
      next_state = IDLE;
      Serial.println("Saving splay");
      save_splay(distance1);
      break;
      

    case BASE:
      next_state = IDLE;
      try
      {
        distance2 = lidar.get_measurement();
      }
      catch(const std::exception& e)
      {
        Serial.println(e.what());
        break;
      }

      if (fabs(distance1 - distance2) / (distance1 + distance2) > 1e-3)
      {
        display.clearDisplay();
        display.setCursor(0,0);
        display.write("Error in distance greater than 1%, please retry!");
      }
      else {
        distance = 0.5*(distance1 + distance2);
        save_splay(distance,true);
      }
      break;

  }
}

void IRAM_ATTR ISR_GET_SHOT()
{
  interrupted = true;
}

void setup(){
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(A0, INPUT);
  attachInterrupt(A0, ISR_GET_SHOT, RISING);
  display = init_OLED(0x3C);
  init_bno();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  magnetometer.init();
  pinMode(GPIO_NUM_14, OUTPUT);
  digitalWrite(GPIO_NUM_14,LOW);
  lidar.init();

  // My_timer = timerBegin(0, 80, true);
  // timerAttachInterrupt(My_timer, ISR_GET_SHOT, true);
  // timerAlarmWrite(My_timer, 3000000, true);
  


  Serial.println("BEGINNING LOOP");
  display.clearDisplay();
  display.setCursor(0,0);
  display.write("MAINLOOP");
  display.display();
}

void loop(){
  if (interrupted)
  {
    Serial.println("mainloop interrupt!");
    display.display();
    interrupt_loop();
    Serial.println("returning to mainloop!");
    interrupted = false;
  }
}