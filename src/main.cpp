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
#include "OLED.h"
#include "config.h"

// Defining global variables
static Adafruit_SSD1306 display;
static float tilt_arr[3];
static float orientation_arr[3];

static struct bno055_t myBNO;
static struct bno055_gravity myGravityData;
static struct bno055_mag myMagData;

static Magnetometer magnetometer(&myMagData);

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

void setup(){
  test_lasercalibration();
  Serial.println("Done!");
  display = init_OLED(0x3C);
  init_bno();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  magnetometer.init();
}

void loop(){
  char buffer[100];
  // get_bno();
  display.clearDisplay();

  // Magnetometer update code
  magnetometer.update();
  magnetometer.add_calibration_data();
  Vector3d mag_data = magnetometer.get_heading();

  // Accelerometer update code


  Serial.print("Progress: ");
  Serial.println(magnetometer.check_calibration_progress());


  Serial.print("X: ");
  Serial.println(float(mag_data(0)));

  Serial.print("Y: ");
  Serial.println(float(mag_data(1)));

  Serial.print("Z: ");
  Serial.println(float(mag_data(2)));
  Serial.println("");



  display.print("Progress: ");
  display.println(magnetometer.check_calibration_progress());


  display.print("X: ");
  display.println(float(mag_data(0)));

  display.print("Y: ");
  display.println(float(mag_data(1)));

  display.print("Z: ");
  display.println(float(mag_data(2)));
  display.println("");

  
  display.setCursor(0,0);
  display.print(buffer);
  display.display();

  delay(1000);
}