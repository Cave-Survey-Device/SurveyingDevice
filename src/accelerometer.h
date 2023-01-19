#ifndef HEADER_ACCELEROMETER
#define HEADER_ACCELEROMETER

#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigenSparse.h>

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <BNO055_support.h>
#include <Wire.h>

#include <iostream>
#include "config.h"
#include "unified.h"


using namespace Eigen;
using std::cout;


class Accelerometer  {
public:
    // Default constructor
    Accelerometer(struct bno055_gravity *myGravityData);

    // Reads data from the sensor and processes it
    void update();

    // Returns the current heading (corrected)
    Vector3d get_grav_vec();

    // Returns rotation about y axis undergone by device
    double get_inclination();

    // Calculate calibration matrix for accelerometer
    void Accelerometer::calibrate();

private:
    // Pointer to the struct containing sensor data
    struct bno055_gravity *sensor_connection;

    // Raw gravity data - un-corrected
    Vector3d raw_gravity_data;

    //Corrected gravity data
    Vector3d corrected_gravity_data;

    // Tranformation used to correct the gravity data
    Matrix3d correction_transformation;
        
};

#endif