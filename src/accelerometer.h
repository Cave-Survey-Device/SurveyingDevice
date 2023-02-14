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
#include "utility.h"


using namespace Eigen;
using std::cout;

static const double CALIB_VAR_LIM = 0.025;
static const int ACCEL_CALIBRATION_N = 23;

// Computes kronecker product of two 3 vectors
Vector<double, 9> kronecker3(Vector3d a, Vector3d b);

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
    bool calibrate();

protected:
    // Gets the raw data from the underlying sensor
    virtual void get_raw_data()=0; 

    Vector3d raw_gravity_data; // Raw gravity data - un-corrected

private:
    Vector3d corrected_gravity_data; // Corrected gravity data
    Matrix3d correction_transformation; // Tranformation used to correct the gravity data

    // Acceleration calibration data
    Matrix<double,3,10> samples_mat; // Accelerometer sensor calibration data
    Matrix<double,3,ACCEL_CALIBRATION_N> calib_data; // Accelerometer sensor calibration data
    Vector3d mean_acceleration;
    int sample_num, calibration_num;
    bool calibrated;
};

#endif