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

static const float CALIB_VAR_LIM = 0.025;
static const int ACCEL_CALIBRATION_N = 6;
const int N_ACC_SAMPLES = 10;

// Computes kronecker product of two 3 vectors
MatrixXf kronecker(MatrixXf a, MatrixXf b);

class Accelerometer  {
public:
    // Default constructor
    Accelerometer();

    // Reads data from the sensor and processes it
    void update();

    // Returns the current heading (corrected)
    Vector3f get_grav_vec();

    // Returns rotation about y axis undergone by device
    float get_inclination();

    // Calculate calibration matrix for accelerometer
    bool calibrate();

    // Test calibration process  for accelerometer
    bool test_calibration();

protected:
    // Gets the raw data from the underlying sensor
    virtual void get_raw_data()=0; 

    Vector3f raw_gravity_data; // Raw gravity data - un-corrected

private:
    Vector3f corrected_gravity_data; // Corrected gravity data
    Matrix3f correction_transformation; // Tranformation used to correct the gravity data

    // Acceleration calibration data
    Matrix<float,3,N_ACC_SAMPLES> samples_mat; // Accelerometer sensor calibration data
    Matrix<float,3,ACCEL_CALIBRATION_N> calib_data; // Accelerometer sensor calibration data

    RowVector<float,12+4*ACCEL_CALIBRATION_N> calculate_newton_iteration(Vector<float,12+4*ACCEL_CALIBRATION_N> theta);
    void run_newton(Matrix<float,3,ACCEL_CALIBRATION_N> ym);

    int sample_num, calibration_num;
    bool calibrated;
};

#endif