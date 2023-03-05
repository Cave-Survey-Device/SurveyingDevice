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

    void calculate_newton_iteration();
    void run_newton(Matrix<float,3,ACCEL_CALIBRATION_N> ym);

    

    // Calibration variables
    Vector<float,9> J_1;
    Vector3f J_2;
    RowVector<float,3*ACCEL_CALIBRATION_N> J_3;
    RowVector<float,ACCEL_CALIBRATION_N> J_4;
    Matrix<float,9,9> H11;
    Matrix<float,9,3> H12;
    Matrix<float,3,9> H21;
    Matrix<float,9,3*ACCEL_CALIBRATION_N> H13;
    Matrix<float,3*ACCEL_CALIBRATION_N,9> H31;
    Matrix<float,9,ACCEL_CALIBRATION_N> H14;
    Matrix<float,ACCEL_CALIBRATION_N,9> H41;
    Matrix3f H22;
    Matrix<float,3,3*ACCEL_CALIBRATION_N> H23;
    Matrix<float,3*ACCEL_CALIBRATION_N,3> H32;
    Matrix<float,3,ACCEL_CALIBRATION_N> H24;
    Matrix<float,ACCEL_CALIBRATION_N,3> H42;
    Matrix<float,3*ACCEL_CALIBRATION_N,3*ACCEL_CALIBRATION_N> H33;
    Matrix<float,3*ACCEL_CALIBRATION_N, ACCEL_CALIBRATION_N> H34;
    Matrix<float,ACCEL_CALIBRATION_N, 3*ACCEL_CALIBRATION_N> H43;
    Matrix<float,ACCEL_CALIBRATION_N,ACCEL_CALIBRATION_N> H44;

    // Composite variables
    RowVector<float, 12+4*ACCEL_CALIBRATION_N> jacobian;
    Matrix<float, 12+4*ACCEL_CALIBRATION_N, 12+4*ACCEL_CALIBRATION_N> hessian;
    Vector<float,12+4*ACCEL_CALIBRATION_N> theta;
    Vector<float,12+4*ACCEL_CALIBRATION_N> theta2;

    // Component variables
    Matrix3f T_a;
    Vector3f b_a;
    Matrix<float,3,ACCEL_CALIBRATION_N> yb; // Body frame
    Matrix<float,3,ACCEL_CALIBRATION_N> ym; // Actual readings
    Vector<float,ACCEL_CALIBRATION_N> lambda;
    Matrix<float,3,ACCEL_CALIBRATION_N> d;

    // Iterator variables
    Vector3f ym_k;
    Vector3f yb_k;
    Vector3f d_k;
    float lambda_k;

    // Temp vars
    Matrix<float,3,ACCEL_CALIBRATION_N> J_3_mat;
    Vector<float,9> tempV;
    Matrix3f ykyk;

    // Other
    Eigen::FullPivLU<Matrix<float,12+4*ACCEL_CALIBRATION_N,12+4*ACCEL_CALIBRATION_N>> fplu;

    int sample_num, calibration_num;
    bool calibrated;
};

#endif