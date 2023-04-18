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
#include <random>


using namespace Eigen;
using std::cout;

static const float CALIB_VAR_LIM = 0.025;

static const int yrots = 4;
static const int xrots = 2;
static const int N_samples = 10;
static const int ym_size = yrots*xrots*N_samples;

static const int ACCEL_CALIBRATION_N = ym_size;
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

    
    // Composite variables
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

    int x,y,i;
    float x_ang, y_ang;
    Vector3f g_vec = {0, 0, 1};
    Matrix3f xrotation_mat;
    Matrix3f yrotation_mat;
    Vector3f noise;
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> norm_dist{0, 0.05};

    // Other
    Eigen::FullPivLU<Matrix<float,12+4*ACCEL_CALIBRATION_N,12+4*ACCEL_CALIBRATION_N>> fplu;

    int sample_num, calibration_num;
    bool calibrated;
};

#endif