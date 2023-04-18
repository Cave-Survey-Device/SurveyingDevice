#include <Arduino.h>
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigenSparse.h>
#include "sensors/Sensors.h"
#include <random>
#include <math.h>
#define _USE_MATH_DEFINES
#include "utils/utility.h"
using namespace Eigen;  

#define USING

#include <Arduino.h>
#undef max
#undef min
#include <stdio.h>
#include <iostream>

using namespace std;

extern "C" {
int _write(int fd, char *ptr, int len) {
  (void) fd;
  return Serial.write(ptr, len);
}
}

float Deg2Rad(float degrees) {
    return degrees * (M_PI / 180.0);
}

Matrix3f x_rotation(float deg)
{
    deg = Deg2Rad(deg);
    Matrix3f R;
    R << cos(deg), 0., sin(deg),
            0., 1., 0.,
            -sin(deg), 0., cos(deg);
    return R;
}

Matrix3f y_rotation(float deg)
{
    deg = Deg2Rad(deg);
    Matrix3f R;
    R << 1., 0., 0.,
            0., cos(deg), -sin(deg),
            0., sin(deg), cos(deg);
    return R;
}

Matrix3f z_rotation(float deg)
{
    deg = Deg2Rad(deg);
    Matrix3f R;
    R << cos(deg), -sin(deg), 0.,
            sin(deg), cos(deg), 0.,
            0., 0. , 1.;
    return R;
}


class TestInertialSensor: public InertialSensor
{
public:
    Vector3f true_vec;
    Matrix3f T;
    Vector3f h;
    Matrix3f T_align;

    TestInertialSensor(Matrix3f T_set, Vector3f h_set, Matrix3f T_align_set)
    {
        T = T_set;
        h = h_set;
        T_align = T_align_set;
    }

    Vector3f GetRawData()
    {
        std::default_random_engine generator;
        std::normal_distribution<float> distribution(0,0.005);
        Vector3f sample;
        Vector3f noise;

        noise << distribution(generator),distribution(generator),distribution(generator);
        sample = T_align * (T * true_vec + h + noise);

        return sample;
    }

    Matrix3f GetT()
    {
        return calibration_matrix;
    }

    Vector3f Geth()
    {
        return calibration_offset;
    }

};


class TestSensorHandler: public SensorHandler
{
public:
    TestInertialSensor* magnetometer;
    TestInertialSensor* accelerometer;

    TestSensorHandler(TestInertialSensor* accel, TestInertialSensor* mag)
    {
        magnetometer = mag;
        accelerometer = accel;
    }

    bool CollectCalibrationData()
    {
        Vector3f mag_true_vec, acc_true_vec;
        mag_true_vec << 1,0,0;
        acc_true_vec << 0,0,1;

        VectorXf x_rots(12);
        x_rots << 0, 0 , 180, 180, 270, 270, 270, 90 , 0  , 0  , 0  , 0;
        VectorXf y_rots(12);
        y_rots << 0, 0 , 90 , 90 , 0  , 0  , 180, 0  , 270, 270, 90 , 90;
        VectorXf z_rots(12);
        z_rots << 0, 90, 0  , 45 , 270, 0  , 0  , 225, 90 , 180, 180, 225;
        Matrix3f Rx, Ry, Rz, R;

        for(int i=0; i<ORIENTATIONS; i++)
        {
            Rx = x_rotation(x_rots(i));
            Ry = y_rotation(y_rots(i));
            Rz = z_rotation(z_rots(i));
            R = Rx * Ry * Rz;
            for(int j=0; j<SAMPLES_PER_ORIENTATION; j++)
            {

                magnetometer->true_vec = R * mag_true_vec;
                accelerometer->true_vec = R * acc_true_vec;

                magnetometer->ColectCalibrationSample();
                accelerometer->ColectCalibrationSample();
            } 
        }
        return 1;
    }

};

int test_main()
{
    Matrix3f Ta, Tm, Tmisalign;
    Vector3f ha, hm;

    Tm <<  0.462,-0.0293,-0.037,
    0.0686,0.4379,0.0303,
    0.0427,-0.0336,0.4369;
    hm << -0.176,0.2214,0.0398;

    Ta <<  9.77,0.0018,-0.030,
    0.0019,9.7032,-0.0011,
    -0.0087, -0.0013,9.6927;
    Ta = Ta * 0.1;
    ha << -0.01472,-0.0011,-0.01274;

    Tmisalign = Matrix3f::Identity();

    TestInertialSensor mag(Tm, hm, Tmisalign);
    TestInertialSensor acc(Ta, ha, Matrix3f::Identity());
    TestSensorHandler sh(&acc, &mag);

    sh.CollectCalibrationData();
    sh.CalibrateInertial();
    sh.AlignInertial();


    cout << "Initial Ta:\n" << Ta << "\n\n";
    cout << "Initial ha:\n" << ha << "\n\n";
    cout << "Calculated Ta:\n" << sh.accelerometer->GetT() << "\n\n";
    cout << "Calculated ha:\n" << sh.accelerometer->Geth() << "\n\n";
    cout << "---------------------------------------------------\n\n";
    cout << "Initial Tm:\n" << Tm << "\n\n";
    cout << "Initial hm:\n" << hm << "\n\n";
    cout << "Calculated Tm:\n" << sh.magnetometer->GetT() << "\n\n";
    cout << "Calculated hm:\n" << sh.magnetometer->Geth() << "\n\n";
    return 0;
    // sh.CollectAlignmentData();
    // sh.AlignLaser();
}
