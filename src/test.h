#ifndef HEADER_TEST
#define HEADER_TEST

#include <Arduino.h>

#include <ArduinoEigenDense.h>

#include "sensors/Sensors.h"
#include "utils/utility.h"
#include "utils/NumericalMethods.h"

#include <random>
#include <math.h>
#define _USE_MATH_DEFINES


using namespace Eigen;  



class TestInertialSensorConnection: public InertialSensorConnection
{
public:
    Vector3f true_vec;
    Matrix3f T;
    Vector3f h;
    Matrix3f T_align;
    int sample_n;

    TestInertialSensorConnection(Vector3f tv, Matrix3f T_set, Vector3f h_set, Matrix3f T_align_set)
    {
        this->T = T_set;
        this->h = h_set;
        this->T_align = T_align_set;
        this->true_vec = tv;
        this->sample_n = 0;
    }

    Vector3f GetRawData()
    {
        std::default_random_engine generator;
        std::normal_distribution<float> distribution(0,0.005);
        Vector3f sample;
        Vector3f noise;

        VectorXf x_rots(12);
        x_rots << 0, 0 , 180, 180, 270, 270, 270, 90 , 0  , 0  , 0  , 0;
        VectorXf y_rots(12);
        y_rots << 0, 0 , 90 , 90 , 0  , 0  , 180, 0  , 270, 270, 90 , 90;
        VectorXf z_rots(12);
        z_rots << 0, 90, 0  , 45 , 270, 0  , 0  , 225, 90 , 180, 180, 225;
        Matrix3f Rx, Ry, Rz, R;

        Rx = x_rotation(x_rots(sample_n%12));
        Ry = y_rotation(y_rots(sample_n%12));
        Rz = z_rotation(z_rots(sample_n%12));
        R = Rx * Ry * Rz;

        noise << distribution(generator), distribution(generator), distribution(generator);
        sample = (T * R * this->true_vec + h + noise);

        sample_n++;
        return sample;
    }
};

class TestSensorHandler: public SensorHandler
{
public:
    TestSensorHandler(InertialSensor* accel, InertialSensor* mag): SensorHandler(accel,mag){}

    bool CollectCalibrationData()
    {
        for(int i=0; i<ORIENTATIONS; i++)
        {
            for(int j=0; j<SAMPLES_PER_ORIENTATION; j++)
            {
                this->magnetometer->ColectCalibrationSample();
                this->accelerometer->ColectCalibrationSample();
            } 
        }
        return 1;
    }

};

void test_main(void * parameter)
{
    while(true)
    {
    Matrix3f Ta;
    Matrix3f Tm;
    Matrix3f TMmisalign;
    Vector3f ha;
    Vector3f hm;
    Matrix3f TAmisalign;

    Tm <<  0.462,-0.0293,-0.037,
    0.0686,0.4379,0.0303,
    0.0427,-0.0336,0.4369;
    hm << -0.176,0.2214,0.0398;

    Ta <<  9.77,0.0018,-0.030,
    0.0019,9.7032,-0.0011,
    -0.0087, -0.0013,9.6927;
    Ta = Ta * 0.1;
    ha << -0.01472,-0.0011,-0.01274;

    TMmisalign << 1.,0.,0. ,0.,1.,0., 0.,0.,1.;
    TAmisalign << 1.,0.,0. ,0.,1.,0., 0.,0.,1.;

    Vector3f mag_true_vec, acc_true_vec;
    mag_true_vec << 1.0,0.0,0.0;
    acc_true_vec << 0.0,0.0,1.0;

    static TestInertialSensorConnection mag_sc(mag_true_vec, Tm, hm, TMmisalign);
    static TestInertialSensorConnection acc_sc(acc_true_vec, Ta, ha, TAmisalign);
    static InertialSensor mag(&mag_sc);
    static InertialSensor acc(&acc_sc);
    static TestSensorHandler sh(&acc, &mag);


    Serial.print("Resetting calibration\n");
    sh.ResetCalibration();

    Serial.print("Collecting calibration data4\n");
    sh.CollectCalibrationData();

    Serial.print("Calibrating data\n");
    sh.CalibrateInertial();
    // Serial.print("Aligning data\n");
    // sh.AlignInertial();

    Serial << "AccelPtr: " << (int)sh.GetAccelPtr() << "\nMagPtr: " << (int)sh.GetMagPtr() << "\n";

    Serial << "Initial Ta:\n";
    displayMat(Ta);

    Serial << "Initial ha:\n";
    displayVec(ha);

    Serial << "Calculated Ta:\n";
    displayMat(sh.GetAccelPtr()->GetT());

    Serial << "Calculated ha:\n";
    displayVec(sh.GetAccelPtr()->Geth());

    Serial << "---------------------------------------------------\n\n";

    Serial << "Initial Tm:\n";
    displayMat(Tm);

    Serial << "Initial hm:\n";
    displayVec(hm);

    Serial << "Calculated Tm:\n";
    displayMat(sh.GetMagPtr()->GetT());

    Serial << "Calculated hm:\n";
    displayVec(sh.GetMagPtr()->Geth());
    // Serial.print(buffer.out);
    // sh.CollectAlignmentData();
    // sh.AlignLaser();

    Serial << "Mag calib data: \n";
    displayMat(mag.GetCalibData().transpose());

    Serial << "Mag correction data: \n";
    displayMat(     (sh.GetMagPtr()->GetT() * (mag.GetCalibData().colwise() - sh.GetMagPtr()->Geth())).transpose()    );


    Serial << "Accel calib data: \n";
    displayMat(acc.GetCalibData().transpose());

    Serial << "Accel correction data: \n";
    displayMat(     (sh.GetAccelPtr()->GetT() * (acc.GetCalibData().colwise() - sh.GetAccelPtr()->Geth())).transpose()    );

    delay(10000);
    }
}


#endif