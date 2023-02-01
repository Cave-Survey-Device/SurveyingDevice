#ifndef HEADER_MAGNETOMETER
#define HEADER_MAGNETOMETER

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

const static int MAGNETOMETER_ARR_LEN = 36*18;

// Abstract function to allow usage of different magnetometers
class Magnetometer  {
    public:
        // Default constructor
        Magnetometer();

        // Reads the current raw magnetometer data into the calibration array
        void add_calibration_data();

        // Calculates the magnetometer HSI correction and Calculates calibration matrix for accelerometer
        void calibrate();

        // returns the current calibration array progress
        int check_calibration_progress();

        // Returns the current heading (corrected)
        Vector3d get_mag_vec();
        
        void reset_calibration_arr();

        // Reads data from the sensor and processes it
        virtual void update()=0;


    private:
        // Given an x,y, and z value, return the index where the data should be stored in the calibration array
        int get_magnetometer_index(double x, double y,double z);

        // Magnetometer calibration array
        Matrix<double,3,MAGNETOMETER_ARR_LEN> magnetometer_arr;

    protected:
        // Raw magnetic data - un-corrected
        Vector3d raw_mag_data;

        //Corrected magnetometer data
        Vector3d corrected_mag_data;

        // Tranformation used to correct the magnetometer data
        Matrix3d correction_transformation;

        
};

#endif