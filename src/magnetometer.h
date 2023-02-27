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

        // Calculates the magnetometer HSI correction and calculates calibration matrix for accelerometer
        bool calibrate();

        // Returns the current calibration array progress
        int check_calibration_progress();

        // Returns the current heading (corrected)
        Vector3f get_mag_vec();
        
        void reset_calibration_data();

        void update();

    protected:
        // Reads data from the sensor and processes it
        virtual void get_raw_data()=0;
        Vector3f raw_mag_data; // Raw magnetic data - un-corrected

    private:
        // Given an x,y, and z value, return the index where the data should be stored in the calibration array
        int get_magnetometer_index(float x, float y,float z);
        
        Matrix<float,3,MAGNETOMETER_ARR_LEN> magnetometer_arr; // Magnetometer calibration array
        Vector3f corrected_mag_data; //Corrected magnetometer data
        Matrix3f correction_transformation; // Tranformation used to correct the magnetometer data
};

#endif