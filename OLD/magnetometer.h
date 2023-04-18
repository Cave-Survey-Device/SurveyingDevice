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

        // Returns the current heading (corrected)
        Vector3f get_mag_vec();

    protected:
        // Reads data from the sensor and processes it
        virtual void get_raw_data()=0;
        Vector3f raw_mag_data; // Raw magnetic data - un-corrected
};

#endif