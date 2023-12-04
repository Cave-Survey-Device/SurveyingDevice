#ifndef CAVESURVEYDEVICE_SENSORS_H
#define CAVESURVEYDEVICE_SENSORS_H

#include <ArduinoEigen.h>
using namespace Eigen;

class Accelerometer
{
public:
    Vector3f getReading();
    Vector3f getSingleSample();
};

class Magnetometer
{
public:
    Vector3f getReading();
    Vector3f getSingleSample();
};

class Laser
{
public:
    Vector3f getReading();
};

#endif