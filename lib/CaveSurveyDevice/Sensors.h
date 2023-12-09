#ifndef CAVESURVEYDEVICE_SENSORS_H
#define CAVESURVEYDEVICE_SENSORS_H

#include <ArduinoEigen.h>
using namespace Eigen;

class Accelerometer
{
public:
    virtual Vector3f getMeasurement()=0;
    virtual void init()=0;

};

class Magnetometer
{
public:
    virtual Vector3f getMeasurement()=0;
    virtual void init()=0;
};

class Laser
{
public:
    virtual float getMeasurement()=0;
    virtual void toggleLaser()=0;
    virtual void toggleLaser(bool mode)=0;
    virtual void init()=0;
};

#endif