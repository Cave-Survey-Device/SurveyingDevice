#ifndef HEADER_SENSORHANDLER
#define HEADER_SENSORHANDLER

#include "magnetometer.h"
#include "accelerometer.h"

class SensorHandler{
    public:
        void update();
        Vector2d get_orientation();
        SensorHandler(struct bno055_gravity *myGravityData, struct bno055_mag *myMagData);

    private:
        Accelerometer accel_sensor;
        Magnetometer mag_sensor;
}

#endif