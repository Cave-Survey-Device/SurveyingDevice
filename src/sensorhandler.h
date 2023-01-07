#ifndef HEADER_SENSORHANDLER
#define HEADER_SENSORHANDLER

#include "magnetometer.h"
#include "accelerometer.h"
#include "lidar.h"

static const int SAMPLING_SIZE = 10;

class SensorHandler{
    public:
        void update();
        double get_inclination();
        double get_heading();
        double get_distance();
        SensorHandler(Accelerometer* accel, Magnetometer* mag, Lidar* lidar);

    private:
        Accelerometer* accel_sensor;
        Magnetometer* mag_sensor;
        Lidar* lidar_sensor;

        double heading;
        double inclination;
        double distance;

        Vector3d mag_data;
        Vector3d grav_data;

        double inclination_correction;
        double heading_correction;

        void get_orientation();

};

#endif