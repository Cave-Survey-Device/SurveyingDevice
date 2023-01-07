#ifndef HEADER_SENSORHANDLER
#define HEADER_SENSORHANDLER

#include "magnetometer.h"
#include "accelerometer.h"
#include "lidar.h"
#include "lasercalibration.h"
#include "unified.h"

static const int SAMPLING_SIZE = 10;
static const int CALIBRATION_SIZE = 8;

class SensorHandler{
    public:
        void update();
        bool calibrate();
        double get_inclination();
        double get_heading();
        double get_distance();
        SensorHandler(Accelerometer* accel, Magnetometer* mag, Lidar* lidar);

    private:
        Accelerometer* accel_sensor;
        Magnetometer* mag_sensor;
        Lidar* lidar_sensor;


        Eigen::Matrix<double,3,CALIBRATION_SIZE> calibraion_tilt_vecs;
        Eigen::Vector<double,CALIBRATION_SIZE> calibration_distances;

        double heading;
        double inclination;
        double distance;

        Vector3d mag_data;
        Vector3d grav_data;

        double inclination_correction;
        double heading_correction;

        int calibration_num;

        void get_orientation();

};

#endif