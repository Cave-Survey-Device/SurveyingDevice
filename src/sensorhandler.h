#ifndef HEADER_SENSORHANDLER
#define HEADER_SENSORHANDLER

#include "magnetometer.h"
#include "accelerometer.h"
#include "lidar.h"
#include "unified.h"

static const int SAMPLING_SIZE = 10;
static const int CALIBRATION_SIZE = 8;

// Calculates the best fit plane to a set of points in 3d and returns the vector normal to this plane
Vector3d calc_normal_vec(MatrixXd point_vec, bool debug = false);

// Catchall class for handling all sensors
class SensorHandler{
    public:
        // Updates all of the sensors and saves the data into `this`
        void update();

        // Calibrates the alignement of all sensors
        bool calibrate();

        // Returns the data of a shot [Heading, Inclination, Distance]
        Vector3d get_shot_data();

        // Returns the current inclination saved, must call update first to get newest data
        double get_inclination();
        // Returns the current heading saved, must call update first to get newest data
        double get_heading();
        // Returns the current distance saved, must call update first to get newest data
        double get_distance();

        // Main constructor, must be fed connections to all sensor objects
        SensorHandler(Accelerometer* accel, Magnetometer* mag, Lidar* lidar);

    private:
        // Connection to accelerometer sensor
        Accelerometer* accel_sensor;

        // Connection to magnetometer sensor
        Magnetometer* mag_sensor;

        // Connection to LIDAR sensor
        Lidar* lidar_sensor;

        // Heading, Inclination, Roll, Distance
        Eigen::Matrix<double,4,CALIBRATION_SIZE> device_calibration_data;

        double heading;
        double inclination;
        double roll;
        double distance;

        // Raw magnetometer data (vector) - of the device
        Vector3d mag_data;
        // Raw acceleration data (vector) - of the device
        Vector3d grav_data;
        // Device vector - vector of the device
        Vector3d device_vec;
        // Calibration vector
        Vector3d calibration_vector;

        // Laser alignment inclination correction in deg WHEN UPRIGHT - MUST INVERT IF TILT > 180!
        double inclination_correction;
        // Laser alignment heading correction in deg WHEN UPRIGHT - MUST INVERT IF TILT > 180!
        double heading_correction;

        // Calculates the device's orientation and saves it in heading and inclination
        void get_orientation();

        // Aligns the LIDAR sensor
        void align_laser();


        // Current position in calibration process
        int calibration_num;


};

#endif