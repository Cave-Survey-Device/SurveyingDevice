#ifndef HEADER_SENSORHANDLER
#define HEADER_SENSORHANDLER

#include "magnetometer.h"
#include "accelerometer.h"
#include "lidar.h"
#include "utility.h"

static const int SAMPLING_SIZE = 10;
static const int LASER_CALIBRATION_SIZE = 8;
static const int INERTIAL_CALIBRATION_SIZE = 100;

// Calculates the best fit plane to a set of points in 3d and returns the vector normal to this plane
Vector3d calc_normal_vec(MatrixXd point_vec, bool debug = false);
// Given sperical coordinates, heading, inclination, distance return the cartesian coordinates 
Vector3d toCartesian(Vector3d spherical);
// Given cartesan coordinates x,y,z return spherical coordinates heading, inclination, distance
Vector3d toSpherical(Vector3d cartesian);
// Given an angle, theta, in radians, produce a matrix corresponding to a rotation of theta clockwise about the x-axis
Matrix3d getXRotation(double theta);

// Catchall class for handling all sensors
class SensorHandler{
    public:
        // Updates all of the sensors and saves the data into `this`
        void update();

        // Calibrates the alignement of all sensors
        bool add_laser_calibration();

        // Returns the data of a shot [Heading, Inclination, Distance]
        Vector3d get_shot_data();

        // Returns the current inclination saved, must call update first to get newest data
        double get_inclination();
        // Returns the current heading saved, must call update first to get newest data
        double get_heading();
        // Returns the current distance saved, must call update first to get newest data
        double get_distance();

        // Calibrate accelerometer and magnetometer
        void calibrate_inertial_sensors();

        // Main constructor, must be fed connections to all sensor objects
        SensorHandler(Accelerometer* accel, Magnetometer* mag, Lidar* lidar);

        void sensor_test();

    private:
        // Connection to accelerometer sensor
        Accelerometer* accel_sensor;
        // Connection to magnetometer sensor
        Magnetometer* mag_sensor;
        // Connection to LIDAR sensor
        Lidar* lidar_sensor;

        // Heading, Inclination, Roll, Distance
        Eigen::Matrix<double,4,LASER_CALIBRATION_SIZE> laser_calibration_data;
        // Heading, Inclination, Roll, Distance
        Eigen::Matrix<double,4,INERTIAL_CALIBRATION_SIZE> inertial_calibration_data;
        

        // In radians
        double heading, inclination, roll;
        // In metres
        double distance;

        // Raw magnetometer data (vector) - of the device
        Vector3d mag_data;
        // Raw acceleration data (vector) - of the device
        Vector3d grav_data;
        // Device vector - vector of the device
        Vector3d device_vec;

        // Laser alignment vector
        Vector3d laser_alignment_vector;

        // Accelerometer calibration matrix
        Matrix3d acc_calibration_mat;
        // Calibration vector
        Matrix3d mag_calibration_mat;

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