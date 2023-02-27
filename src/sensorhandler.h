#ifndef HEADER_SENSORHANDLER
#define HEADER_SENSORHANDLER

#include "magnetometer.h"
#include "accelerometer.h"
#include "lidar.h"
#include "utility.h"
#include <queue>


static const int SAMPLING_SIZE = 10;
static const int LASER_CALIBRATION_SIZE = 8;
static const int INERTIAL_CALIBRATION_SIZE = 100;



// Calculates the best fit plane to a set of points in 3d and returns the vector normal to this plane
Vector3f calc_normal_vec(MatrixXf point_vec, bool debug = false);
// Given sperical coordinates, heading, inclination, distance return the cartesian coordinates 
Vector3f toCartesian(Vector3f spherical);
// Given cartesan coordinates x,y,z return spherical coordinates heading, inclination, distance
Vector3f toSpherical(Vector3f cartesian);
// Given an angle, theta, in radians, produce a matrix corresponding to a rotation of theta clockwise about the x-axis
Matrix3f getXRotation(float theta);

// Catchall class for handling all sensors
class SensorHandler{
    public:
        // Updates all of the sensors and saves the data into `this`
        void update();
        // Calibrates the alignement of all sensors
        bool add_laser_calibration();
        // Returns the data of a shot [Heading, Inclination, Distance]
        Vector3f get_shot_data();
        // Returns the current inclination saved, must call update first to get newest data
        float get_inclination();
        // Returns the current heading saved, must call update first to get newest data
        float get_heading();
        // Returns the current distance saved, must call update first to get newest data
        float get_distance();
        // Calibrate accelerometer and magnetometer
        bool calibrate_inertial_sensors();
        // Main constructor, must be fed connections to all sensor objects
        SensorHandler(Accelerometer* accel, Magnetometer* mag, Lidar* lidar);

        void sensor_test();

    private:
        
        Accelerometer* accel_sensor; // Connection to accelerometer sensor
        Magnetometer* mag_sensor; // Connection to magnetometer sensor
        Lidar* lidar_sensor; // Connection to LIDAR sensor

        Eigen::Matrix<float,4,LASER_CALIBRATION_SIZE> laser_calibration_data; // Heading, Inclination, Roll, Distance
        Eigen::Matrix<float,3,INERTIAL_CALIBRATION_SIZE> mag_joint_calibration_data; // mag x,y,z;
        Eigen::Matrix<float,3,INERTIAL_CALIBRATION_SIZE> acc_joint_calibration_data; // acc x,y,z;
        
        bool acc_calibrated, mag_calibrated, imu_aligned, las_calibrated = false; // Calibration flags


        // Magnetometer and imu calibration data
        Vector3f mag_data; // Raw magnetometer data (vector) - of the device
        Vector3f grav_data; // Raw acceleration data (vector) - of the device

        // Sensor correction data
        Matrix3f acc_correction_mat;
        Matrix3f mag_correction_mat;
        Matrix3f imu_correction_mat;
        float inclination_correction; // Laser alignment inclination correction in deg WHEN UPRIGHT - MUST INVERT IF TILT > 180!
        float heading_correction; // Laser alignment heading correction in deg WHEN UPRIGHT - MUST INVERT IF TILT > 180!

        // General purpose data
        int calibration_num; // Current position in calibration process
        Vector3f device_vec; // Device vector - vector of the device
        float heading, inclination, roll; // In radians
        float distance; // In metres
        
        // Calculates the device's orientation and saves it in heading and inclination
        void get_orientation();

        // Aligns the LIDAR sensor
        void align_laser();





};

#endif