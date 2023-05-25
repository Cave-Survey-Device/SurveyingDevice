
#ifndef HEADER_BEAN_BOI_SENSORS
#define HEADER_BEAN_BOI_SENSORS


#include <ArduinoEigenDense.h>
#include "inertialsensor.h"
#include "accelerometer_csd.h"
#include "magnetometer_csd.h"
#include "sensor_config_csd.h"
using namespace Eigen;


class LaserSensor
{
public:
    // Initialise lidar module
    virtual void init()=0;

    // get lidar mesaurement
    virtual float getMeasurement()=0;

    // Toggle laser
    virtual void toggleLaser(bool mode)=0;
};

/**
 * @brief The sensorhandler class functions as a manager for all connected sensors. This includes a magnetometer, accelerometer, and laser distance sensor. SensorHandler requires implementations of the generic class InertialSensor to connect it to a magnetometer and an accelerometer. Additionally, an implementation of LaserSensor is required to add connection to a laser distance sensor.
 * 
 * This class is used to handle ALL sensor functions. This includes calibration, alignment, getting readings, etc.
 */
class SensorHandler
{
protected:
    Matrix<float,3,N_LASER_CAL> laser_alignment_data; // Alignment data, heading, inclination, roll, distance
    float laser_inclination_alignment; // Value to be ADDED to value of sensors to align laser and inertial sensors
    float laser_heading_alignment; // Value to be ADDED to value of sensors to align laser and inertial sensors
    int laser_alignment_progress;
    Matrix3f laser_alignment_mat;

    Matrix3f inertial_alignment_mat;
    float inclination_angle;

    Accelerometer* accelerometer; // Connection to accelerometer sensor
    Magnetometer* magnetometer; // Connection to magnetometer sensor
    LaserSensor* laser; // Connection to LIDAR sensor

    bool using_laser;

public:


    /**
     * @brief Construct a new SensorHandler object with a magnetometer, accelerometer, and laser sensor
     * 
     * @param acc Pointer to the connected accelerometer object
     * @param mag Pointer to the connected magnetometer object
     * @param las Pointer to the connected laser sensor object
     */
    SensorHandler(Accelerometer* acc, Magnetometer* mag, LaserSensor* las);

    /**
     * @brief Construct a new SensorHandler object with a magnetometer and accelerometer
     * 
     * @param acc Pointer to the connected accelerometer object
     * @param mag Pointer to the connected magnetometer object
     */
    SensorHandler(Accelerometer* acc, Magnetometer* mag);

    /**
     * @brief Default SensorHandler constructor
     * 
     */
    SensorHandler();
 
    /**
     * @brief Calibrate the inertial sensors - accelerometer and magnetometer. This runs a one-off calibration using the stored calibration data in each InertialSensor bject respectively.
     */
    void calibrateInertial();

    /**
     * @brief Runs a one-off alignment of the inertial sensors using the stored calibration data in each InertialSensor respectively.
     */
    void alignInertial();

    /**
     * @brief Runs a one-off alignment of the laser and inertial sensors using the stored calibration data in each alignment_data respectively.
     */
    void alignLaser();   

    /**
     * @brief Collects one shot of laser alignment data.
     * 
     * @return true if all alignment shots HAVE been collected
     * @return false if all alignment shots HAVE NOT been collected
     */
    bool collectLaserAlignmentData();   

    /**
     * @brief Collects one orientation of calibration data.
     * 
     * @return true if 12 orientations have been sampled.
     * @return false if less than 12 orientations have been sampled.
     */
    int collectInertialAlignmentData(); 
    
    /**
     * @brief Resets the clalibration data and progress. MUST be run before calibration.
     */
    void resetCalibration();

    /**
     * @brief Loads calibration data from file
     */
    void loadCalibration();

    /**
     * @brief Resets alignment data and progress. MUST be run before alignment.
     */
    void resetAlignment();

    /**
     * @brief Loads alignment data from file
     */
    void loadAlignment();

    /**
     * @brief Enables the laser dot on the laser sensor
     */
    void enableLaser();

    /**
     * @brief Disables the laser dot on the laser sensor
     */
    void disableLaser();

    /**
     * @brief Updates the sensors and gets a reading from all sensors
     * 
     * @return Vector3f Heading [deg], Inclination [deg], Distance [m]
     */
    Vector3f getReading();

    /**
     * @brief Saves inertial alignment data to non-volatile storage in a seperate location
     * This is to allow form calibration data to be generated and discarded if necessary.
     * 
     */
    void save_tmp_inertial_align_data();

    InertialSensor* getAccelPtr();
    InertialSensor* getMagPtr();
    LaserSensor* getLaserPtr();
};

#endif