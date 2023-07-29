
#ifndef HEADER_BEAN_BOI_SENSORS
#define HEADER_BEAN_BOI_SENSORS


#include <ArduinoEigenDense.h>
#include "inertialsensor.h"
#include "accelerometer_csd.h"
#include "magnetometer_csd.h"
#include "sensor_config_csd.h"
#include "laser_csd.h"
using namespace Eigen;


/**
 * @brief The sensorhandler class functions as a manager for all connected sensors. This includes a magnetometer, accelerometer, and laser distance sensor. SensorHandler requires implementations of the generic class InertialSensor to connect it to a magnetometer and an accelerometer. Additionally, an implementation of LaserSensor is required to add connection to a laser distance sensor.
 * 
 * This class is used to handle ALL sensor functions. This includes calibration, alignment, getting readings, etc.
 */
class SensorHandler
{
protected:
    Matrix<float,3,N_LASER_CAL> laser_alignment_data; /** Alignment data, heading, inclination, distance */
    float laser_inclination_alignment; /** Value to be ADDED to value of sensors to align laser and inertial sensors */
    float laser_heading_alignment; /** Value to be ADDED to value of sensors to align laser and inertial sensors */
    int laser_alignment_progress; /** Current progress in alignment of laser */

    Matrix3f laser_alignment_mat; /** Alignment matrix to correctly align the laser with the body frame*/
    Matrix3f inertial_alignment_mat /** Alignment matrix, R, to align the magnetometer with the accelerometer*/;
    float inclination_angle; /** Magnetic inclination angle calculated from calibration*/



    Vector3f mag_data; /** Data collected from the magnetometer*/
    Vector3f accel_data; /** Data collected from the accelerometer*/
    float laser_data; /** Data collected from the laser*/

    Vector3f device_orientation; /** Device orientation: Heading [deg], Inclination [deg], Roll [deg] */
    Vector3f shot_data; /** Stored data for most recent shot Heading [deg], Inclination [deg], Distance [m] */

    Accelerometer* accelerometer; /** Connection to accelerometer sensor */
    Magnetometer* magnetometer; /** Connection to magnetometer sensor */
    LaserSensor* laser; /** Connection to LIDAR sensor */

    bool using_laser; /** Using a laser on the device?*/

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
 

    // -------------------------------------------------- Inertial Calibration Functions -------------------------------------------------- //
    /**
     * @brief Collects one orientation of calibration data.
     * 
     * @return true if 12 orientations have been sampled.
     * @return false if less than 12 orientations have been sampled.
     */
    int collectCalibration(); 

    /**
     * @brief Runs a one-off calibration of the inertial sensors using the stored calibration data in each InertialSensor respectively.
     */
    void calibrate();

    /**
     * @brief Loads calibration data from file
     */
    void loadCalibration();

    /**
     * @brief Saves calibration data to file
     */
    void saveCalibration();

    /**
     * @brief Resets the clalibration data and progress. MUST be run before calibration.
     */
    void resetCalibration();


    // -------------------------------------------------- Laser Calibration Functions -------------------------------------------------- //
    /**
     * @brief Runs a one-off alignment of the laser and inertial sensors using the stored calibration data in each alignment_data respectively.
     */
    void align();   

    /**
     * @brief Collects one shot of laser alignment data.
     * 
     * @return true if all alignment shots HAVE been collected
     * @return false if all alignment shots HAVE NOT been collected
     */
    bool collectAlignment();
       
    /**
     * @brief Loads alignment data from file
     */
    void loadAlignment();

    /**
     * @brief Saves alignment data to file
     */
    void saveAlignment();

    /**
     * @brief Resets alignment data and progress. MUST be run before alignment.
     */
    void resetAlignment();


    // -------------------------------------------------- Laser Operation Functions -------------------------------------------------- //
    /**
     * @brief Enables the laser dot on the laser sensor
     */
    void enableLaser();

    /**
     * @brief Disables the laser dot on the laser sensor
     */
    void disableLaser();


    // -------------------------------------------------- Compound Sensor Functions -------------------------------------------------- //
    /**
     * @brief Updates the sensors and gets a reading from all sensors
     * 
     * @return Vector3f Heading [deg], Inclination [deg], Roll [deg]
     */
    Vector3f update();

    /**
     * @brief Updates the sensors and gets a reading from all sensors
     * 
     * @return Vector3f Heading [deg], Inclination [deg], Distance [m]
     */
    Vector3f takeShot();

    /**
     * @brief Returns sensor data from last update
     * 
     * @return Vector3f Heading [deg], Inclination [deg], Distance [m]
     */
    Vector3f getShotData();

    /**
     * @brief Get pointer to the accelerometer associated with this sensorhandler
     * 
     * @return InertialSensor* 
     */
    InertialSensor* getAccelPtr();

    /**
     * @brief Get pointer to the magnetometer associated with this sensorhandler
     * 
     * @return InertialSensor* 
     */
    InertialSensor* getMagPtr();

    /**
     * @brief Get pointer to the laser associated with this senorhandler
     * 
     * @return LaserSensor* 
     */
    LaserSensor* getLaserPtr();
};

#endif