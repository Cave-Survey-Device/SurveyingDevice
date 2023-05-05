
#ifndef HEADER_BEAN_BOI_SENSORS
#define HEADER_BEAN_BOI_SENSORS


#include <ArduinoEigenDense.h>

using namespace Eigen;

#define SAMPLES_PER_ORIENTATION 15
#define ORIENTATIONS 12
#define N_CALIB ORIENTATIONS*SAMPLES_PER_ORIENTATION
#define N_ALIGNMENT 8
#define SAMPLES_PER_READING 5
#define CALIBRATION_STDEV_MIN 0.01
#define N_CALIB_STDEV 5


class InertialSensorConnection
{
public:
    virtual Vector3f GetRawData();
};

class InertialSensor
{
protected:
    Matrix3f calibration_matrix;
    Vector3f calibration_offset;
    Vector3f calibrated_data;
    Vector3f raw_data;
    Matrix<float,3,N_CALIB> calibration_data;
    int calib_num;

    InertialSensorConnection* sensor;

public:
    InertialSensor(InertialSensorConnection* sc);
    // Adds a sample of raw_data to the pool of calibration data, returns 0 for uncalibrated, returns 1 for calibrated
    bool ColectCalibrationSample();
    // Calibrates the sensor
    void CalibrateLinear();  
    // Returns thre value of the sensor after calibration but before alignment
    Vector3f GetReading();
    // Reset calibration data
    void ResetCalibration();

    Matrix3f GetT();
    Vector3f Geth();
    Matrix<float,3,N_CALIB> GetCalibData();

};

class LaserSensor
{
public:
    // Initialise lidar module
    virtual void init()=0;

    // Get lidar mesaurement
    virtual float GetMeasurement()=0;

    // Toggle laser
    virtual void ToggleLaser(bool mode)=0;
};

/**
 * @brief The sensorhandler class functions as a manager for all connected sensors. This includes a magnetometer, accelerometer, and laser distance sensor. SensorHandler requires implementations of the generic class InertialSensor to connect it to a magnetometer and an accelerometer. Additionally, an implementation of LaserSensor is required to add connection to a laser distance sensor.
 * 
 * This class is used to handle ALL sensor functions. This includes calibration, alignment, getting readings, etc.
 */
class SensorHandler
{
protected:
    Matrix3f alignment_matrix; // Alignment matrix to apply to magnetometer
    Matrix<float,3,N_ALIGNMENT> alignment_data; // Alignment data, heading, inclination, roll, distance
    float inclination_alignment; // Value to be ADDED to value of sensors to align laser and inertial sensors
    float heading_alignment; // Value to be ADDED to value of sensors to align laser and inertial sensors
    float alignment_progress;

    InertialSensor* accelerometer; // Connection to accelerometer sensor
    InertialSensor* magnetometer; // Connection to magnetometer sensor
    LaserSensor* laser; // Connection to LIDAR sensor

public:


    /**
     * @brief Construct a new SensorHandler object with a magnetometer, accelerometer, and laser sensor
     * 
     * @param acc Pointer to the connected accelerometer object
     * @param mag Pointer to the connected magnetometer object
     * @param las Pointer to the connected laser sensor object
     */
    SensorHandler(InertialSensor* acc, InertialSensor* mag, LaserSensor* las);

    /**
     * @brief Construct a new SensorHandler object with a magnetometer and accelerometer
     * 
     * @param acc Pointer to the connected accelerometer object
     * @param mag Pointer to the connected magnetometer object
     */
    SensorHandler(InertialSensor* acc, InertialSensor* mag);

    /**
     * @brief Default SensorHandler constructor
     * 
     */
    SensorHandler();

    // 
    /**
     * @brief Takes a measurement using all 3 sensors. Value returned is Vector3d(Heading, Inclination, Distace)
     * 
     * @return Vector3f Heading [deg], Inclination [deg], Distance [m]
     */
    Vector3f get_measurement();

 
    /**
     * @brief Calibrate the inertial sensors - accelerometer and magnetometer. This runs a one-off calibration using the stored calibration data in each InertialSensor bject respectively.
     */
    void CalibrateInertial();

    /**
     * @brief Runs a one-off alignment of the inertial sensors using the stored calibration data in each InertialSensor respectively.
     */
    void AlignInertial();

    /**
     * @brief Runs a one-off alignment of the laser and inertial sensors using the stored calibration data in each alignment_data respectively.
     */
    void AlignLaser();   

    /**
     * @brief Collects one shot of alignment data.
     * 
     * @return true if all alignment shots HAVE been collected
     * @return false if all alignment shots HAVE NOT been collected
     */
    bool CollectAlignmentData();   

    /**
     * @brief Collects one orientation of calibration data.
     * 
     * @return true if 12 orientations have been sampled.
     * @return false if less than 12 orientations have been sampled.
     */
    bool CollectCalibrationData(); 
    
    /**
     * @brief Resets the clalibration data and progress. MUST be run before calibration.
     */
    void ResetCalibration();

    /**
     * @brief Loads calibration data from file
     */
    void LoadCalibration();

    /**
     * @brief Resets alignment data and progress. MUST be run before alignment.
     */
    void ResetAlignment();

    /**
     * @brief Loads alignment data from file
     */
    void LoadAlignment();

    /**
     * @brief Enables the laser dot on the laser sensor
     */
    void EnableLaser();

    /**
     * @brief Disables the laser dot on the laser sensor
     */
    void DisableLaser();

    /**
     * @brief Updates the sensors and gets a reading from all sensors
     * 
     * @return Vector3f Heading [deg], Inclination [deg], Distance [m]
     */
    Vector3f GetReading();

    InertialSensor* GetAccelPtr();
    InertialSensor* GetMagPtr();
    LaserSensor* GetLaserPtr();
};

#endif