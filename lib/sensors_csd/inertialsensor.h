#ifndef HEADER_INERTIALSENSOR
#define HEADER_INERTIALSENSOR
#include <ArduinoEigenDense.h>
#include "sensor_config_csd.h"

using namespace Eigen;

class InertialSensorConnection
{
public:
    virtual Vector3f getRawData();
};

class InertialSensor
{
protected:
    Matrix3f calibration_matrix; /** R, matrix used to calibrate the sensor to the body frame of the device*/
    Vector3f calibration_offset; /** h, the offset used to remove bias error*/
    Vector3f calibrated_data; /** Calibrated version of the raw_data*/
    Vector3f raw_data; /** Raw data from the sensor connection*/

    InertialSensorConnection* sensor; /** Connected sensor*/

    bool calibrate_with_alignment; /** Calibrate this sensor at the same time as the alignment?*/
    int align_num; /** Stage of alignment process*/

    Eigen::Map<Matrix<float,3,-1>> ref_calibration_data; /** Reference object to the calibration data of the sensor. This uses the child's instantiation of calibration data to allow for different sizings for different sensors*/
    bool separate_calib; /** Does this sensor need calibration seperately?*/
    char device_ID[6]; /** Device's ID for saving calibration data */

public:

    /**
     * @brief Construct a new Inertial Sensor object
     * 
     * @param sc The sensor connection to the inertial sensor
     * @param ptr Pointer to the calibration data matrix
     * @param size Size of the calibration data matrix in readings
     * 
     * @todo Change ptr and size to Eigen::Ref<float>
     */
    InertialSensor(InertialSensorConnection* sc, float* ptr, int size);

    /**
     * @brief Get the calibration mode of the sensor
     * 
     * @return true if separate calibration
     * @return false if joint calibrtion
     */
    bool getCalibMode();

    /**
     * @brief Sets the calibration mode of the sensor
     * 
     * @param mode 
     */
    void setCalibMode(bool mode);


    /**
     * @brief Linear calibrataes an inertial sensor via ellipsoid fitting.
     * 1. Sorts all zero-valued columns of the calibration data to the end of the matrix
     * 2. Performs ellipsoid fit with first n elements of sorted matrix where n is the numer of non-zero values
     * 
     */
    void calibrateLinear();  

    /**
     * @brief Gets a single sample from the connected inertial sensor. No calibration or averaging, "raw data" from the sensor.
     * 
     * @return Vector3f 
     */
    Vector3f getSingleSample();

    /**
     * @brief Collectes a reading from the connected inertial sensor. This means that calibration and averaging is applied
     * 
     * @return Vector3f 
     */
    virtual Vector3f getReading();

    /**
     * @brief Resets the calibration data in memory
     * 
     */
    void resetCalibration();

    /**
     * @brief Collects a sample for inertial sensor axis alignment
     * 
     * @return true 
     * @return false 
     */
    bool collectAlignmentSample();

    /**
     * @brief Gets the transformation matrix for the sensor calibration
     * 
     * @return Matrix3f 
     */
    Matrix3f getT();

    /**
     * @brief Gets the bias vector for the sensor calibration
     * 
     * @return Matrix3f 
     */
    Vector3f geth();

    virtual Ref<MatrixXf> getCalibData()=0;

    /**
     * @brief Loads calibration data and paremeters from non-volatile memory
     * 
     */
    void load_calibration_data();

    /**
     * @brief Saves the calibration data and correction paremeters to non-volatile memory
     * 
     */
    void save_calibration_data();

    /**
     * @brief Loads non-default (temporary) calibration data and paremeters from non-volatile memory
     * 
     */
    void load_tmp_calibration_data();

    /**
     * @brief Saves calibration data and parameters to non-volatile storage in a seperate location
     * This is to allow form calibration data to be generated and discarded if necessary.
     * 
     */
    void save_tmp_calibration_data();
    

    /**
     * @brief Sets the device's ID such that calibration data for teh device can be read
     * 
     * @param ID 
     */
    void setID(const char* ID);
};

#endif