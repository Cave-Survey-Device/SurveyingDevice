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
    Eigen::Matrix3f calibration_matrix; /** R, matrix used to calibrate the sensor to the body frame of the device*/
    Eigen::Vector3f calibration_offset; /** h, the offset used to remove bias error*/
    Eigen::Vector3f calibrated_data; /** Calibrated version of the raw_data*/
    Eigen::Vector3f raw_data; /** Raw data from the sensor connection*/

    InertialSensorConnection* sensor; /** Connected sensor*/

    Eigen::Map<Matrix<float,3,-1>> ref_calibration_data; /** Reference object to the calibration data of the sensor. This uses the child's instantiation of calibration data to allow for different sizings for different sensors*/
    char device_ID[6]; /** Device's ID for saving calibration data */

    uint16_t calib_num; /** Current number of calibration*/
    uint16_t calib_max; /** Size of the calibraion matrix*/

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
     * @brief Linear calibration of an inertial sensor via ellipsoid fitting.
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

    void setCalibMat(Matrix3f mat);
    void setCalibBias(Vector3f vec);

    /**
     * @brief Gets the transformation matrix for the sensor calibration
     * 
     * @return Matrix3f 
     */
    Matrix3f getCalibMat();

    /**
     * @brief Gets the bias vector for the sensor calibration
     * 
     * @return Matrix3f 
     */
    Vector3f getCalibBias();


    /**
     * @brief Get the Calib Data object - This is a reference to the actual data
     * 
     * @return Ref<MatrixXf> 
     */
    virtual Ref<MatrixXf> getCalibData()=0;

    /**
     * @brief Sets the calibration data 
     * 
     */
    void setCalibrationData();

    /**
     * @brief Adds a single value into the calibration data of the inertial sensor
     * 
     */
    bool addCalibData();

    uint16_t getCalibProgress();

    /**
     * @brief Sets the device's ID such that calibration data for teh device can be read
     * 
     * @param ID 
     */
    void setID(const char* ID);

    const char* getID();

};

#endif