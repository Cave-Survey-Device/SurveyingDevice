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
    Matrix3f calibration_matrix;
    Vector3f calibration_offset;
    Vector3f calibrated_data;
    Vector3f raw_data;

    InertialSensorConnection* sensor;

    bool calibrate_with_alignment;
    int align_num;

    Eigen::Map<Matrix<float,3,-1>> ref_calibration_data;
    // Ref<MatrixXf> ref_calibration_data;
    //MatrixXf ref_calibration_data;
    bool separate_calib;
    char device_ID[6]; // Device's ID for saving calibration data

public:

    bool getCalibMode();
    void setCalibMode(bool mode);

    InertialSensor(InertialSensorConnection* sc, float* ptr, int size);

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
     * @brief 
     * 
     * @return true 
     * @return false 
     */
    bool collectAlignmentSample();

    Matrix3f getT();
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