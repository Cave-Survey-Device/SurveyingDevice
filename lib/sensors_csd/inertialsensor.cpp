#include <filesystem_csd.h>
#include "inertialsensor.h"
#include "NumericalMethods_csd.h"

bool InertialSensor::addCalibData()
{
    if (this->calib_num < calib_max)
    {
        Vector3f data = getReading();
        ref_calibration_data.col(calib_num) << data;
        calib_num++;
        return 0;
    } else {
        return 1;
    }

}

void InertialSensor::calibrateLinear()
{    
    RowVector<float,10> U;
    Matrix3f M;
    Vector3f n;
    float d;

    Serial << "Begin ellipsoid fitting...\n";

    // Re-arrange data and remove values equal to 0,0,0;
    // Passing non-const references appears to be a bit broken in Eigen so a workaround is to use maps
    // float* data_ptr = &getCalibData()(0,0);
    // int data_size = getCalibData().cols();
    int n_zeros = removeNullData(getCalibData());

    Serial.print("Sorted data:\n");
    displayMat(getCalibData().block(0,0,3,getCalibData().cols()-n_zeros));

    // Calculate ellipsoid parameters
    U = fit_ellipsoid(getCalibData(), getCalibData().cols()-n_zeros);
    
    M << U[0], U[5], U[4], U[5], U[1], U[3], U[4], U[3], U[2];
    n << U[6], U[7], U[8];
    d = U[9];

    Serial << "Begin ellipsoid transformation calculations\n";
    Vector<float, 12> transformation = calculate_ellipsoid_transformation(M, n, d);

    this->calibration_matrix << transformation[0], transformation[1], transformation[2], transformation[3], transformation[4], transformation[5], transformation[6], transformation[7], transformation[8];
    this->calibration_offset << transformation[9], transformation[10], transformation[11];
    
    // Save calibration data in tmp file
    // save_tmp_calibration_data();
}

Vector3f InertialSensor::getSingleSample()
{
    Vector3f data;
    data = this->sensor->getRawData();
    return data;
}

Vector3f InertialSensor::getReading()
{
    debug(DEBUG_INERTIALSENSOR, "InertialSensor::getReading()");
    Vector3f reading;
    reading.setZero();
    for (int i=0;i<SAMPLES_PER_READING;i++)
    {
        reading += this->sensor->getRawData();
    }
    reading = reading/SAMPLES_PER_READING;
    
    // Serial << "Reading:";
    // displayRowVec(reading);
    // Serial << "Corrected reading:";
    // displayRowVec(this->calibration_matrix * (reading - this->calibration_offset));
    return this->calibration_matrix * (reading - this->calibration_offset);
}

void InertialSensor::resetCalibration()
{
    calib_num = 0;
    calib_max = ref_calibration_data.cols();
    ref_calibration_data.setZero();
    calibration_matrix = Matrix3f::Identity();
    calibration_offset = Vector3f::Zero();
}

InertialSensor::InertialSensor(InertialSensorConnection* sc, float* ptr, int size) : ref_calibration_data(ptr,3,size)
{
    this->sensor = sc;
    this->resetCalibration();
}

Matrix3f InertialSensor::getCalibMat()
{
    return this->calibration_matrix;
}
void InertialSensor::setCalibMat(Matrix3f calib_mat)
{
    this->calibration_matrix = calib_mat;
}
Vector3f InertialSensor::getCalibBias()
{
    return this->calibration_offset;
}
void InertialSensor::setCalibBias(Vector3f calib_bias)
{
    this->calibration_offset = calib_bias;
}

void InertialSensor::setID(const char* ID)
{
    // debugf(DEBUG_INERTIALSENSOR,"InertialSensor::setID(const char* ID), ID = %c", ID);
    debug(DEBUG_INERTIALSENSOR, "Setting device_ID");
    strncpy(device_ID,ID,sizeof(device_ID)-1);
}
const char* InertialSensor::getID()
{
    return device_ID;
}