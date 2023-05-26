#include <filesystem_csd.h>
#include "inertialsensor.h"
#include "NumericalMethods_csd.h"

bool InertialSensor::collectAlignmentSample()
{
    if (this->align_num < N_INERTIAL_ALIGNMENT)
    {
        Vector3f data = this->getReading();
        Serial << "Calib num: " << this->align_num << "\t" << data(0) << " " << data(1) << " " << data(2) << "\n";
        ref_calibration_data.col(this->align_num) << data;
        // Serial << "ref_calibration_data: " << ref_calibration_data.col(this->align_num)(0) << "\t" << ref_calibration_data.col(this->align_num)(1) << " " << ref_calibration_data.col(this->align_num)(2) << "\n";
        // Serial << "calibration_data: " << getCalibData().col(this->align_num)(0) << "\t" << getCalibData().col(this->align_num)(1) << " " << getCalibData().col(this->align_num)(2) << "\n";
        this->align_num++;
        
        return 0;
    } 
    return 1;
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
    save_tmp_calibration_data();
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
    
    return this->calibration_matrix * (reading - this->calibration_offset);
}

void InertialSensor::resetCalibration()
{
    align_num = 0;
    ref_calibration_data.setZero();
    calibration_matrix = Matrix3f::Identity();
    calibration_offset = Vector3f::Identity();
    calibrate_with_alignment = true;
}

InertialSensor::InertialSensor(InertialSensorConnection* sc, float* ptr, int size) : ref_calibration_data(ptr,3,size)
{
    this->sensor = sc;
    this->resetCalibration();
    separate_calib = false;
}

Matrix3f InertialSensor::getT()
{
    return this->calibration_matrix;
}

Vector3f InertialSensor::geth()
{
    return this->calibration_offset;
}

bool InertialSensor::getCalibMode()
{
    return separate_calib;
}

void InertialSensor::setCalibMode(bool mode)
{
    separate_calib = mode;
}

void InertialSensor::load_calibration_data()
{
    debug(DEBUG_INERTIALSENSOR,"InertialSensor::load_calibration_data()");
    //read_from_file("calib_data",device_ID,&ref_calibration_data(0),ref_calibration_data.size());
    read_from_file(device_ID,"calib_data",getCalibData());
    read_from_file(device_ID,"calib_mat",calibration_matrix);
    read_from_file(device_ID,"calib_off",calibration_offset);
}
void InertialSensor::save_calibration_data()
{
    debug(DEBUG_INERTIALSENSOR,"InertialSensor::save_calibration_data()");
    //write_to_file("calib_data",device_ID,&ref_calibration_data(0),ref_calibration_data.size());
    write_to_file(device_ID,"calib_data",getCalibData());
    write_to_file(device_ID,"calib_mat",calibration_matrix);
    write_to_file(device_ID,"calib_off",calibration_offset);
}

void InertialSensor::load_tmp_calibration_data()
{
    debug(DEBUG_INERTIALSENSOR,"InertialSensor::load_calibration_data()");
    //read_from_file("calib_data",device_ID,&ref_calibration_data(0),ref_calibration_data.size());
    read_from_file(device_ID,"tmp_calib_data",getCalibData());
    read_from_file(device_ID,"tmp_calib_mat",calibration_matrix);
    read_from_file(device_ID,"tmp_calib_off",calibration_offset);
}
void InertialSensor::save_tmp_calibration_data()
{
    debug(DEBUG_INERTIALSENSOR,"InertialSensor::save_tmp_calibration_data()");
    write_to_file(device_ID,"tmp_calib_data",getCalibData());
    write_to_file(device_ID,"tmp_calib_mat",calibration_matrix);
    write_to_file(device_ID,"tmp_calib_off",calibration_offset);
}

void InertialSensor::setID(const char* ID)
{
    // debugf(DEBUG_INERTIALSENSOR,"InertialSensor::setID(const char* ID), ID = %c", ID);
    debug(DEBUG_INERTIALSENSOR, "Setting device_ID");
    strncpy(device_ID,ID,sizeof(device_ID)-1);
}