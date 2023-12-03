#ifndef CAVESURVEYDEVICE_SENSORHANDLER_H
#define CAVESURVEYDEVICE_SENSORHANDLER_H

#include <ArduinoEigen.h>
#include <NumericalMethods.h>
#include "Sensors.h"

using namespace Eigen;

struct DeviceCalibrationParameters
{
    Matrix3f Ra, Rm , Ralign;
    Vector3f ba, bm;
    float laser_inclination, laser_heading;
};


class SensorHandler
{
private:
    const static int N_MAG_CAL = 250; // Size of magnetometer calibration matrix
    static bool MAG_COMBINED_CAL; // Calibrate magnetometer separately to alignment

    // Sensor objects
    Accelerometer acc;
    Magnetometer mag;
    Laser las;

    // Calibration and alignment data
    Matrix<float,3,N_MAG_CAL> MagCalibData;
    Matrix<float,3,N_ALIGN_MAG_ACC> MagAlignData;
    Matrix<float,3,N_ALIGN_MAG_ACC> AccAlignData;
    Matrix<float,3,N_LASER_CAL> LaserAlignData;

    // Calibration parameters
    DeviceCalibrationParameters calib_parms;

    // Data collected from sensors
    Vector3f acc_data, mag_data, laser_data;
    Vector4f shot_data; // HIRD

public:
    Vector3f getAccData();
    Vector3f getMagData();
    Vector3f getLasData();

    Vector3f getCardan();
    Vector3f getCartesian();

    // Returns Heading, Inclination, Distance
    Vector3f takeShot();

    int collectMagCalibData();
    int collectMagAccAlignData();
    int collectLaserAlignData();

    DeviceCalibrationParameters getCalibParms();
    void setCalibParms(const DeviceCalibrationParameters &parms);
};
#endif