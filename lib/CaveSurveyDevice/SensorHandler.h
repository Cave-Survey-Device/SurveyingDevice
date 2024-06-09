#ifndef CAVESURVEYDEVICE_SENSORHANDLER_H
#define CAVESURVEYDEVICE_SENSORHANDLER_H

#include <ArduinoEigen.h>
#include <NumericalMethods.h>
#include "Sensors.h"
#include <EigenFileFuncs.h>


using namespace Eigen;

// Define constants for general use

const int N_MAG_CAL_HEADING = 25; // Size of magnetometer calibration matrix
const int N_MAG_CAL_INCLINATION = 15; // Size of magnetometer calibration matrix
const int N_MAG_CAL = N_MAG_CAL_HEADING * N_MAG_CAL_INCLINATION;
const int N_SHOT_SMAPLES = 100;
const int N_UPDATE_SAMPLES = 25;
const float STDEV_LIMIT = 0.05;
const int N_STABILISATION = 10;

struct DeviceCalibrationParameters
{
    Matrix3f Ra_cal, Rm_cal, Ra_las, Rm_las, Rm_align;
    Vector3f ba_cal, bm_cal;
    float inclination_angle;
};

struct LaserCalibrationData{
    Matrix<float,3,N_LASER_CAL> mag_data;
    Matrix<float,3,N_LASER_CAL> acc_data;
};

struct StaticCalibrationData{
    Matrix<float,3,N_ALIGN_MAG_ACC> mag_data;
    Matrix<float,3,N_ALIGN_MAG_ACC> acc_data;
};

struct ShotData{
    Vector3f m, g, HIR, v;
    float d;
    int ID;
};

void saveShotData(const ShotData sd);
void readShotData(ShotData &sd);

class SensorHandler
{
private:
    bool MAG_COMBINED_CAL = true; // Calibrate magnetometer separately to alignment

    int static_calib_progress, las_calib_progress;

    // Sensor objects
    Accelerometer &acc;
    Magnetometer &mag;
    Laser &las;

    // Calibration and alignment data
    LaserCalibrationData laser_calib_data;
    StaticCalibrationData static_calib_data;

    // Calibration parameters
    DeviceCalibrationParameters calib_parms;

    // Data collected from sensors
    ShotData shot_data, corrected_shot_data;
    Vector3f acc_data, mag_data, corrected_acc_data, corrected_mag_data;
    float las_data;

public:
    SensorHandler(Accelerometer &a, Magnetometer &m, Laser &l);

    void init();

    Vector3f getAccData();
    Vector3f getMagData();
    float getLasData();

    const StaticCalibrationData &getStaticCalibData();
    const LaserCalibrationData &getLaserCalibData();
    const DeviceCalibrationParameters &getCalibParms();

    void update();
    Vector3f getCardan(bool corrected = true);
    Vector3f getCartesian(bool corrected = true);
    Vector3f getFinalMeasurement(bool corrected = true);

    void eraseFlash();
    void getFlashStats();

    void correctData(Vector3f &m, Vector3f &g);
    
    void resetCalibration();
    void saveCalibration();
    void loadCalibration();

    /**
     * @brief Take shot using laser by default.
     * Returns 0 if success, anything else is an error.
     * 
     * @param laser_reading 
     * @return int 
     */
    int takeShot(const bool laser_reading = true, const bool use_stabilisation = true);

    /**
     * @brief Collects a sample of alignment data for joint accelerometer and magnetometer alignment.
     * Returns the current progress out of N_MAG_ACC_ALIGN, -1 if complete.
     * 
     * @return int 
     */
    int collectStaticCalibData();

    /**
     * @brief Collects a sample of alignment data for laser alignment.
     * Returns the current progress out of N_LASER_CAL, -1 if complete.
     * 
     * @return int 
     */
    int collectLaserCalibData();

    void removePrevCalib(bool static_calib);
    int getCalibProgress(bool static_calib);

    int calibrate();
    int align();
    int staticAlign();

    Vector2f getDirection();
    ShotData getShotData(bool corrected = true);



    void setCalibParms(const DeviceCalibrationParameters &parms);
};
#endif