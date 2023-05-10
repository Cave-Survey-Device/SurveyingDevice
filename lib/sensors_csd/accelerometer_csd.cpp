#include "accelerometer_csd.h"

Accelerometer::Accelerometer(InertialSensorConnection* sc) : InertialSensor(sc, &calibration_data(0,0), n_calib)
{
    // ref_calibration_data = calibration_data;
    // new (&ref_calibration_data) Map<MatrixXf>(calibration_data.data(),calibration_data.rows(),calibration_data.cols());
    //ref_calibration_data = calibrated_data;
    //new (&ref_calibration_data) Map<Matrix<float,3,-1>>(&calibration_data(0,0),3,n_calib);
    static_assert(n_calib >= N_INERTIAL_ALIGNMENT,  "STATIC_ASSERT n_calib >= N_INERTIAL_ALIGNMENT");
}

MatrixXf Accelerometer::getCalibData()
{
  return calibration_data;
}