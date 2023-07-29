#include "accelerometer_csd.h"

Accelerometer::Accelerometer(InertialSensorConnection* sc) : InertialSensor(sc, &calibration_data(0,0), n_calib)
{
    static_assert(n_calib >= N_INERTIAL_ALIGNMENT,  "STATIC_ASSERT n_calib >= N_INERTIAL_ALIGNMENT");
    setID("acc");
}

Ref<MatrixXf> Accelerometer::getCalibData()
{
  return calibration_data;
}