#ifndef HEADER_ACCELEROMETER
#define HEADER_ACCELEROMETER
#include <ArduinoEigenDense.h>
#include "inertialsensor.h"
#include "sensor_config_csd.h"

using namespace Eigen;


class Accelerometer: public InertialSensor
{
private:
  const static int n_calib = 36*18; 
  Matrix<float,3,n_calib> calibration_data;

public:
    Accelerometer(InertialSensorConnection* sc);
    MatrixXf getCalibData();
};

#endif