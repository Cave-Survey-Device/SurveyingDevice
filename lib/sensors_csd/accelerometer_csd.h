#ifndef HEADER_ACCELEROMETER
#define HEADER_ACCELEROMETER
#include <ArduinoEigenDense.h>
#include "inertialsensor.h"
#include "sensor_config_csd.h"

using namespace Eigen;


class Accelerometer: public InertialSensor
{
private:
  const static int n_calib = 12*5; 
  Matrix<float,3,n_calib> calibration_data;

public:
    Accelerometer(InertialSensorConnection* sc);
    Ref<MatrixXf> getCalibData();
};

#endif