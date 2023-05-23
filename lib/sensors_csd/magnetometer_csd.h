#ifndef HEADER_MAGNETOMETER
#define HEADER_MAGNETOMETER

#include "inertialsensor.h"
#include "sensor_config_csd.h"
#include <ArduinoEigenDense.h>
using namespace Eigen;

class Magnetometer: public InertialSensor
{
private:
  const static int n_calib = 12*5; 
  Matrix<float,3,n_calib> calibration_data;
  //float calibration_data[3][n_calib];
  
public:
  Magnetometer(InertialSensorConnection* sc);
  int checkCalibrationProgress();
  int getMagnetometerIndex(const Vector3f &sample);
  void addCalibrationData();

  Ref<MatrixXf> getCalibData();

};

#endif