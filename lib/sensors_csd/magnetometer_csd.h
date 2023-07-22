#ifndef HEADER_MAGNETOMETER
#define HEADER_MAGNETOMETER

#include "inertialsensor.h"
#include "sensor_config_csd.h"
#include <ArduinoEigenDense.h>
using namespace Eigen;

/**
 * @brief 
 * 
 */
class Magnetometer: public InertialSensor
{
private:
  const static int n_calib = 12*5; /** Number of columns in the calibration data matrix*/
  Matrix<float,3,n_calib> calibration_data; /** Calibration data matrix*/
  
public:
  Magnetometer(InertialSensorConnection* sc); /** Constructor for the Magnetometer object*/
  int checkCalibrationProgress(); /** Checks the calibration progress of the magnetometer when using the standalone calibration*/
  int getMagnetometerIndex(const Vector3f &sample); /** Gets the index of a sample in the magnetometer calibration array */
  void addCalibrationData(); /** Collects a sample and and adds it to the magnetometer calibration Matrix when using standalone calibration*/

  Ref<MatrixXf> getCalibData(); /** Gets the matrix of calibration data*/

};

#endif