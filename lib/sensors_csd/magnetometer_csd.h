#ifndef HEADER_MAGNETOMETER
#define HEADER_MAGNETOMETER

#include "inertialsensor.h"
#include "sensor_config_csd.h"
#include <ArduinoEigenDense.h>
using namespace Eigen;

/**
 * @brief The Magnetometer class.
 * This calss inherits from InertialSensor and provides the additioanly functionality available to magnetometer sensors.
 */
class Magnetometer: public InertialSensor
{
private:
  const static int n_calib = 12*5; /** Number of columns in the calibration data matrix*/
  Matrix<float,3,n_calib> calibration_data; /** Calibration data matrix*/
  
public:
  /**
   * @brief Constructor for the Magnetometer object
   * 
   * @param sc 
   */
  Magnetometer(InertialSensorConnection* sc); 

  /**
   * @brief Checks the calibration progress of the magnetometer when using the standalone calibration
   * 
   * @return int 
   */
  int checkCalibrationProgress(); 
  
  /**
   * @brief Gets the index of a sample in the magnetometer calibration array
   * 
   * @param sample 
   * @return int 
   */
  int getMagnetometerIndex(const Vector3f &sample); 
  
  /**
   * @brief Collects a sample and and adds it to the magnetometer calibration Matrix when using standalone calibration
   * 
   */
  void addCloudCalidData(); 

  /**
   * @brief Gets the matrix of calibration data
   * 
   * @return Ref<MatrixXf> 
   */
  Ref<MatrixXf> getCalibData();

};

#endif