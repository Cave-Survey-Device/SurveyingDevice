#ifndef HEADER_ACCELEROMETER
#define HEADER_ACCELEROMETER
#include <ArduinoEigenDense.h>
#include "inertialsensor.h"
#include "sensor_config_csd.h"

using namespace Eigen;

/**
 * @brief The Accelerometer class.
 * This calss inherits from InertialSensor and provides the additioanly functionality available to accelerometer sensors.
 */
class Accelerometer: public InertialSensor
{
private:
  const static int n_calib = 12*5; /** Number of columns in the calibration data matrix*/
  Matrix<float,3,n_calib> calibration_data; /** Calibration data matrix*/

public:
  /**
   * @brief Construct a new Accelerometer object
   * 
   * @param sc 
   */
    Accelerometer(InertialSensorConnection* sc);

    /**
     * @brief Returns a reference to the calibration data matrix for this
     * 
     * @return Ref<MatrixXf> 
     */
    Ref<MatrixXf> getCalibData();
};

#endif