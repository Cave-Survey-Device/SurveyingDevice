#include "magnetometer_csd.h"
#include <config.h>
#include <Arduino.h>

Magnetometer::Magnetometer(InertialSensorConnection* sc) : InertialSensor(sc, &calibration_data(0,0), n_calib)
{
    Serial.print("Magnetometer::Magnetometer(InertialSensorConnection* sc)\n");
    // ref_calibration_data = calibration_data;
    //new (&ref_calibration_data) Map<Matrix<float,3,-1>>(&calibration_data(0,0),3,n_calib);
    //ref_calibration_data = Map<Matrix<float,3,n_calib>>(&calibration_data(0,0),3,n_calib);
    //ref_calibration_data = calibrated_data;
    static_assert(n_calib >= N_INERTIAL_ALIGNMENT,  "ASSERT n_calib >= N_INERTIAL_ALIGNMENT");
}

void Magnetometer::addCalibrationData(){
    int index;
    Vector3f sample = getSingleSample();
    index = this->getMagnetometerIndex(sample);
    calibration_data.col(index) << sample;
}

int Magnetometer::getMagnetometerIndex(const Vector3f &sample){
    // -pi to pi
    float azimuth = 0;
    //-pi/2 to pi/2
    float elevation = 0;
    // 0 - 17
    int index1;
    // 0 - 35
    int index2;

    // https://uk.mathworks.com/help/matlab/ref/cart2sph.html
    azimuth = atan2(sample(1),sample(0)) + PI;
    elevation = atan2(sample(2),sqrt(pow(sample(0),2) + pow(sample(1),2))) + PI/2;

    index1 = (int)floor(0.1 * RAD_TO_DEG * elevation - 1);
    index2 = (int)floor(0.1 * RAD_TO_DEG * azimuth - 1);

    if (index1 < 0)
    {
    index1 = 0;
    }
    if (index2 < 0)
    {
    index2 = 0;
    } 
    return index1 * 36 + index2;
}

int Magnetometer::checkCalibrationProgress(){
  int progress = 0;
  int i;
  for (i=0;i<n_calib;++i){
    if ((calibration_data(0,i) != 0) && (calibration_data(1,i) != 0) && (calibration_data(2,i) != 0))
    {
      ++progress;
    }
  }
  return int(progress*100.0/643.0);
}

MatrixXf Magnetometer::getCalibData()
{
  return calibration_data;
}
