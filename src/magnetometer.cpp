#include "magnetometer.h"


Magnetometer::Magnetometer(struct bno055_mag *myMagData)
{
  sensor_connection = myMagData;
  init();
}

void Magnetometer::calc_magnetometer_HSI(){
  /**************************************************************************************************
   * 1. Subtract mean of point cloud from data to remove hard iron effects and center the data
   * 2. Find the covariance of the data to find the transformation from a perfect sphere
   * 3. Get the eigen-decomposition of the covariance
   * 4. Find the squareroot of the eigenvalues
   * 5. Multiply the eigenvectors by the diagonal matrix formed by the squareroot of the eigenvalues
   * 6. Find the inverse of this and return it
  **************************************************************************************************/
  MatrixXd centered = magnetometer_arr.colwise() - magnetometer_arr.rowwise().mean();
  Matrix3d cov = (centered.transpose() * centered) / double(magnetometer_arr.cols() - 1);
  EigenSolver<Matrix3d> eig;
  eig.compute(cov);
  cout << eig.eigenvalues() << "\n";
  Vector3cd vec = eig.eigenvalues();
  Vector3cd eigenvalues_sqrt = vec.array().pow(0.5);
  DiagonalMatrix<std::complex<double>, 3> diag_sqrt_eig(eigenvalues_sqrt);
  Matrix3cd T = eig.eigenvectors() * diag_sqrt_eig;
  Matrix3cd inv_T = T.inverse();
  correction_transformation = inv_T.real();
}

void Magnetometer::init(){  
  /**********************************************************
   * Initialises the calibration array for the magnetometer
  ***********************************************************/
  int i;
  for (i=0;i<MAGNETOMETER_ARR_LEN;++i){
    magnetometer_arr(0,i) = 0;
    magnetometer_arr(1,i) = 0;
    magnetometer_arr(2,i) = 0;
  }
}

int Magnetometer::get_magnetometer_index(double x, double y,double z){
  // -pi to pi
  float azimuth = 0;
  //-pi/2 to pi/2
  float elevation = 0;
  // 0 - 17
  int index1;
  // 0 - 35
  int index2;

  // https://uk.mathworks.com/help/matlab/ref/cart2sph.html
  azimuth = atan2(y,x) + PI;
  elevation = atan2(z,sqrt(pow(x,2) + pow(y,2))) + PI/2;

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

  if (DEBUG){
    Serial.print("Azimuth: ");
    Serial.println(RAD_TO_DEG*azimuth);
    Serial.print("Elevation: ");  
    Serial.println(RAD_TO_DEG*elevation);
  }

  return index1 * 36 + index2;
}

void Magnetometer::add_calibration_data(){
  /**************************************************************************************
   * Adds calibration data from raw_mag_data updated by update() to the calibration array
   * Data is added to spaces corresponding to 1deg sectors of the spheres surface and is
   * overwritten when new data at the same location is provided.
  ***************************************************************************************/
  int index;
  double x,y,z;
  x = raw_mag_data(0);
  y = raw_mag_data(1);
  z = raw_mag_data(2);
  index = get_magnetometer_index(x,y,z);

  if (DEBUG){
    Serial.print("Index:" );
    Serial.println(index);
    Serial.print("RAW_DATA:" );
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(z);
  }

  magnetometer_arr(0,index) = x;
  magnetometer_arr(1,index) = y;
  magnetometer_arr(2,index) = z;
}

int Magnetometer::check_calibration_progress(){
  /***************************************************************************************
   * Checks the calibration progress of the magnetometer by counting how many locations
   * are nonzero and the dividing by the total number of possible positions
  ****************************************************************************************/
  int progress = 0;
  int i;
  for (i=0;i<MAGNETOMETER_ARR_LEN;++i){
    if ((magnetometer_arr(0,i) != 0) && (magnetometer_arr(1,i) != 0) && (magnetometer_arr(2,i) != 0))
    {
      ++progress;
    }
  }
  return int(progress*100.0/643.0);
}

void Magnetometer::update()
{
  /****************************************************************
   * Updates the magnetometers stored values from the sensor
   * Stores both raw and corrected data
  *****************************************************************/
  bno055_read_mag_xyz(sensor_connection);
  raw_mag_data << sensor_connection->x, sensor_connection->y, sensor_connection->z;
  corrected_mag_data = correction_transformation * raw_mag_data;
}

Vector3d Magnetometer::get_mag_vec()
{
    return corrected_mag_data;
};
