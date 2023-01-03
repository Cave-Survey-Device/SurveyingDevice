#include "magnetometer.h"


Magnetometer::Magnetometer(struct bno055_mag *myMagData)
{
  sensor_connection = myMagData;
  init();
}

void Magnetometer::calc_magnetometer_HSI(){
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
  bno055_read_mag_xyz(sensor_connection);
  raw_mag_data << sensor_connection->x, sensor_connection->y, sensor_connection->z;
  //Serial.printf("Data x: %f y: %f z: %f\n",raw_mag_data(0),raw_mag_data(1),raw_mag_data(2));
  corrected_mag_data = raw_mag_data; //correction_transformation * raw_mag_data;
}

double Magnetometer::get_heading()
{
  // Find rotation about z-axis
  Matrix3d I;
  Vector3d x_axis;
  Matrix3d vx;
  Matrix3d rotation_mat;
  Vector3d cross_prod;
  double c;
  
  x_axis << 1,0,0;
  cross_prod = corrected_mag_data.cross(x_axis);
  c = raw_mag_data.dot(x_axis);

  vx <<              0, -cross_prod[2],  cross_prod[1],
         cross_prod[2],              0, -cross_prod[0],
        -cross_prod[1],  cross_prod[0],              0;

  I << 1,0,0,
       0,1,0,
       0,0,1;

  
  rotation_mat = I + vx + vx*vx * (1/(1+c));
  char str[30];
  sprintf(str,"Got heading: %f",RAD_TO_DEG * atan2(rotation_mat(1,0), rotation_mat(0,0)));
  debug(DEBUG_MAG,str);
	return atan2(rotation_mat(1,0), rotation_mat(0,0));

}