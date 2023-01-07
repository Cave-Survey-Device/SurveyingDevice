#include "lasercalibration.h"

Vector3d calc_normal_vec(MatrixXd g_vec, bool debug /*= false*/){
  char buffer[150];
  Vector3d normal;
  MatrixXd left_singular_mat;
  int U_cols;

  JacobiSVD<MatrixXd> svd(g_vec, ComputeThinU | ComputeThinV);
  left_singular_mat = svd.matrixU();
  U_cols = left_singular_mat.cols();
  normal << left_singular_mat(0,U_cols-1), left_singular_mat(1,U_cols-1), left_singular_mat(2,U_cols-1);

  if (debug){
    sprintf(buffer, "U: ", svd.matrixU(), "Sigma: ", svd.singularValues(), "\n");
    Serial.println(buffer);
    sprintf(buffer, "Normal Vector: \nX: %f \nY: %f \nZ: %f\n", normal[0], normal[1], normal[2]);
    Serial.printf(buffer);
  }

  return normal;
};

Matrix<double,3,1> calc_true_vec(Vector3d normal_vec, VectorXd laser_distances, bool DEBUG /*= false*/){
  Matrix<double,3,1> x_axis;
  Vector3d true_vec;
  char buffer[150];
  double laser_len;
  double alpha;
  double l_0, l_1, l_2, l_3;

  laser_len = laser_distances.mean();
  x_axis << 1.0, 0.0, 0.0;

  alpha = acos(normal_vec.dot(x_axis) / (normal_vec.norm()*x_axis.norm()));
  if (alpha > M_PI_2){
    alpha = M_PI-alpha;
    normal_vec = -normal_vec;
  };
  
  l_0 = DISTO_LEN;
  l_1 = laser_len;
  l_2 = l_0 * sin(alpha);
  l_3 = l_0 * cos(alpha) + sqrt(pow(l_1,2)-pow(l_2,2));

  true_vec = l_3 * x_axis + l_0 * normal_vec;
  true_vec = true_vec / true_vec.norm();

  if (DEBUG_LASER_CAL){
    sprintf(buffer, "Alpha: %f \nNormal vector: %f %f %f\n", RAD_TO_DEG * alpha, normal_vec[0], normal_vec[1], normal_vec[2]);
    Serial.printf(buffer);
    sprintf(buffer, "L0: %f \nL1: %f \nL2: %f \nL3: %f\n",l_0,l_1,l_2,l_3);
    Serial.printf(buffer);
    sprintf(buffer, "True vec: %f %f %f\n",true_vec[0],true_vec[1],true_vec[2]);
    Serial.printf(buffer);
  }
  return true_vec;
}