#ifndef HEADER_NUMERICAL_METHODS
#define HEADER_NUMERICAL_METHODS

#include <ArduinoEigenDense.h>
#include <sensors/Sensors.h>
#include <utils/utility.h>
#include <queue>

using namespace Eigen;

// // Given a point cloud, calculate the best fit ellipsoid, returning the quadratic ellipsoid parameters
// RowVector<float,10> fit_ellipsoid(MatrixXf samples)
// {
//     int N = samples.cols();
//     // Design matrix
//     VectorXf x = samples.row(0);
//     VectorXf y = samples.row(1);
//     VectorXf z = samples.row(2);

//     // Create design matrix
//     MatrixXf D = MatrixXf::Zero(N,10);

//     D << x.array().pow(2), y.array().pow(2), z.array().pow(2), 2*y.array()*z.array(), 2*x.array()*z.array(), 2*x.array()*y.array(), 2*x.array(), 2*y.array(), 2*z.array(), VectorXf::Ones(N);
//     D.transposeInPlace();

//     // D seems correct
//     //std::cout << "D: \n" << D << "\n\n";


//     // Apply constraint kJ > I^2
//     // The quadratic surface is an ellipse if k = 4
//     int k = 4;

//     // Create constrain matrix C - Eq(7)
//     Matrix<float,6,6> C;
//     C <<   -1, 0.5*k-1, 0.5*k-1, 0, 0, 0,
//             0.5*k-1, -1, 0.5*k-1, 0, 0, 0,
//             0.5*k-1, 0.5*k-1, -1, 0, 0, 0,
//             0, 0, 0, -k, 0, 0,
//             0, 0, 0, 0, -k, 0,
//             0, 0, 0, 0, 0, -k;

//     // Create S matrix from D.T*D - Eqn(11)
//     MatrixXf S = D * D.transpose();
//     Matrix<float,6,6> S11 = S.block<6,6>(0,0);
//     Matrix<float,6,4> S12 = S.block<6,4>(0,6);
//     Matrix<float,4,6> S21 = S.block<4,6>(6,0);
//     Matrix<float,4,4> S22 = S.block<4,4>(6,6);

// //    std::cout << "\n\n";
// //    std::cout << "S11: \n" << S11 << "\n\n";
// //    std::cout << "S12: \n" << S12 << "\n\n";
// //    std::cout << "S21: \n" << S21 << "\n\n";
// //    std::cout << "S22: \n" << S22 << "\n\n";


//     // Solve least squares - Eqn(14) and Eqn(15)
//     MatrixXf M  = C.inverse() * (S11 - S12*S22.inverse() * S21);
//     EigenSolver<MatrixXf> es(M);
//     Vector<std::complex<float>,6> eigenvalues = es.eigenvalues();
//     Matrix<std::complex<float>,6,6> eigenvectors = es.eigenvectors();

//     Vector<float,6> eval = eigenvalues.array().real();
//     Matrix<float,6,6> evec = eigenvectors.array().real();
// //    std::cout << "Matrix to decompose: \n" << M << "\n";
// //    std::cout << "Eigenvalues: \n" << eval << "\n";
// //    std::cout << "Eigenvectors: \n" << evec << "\n\n";

//     // Find eigenvector corresponding to largest eigenvalue
//     Vector<float,6> u1;
//     for (int i=0;i<6;i++)
//     {
//         if(eval[i] > 0.0) {
//             u1 = evec.col(i);
//             break;
//         } else if (i == 5) {
//             // No eigenvalues found
//         }
//     }

//     Vector<float,4> u2 = (-(S22.inverse() * S21) * u1);
//     Vector<float,10> U;
//     U << u1, u2;
//     return U;
// }

void displayMat(const MatrixXf &m);
void displayVec(const VectorXf &v);
float Deg2Rad(float degrees);
Matrix3f x_rotation(float deg);
Matrix3f y_rotation(float deg);
Matrix3f z_rotation(float deg);

// Given a point cloud, calculate the best fit ellipsoid, returning the quadratic ellipsoid parameters
RowVector<float,10> fit_ellipsoid(const Matrix<float,3,N_CALIB> &samples);

Vector<float,12> calculate_ellipsoid_transformation(Matrix3f &M, Vector3f &n, float d);

// Given a point cloud, find a plane of best fit and return the vector normal to this plane
Vector3f NormalVec(MatrixXf point_cloud);

// Calculates a 3x3 matrix representing a rotation of "rads" about the x axis
Matrix3f XRotation(float rads);

float StdDev(std::queue<Vector3f> q);

#endif