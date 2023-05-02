#include <utils/NumericalMethods.h>
#include <utils/utility.h>

void displayMat(const MatrixXf &m)
{
    int rows = m.rows();
    int cols = m.cols();
    for(int i=0; i<rows;i++)
    {
        Serial << "[\t";
        for(int j=0; j<cols;j++)
        {
            Serial << m(i,j) << "\t";
        }
        Serial << "\t]\n";
    }
    Serial << "    \n";
}

void displayVec(const VectorXf &v)
{
    int n = v.size();
    for(int i=0; i<n;i++)
    {
        Serial << "[\t" << v(i) << "\t]\n";
    }
    Serial << "\n";
}


float Deg2Rad(float degrees) {
    return degrees * (3.14159265359 / 180.0);
}

Matrix3f x_rotation(float deg)
{
    deg = Deg2Rad(deg);
    Matrix3f R;
    R << cos(deg), 0., sin(deg),
            0., 1., 0.,
            -sin(deg), 0., cos(deg);
    return R;
}

Matrix3f y_rotation(float deg)
{
    deg = Deg2Rad(deg);
    Matrix3f R;
    R << 1., 0., 0.,
            0., cos(deg), -sin(deg),
            0., sin(deg), cos(deg);
    return R;
}

Matrix3f z_rotation(float deg)
{
    deg = Deg2Rad(deg);
    Matrix3f R;
    R << cos(deg), -sin(deg), 0.,
            sin(deg), cos(deg), 0.,
            0., 0. , 1.;
    return R;
}


// Given a point cloud, calculate the best fit ellipsoid, returning the quadratic ellipsoid parameters
RowVector<float,10> fit_ellipsoid(const Matrix<float,3,N_CALIB> &samples)
{
    // Design matrix
    static Vector<float,N_CALIB> x = samples.row(0);
    static Vector<float,N_CALIB> y = samples.row(1);
    static Vector<float,N_CALIB> z = samples.row(2);

    // Create design matrix
    static Matrix<float,N_CALIB,10> D_T;
    D_T.setZero();
    static Matrix<float,10,N_CALIB> D;
    D.setZero();


    D_T << x.array().pow(2), y.array().pow(2), z.array().pow(2), 2*y.array()*z.array(), 2*x.array()*z.array(), 2*x.array()*y.array(), 2*x.array(), 2*y.array(), 2*z.array(), VectorXf::Ones(N_CALIB);
    D = D_T.transpose();

    // D seems correct
    //std::cout << "D: \n" << D << "\n\n";


    // Apply constraint kJ > I^2
    // The quadratic surface is an ellipse if k = 4
    int k = 4;

    // Create constrain matrix C - Eq(7)
    static Matrix<float,6,6> C;
    C <<   -1, 0.5*k-1, 0.5*k-1, 0, 0, 0,
            0.5*k-1, -1, 0.5*k-1, 0, 0, 0,
            0.5*k-1, 0.5*k-1, -1, 0, 0, 0,
            0, 0, 0, -k, 0, 0,
            0, 0, 0, 0, -k, 0,
            0, 0, 0, 0, 0, -k;

    // Create S matrix from D.T*D - Eqn(11)
    static Matrix<float,10,10> S = D * D_T;
    static Matrix<float,6,6> S11 = S.block<6,6>(0,0);
    static Matrix<float,6,4> S12 = S.block<6,4>(0,6);
    static Matrix<float,4,6> S21 = S.block<4,6>(6,0);
    static Matrix<float,4,4> S22 = S.block<4,4>(6,6);

//    std::cout << "\n\n";
//    std::cout << "S11: \n" << S11 << "\n\n";
//    std::cout << "S12: \n" << S12 << "\n\n";
//    std::cout << "S21: \n" << S21 << "\n\n";
//    std::cout << "S22: \n" << S22 << "\n\n";


    // Solve least squares - Eqn(14) and Eqn(15)
    MatrixXf M  = C.inverse() * (S11 - S12*S22.inverse() * S21);
    EigenSolver<MatrixXf> es(M);
    Vector<std::complex<float>,6> eigenvalues = es.eigenvalues();
    Matrix<std::complex<float>,6,6> eigenvectors = es.eigenvectors();

    Vector<float,6> eval = eigenvalues.array().real();
    Matrix<float,6,6> evec = eigenvectors.array().real();
//    std::cout << "Matrix to decompose: \n" << M << "\n";
//    std::cout << "Eigenvalues: \n" << eval << "\n";
//    std::cout << "Eigenvectors: \n" << evec << "\n\n";

    // Find eigenvector corresponding to largest eigenvalue
    Vector<float,6> u1;
    for (int i=0;i<6;i++)
    {
        if(eval[i] > 0.0) {
            u1 = evec.col(i);
            break;
        } else if (i == 5) {
            // No eigenvalues found
        }
    }

    Vector<float,4> u2 = (-(S22.inverse() * S21) * u1);
    Vector<float,10> U;
    U << u1, u2;
    Serial << "Output coefficients:\n";
    displayVec(U);
    return U;
}





Vector<float,12> calculate_ellipsoid_transformation(Matrix3f &M, Vector3f &n, float d)
{
    // Inverse of M
    Matrix3f M_ = M.inverse();
    // Calculate offset vector
    Vector3f b = -M_ * n;

    /******************************************************************************************
     * Equation (17) of https://teslabs.com/articles/magnetometer-calibration/
     * Used to convert ellipsoid parameters to a transformation matrix back to a unit sphere
     * Matrix square root follows https://math.stackexchange.com/a/59391
     ******************************************************************************************/
    EigenSolver<MatrixXf> es(M);
    Matrix3cf eval = es.eigenvalues().asDiagonal();
    Matrix3cf evec = es.eigenvectors();

    // Calculate diagonal matrix of square roots of eigenvectors
    Matrix3cf eval_sqrt;
    eval_sqrt << eval.array().sqrt();

    // Calculate square root of matrix
    Matrix3cf M_sqrt;
    M_sqrt << evec * eval_sqrt * evec.inverse();

    // Calculate inner square root
    std::complex<float> inner = n.transpose() * M_ * n - d;
    std::complex<float> sqrtinv = (std::complex<float>)pow(sqrt(inner),-1);

    // Calculate final transformation matrix
    Matrix3cf A_1;
    A_1 << (sqrtinv * M_sqrt).array();
    Matrix3f A_1real = A_1.array().real();

    // Return as a vector
    Map<VectorXf> v1(A_1real.data(),9);
    Map<VectorXf> v2(b.data(), 3);
    Vector<float,12> V;
    V << v1, v2;
    //A_1 << A_1.inverse();
    // std::cout << "Transformation matrix: \n" << v1.reshaped(3,3) << "\n\n";
    // std::cout << "Bias vector: \n" << v2.reshaped(3,1) << "\n\n";
    return V;
}

// Given a point cloud, find a plane of best fit and return the vector normal to this plane
Vector3f NormalVec(MatrixXf point_cloud){
  Vector3f normal;
  MatrixXf left_singular_mat;
  // int U_cols;

  // Subtract mean from each point otherwise its wrong XD
  // https://www.ltu.se/cms_fs/1.51590!/svd-fitting.pdf
  Vector3f point_vec = point_vec.colwise()-point_vec.rowwise().mean();

  JacobiSVD<MatrixXf> svd(point_vec, ComputeThinU | ComputeThinV);
  left_singular_mat = svd.matrixU();
  // U_cols = left_singular_mat.cols();
  // 3rd col of U contains normal vec
  normal << left_singular_mat(0,2), left_singular_mat(1,2), left_singular_mat(2,2);

  return normal;
};

// Calculates a 3x3 matrix representing a rotation of "rads" about the x axis
Matrix3f XRotation(float rads)
{
    Matrix3f T;
    T << 1,0,0,
         0,cos(rads),-sin(rads),
         0,sin(rads),cos(rads);
    return T;
};

// // Standard deviation of matrix (cols = x,y,z,...)
// float StdDev(MatrixXf m)
// {
//     VectorXf mean = m.rowwise().mean();
//     return (m.rowwise() - mean).colwise().norm().array().sum();
// }

float StdDev(std::queue<Vector3f> q);
