#include "NumericalMethods_csd.h"
#include "sensor_config_csd.h"
// ------------------------------------------------- GENERAL FUNCTIONS -------------------------------------------------
MatrixXf kron(MatrixXf m1, MatrixXf m2)
{
    int m = m1.rows();
    int n = m1.cols();
    int p = m2.rows();
    int q = m2.cols();

    MatrixXf out(m*p, n*q);
    for(int i=0; i<m; i++)
    {
        for(int j=0; j<n; j++) {
            out.block(i*p,j*q,p,q) = m1(i,j) * m2;
        }
    }
    return out;
}

int sign(float f)
{
    if (f>=0)
    {
        return 1;
    } else {
        return -1;
    }
}
//  ------------------------------------------------ ROTATION FUNCTIONS  ------------------------------------------------

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

//  ------------------------------------------ INERTIAL CALIBRATION FUNCTIONS  -------------------------------------------

RowVector<float,10> fit_ellipsoid(const MatrixXf &samples, int n_samples)
{
    // Design matrix
    static Matrix<float,6,6> C;
    static Matrix<float,10,10> S;
    static Matrix<float,6,6> S11;
    static Matrix<float,6,4> S12;
    static Matrix<float,4,6> S21;
    static Matrix<float,4,4> S22;
    static Matrix<float,6,6> M;
    static EigenSolver<Matrix<float,6,6>> es;
    static Vector<float,6> eval;
    static Matrix<float,6,6> evec;
    static Vector<std::complex<float>,6> eigenvalues;
    static Matrix<std::complex<float>,6,6> eigenvectors;

    if (n_samples == -1)
    {
        n_samples = samples.cols();
    }

    const VectorXf &x = samples.row(0).segment(0,n_samples);
    const VectorXf &y = samples.row(1).segment(0,n_samples);
    const VectorXf &z = samples.row(2).segment(0,n_samples);

    MatrixXf D_T(n_samples,10);

    // Create design matrix
    D_T.setZero();
    C.setZero();

    displayVec(x.array().pow(2));

    D_T.col(0) << x.array().pow(2);
    D_T.col(1) << y.array().pow(2);
    D_T.col(2) << z.array().pow(2);
    D_T.col(3) << 2*y.array()*z.array();
    D_T.col(4) << 2*x.array()*z.array();
    D_T.col(5) << 2*x.array()*y.array();
    D_T.col(6) << 2*x.array();
    D_T.col(7) << 2*y.array();
    D_T.col(8) << 2*z.array();
    D_T.col(9) << VectorXf::Ones(n_samples);

    int k = 4;

    // Create constrain matrix C - Eq(7)
    C <<   -1, 0.5*k-1, 0.5*k-1, 0, 0, 0,
            0.5*k-1, -1, 0.5*k-1, 0, 0, 0,
            0.5*k-1, 0.5*k-1, -1, 0, 0, 0,
            0, 0, 0, -k, 0, 0,
            0, 0, 0, 0, -k, 0,
            0, 0, 0, 0, 0, -k;

    // Create S matrix from D*D.T - Eqn(11)
    S = D_T.transpose() * D_T;
    S11 = S.block<6,6>(0,0);
    S12 = S.block<6,4>(0,6);
    S21 = S.block<4,6>(6,0);
    S22 = S.block<4,4>(6,6);

    // Solve least squares - Eqn(14) and Eqn(15)
    M  = C.inverse() * (S11 - S12*S22.inverse() * S21);
    es.compute(M);
    eigenvalues = es.eigenvalues();
    eigenvectors = es.eigenvectors();

    eval = eigenvalues.array().real();
    evec = eigenvectors.array().real();

    // Find eigenvector corresponding to largest eigenvalue
    Vector<float,6> u1;
    float max_eval = eval[0];
    for (int i=0;i<6;i++)
    {
        if(eval[i] > max_eval) {
            max_eval = eval[i];
            u1 = evec.col(i);
        } else if (i == 5) {
            // No eigenvalues found
        }
    }

    Vector<float,4> u2;
    if (S22.determinant() < 0.05)
    {
        u2 = -(pseudoInverse(S22) * S21) * u1;
    } else {
        u2 = (-(S22.inverse() * S21) * u1);
    }

    Vector<float,10> U;
    U << u1, u2;

    return U;
}

Vector<float,12> calculate_ellipsoid_transformation(Matrix3f &M, Vector3f &n, float d)
{
    Serial << "Beginning elipsoid transformation calculations...\n";
    Serial << "Finding inverse of M...\n";
    // Inverse of M
    Matrix3f M_ = M.inverse();
    // Calculate offset vector
    Vector3f b = -M_ * n;

    /******************************************************************************************
     * Equation (17) of https://teslabs.com/articles/magnetometer-calibration/
     * Used to convert ellipsoid parameters to a transformation matrix back to a unit sphere
     * Matrix square root follows https://math.stackexchange.com/a/59391
     ******************************************************************************************/
    Serial << "Calculating eigendecomposition...\n";
    EigenSolver<MatrixXf> es(M);
    Matrix3cf eval = es.eigenvalues().asDiagonal();
    Matrix3cf evec = es.eigenvectors();

    Serial << "Calculating eigenvalues sqrt...\n";
    // Calculate diagonal matrix of square roots of eigenvectors
    Matrix3cf eval_sqrt;
    eval_sqrt << eval.array().sqrt();

    // Calculate square root of matrix
    Serial << "Calculating matrix sqrt...\n";
    Matrix3cf M_sqrt;
    M_sqrt << evec * eval_sqrt * evec.inverse();

    // Calculate inner square root
    Serial << "Calculate inner sqrt..\n";
    std::complex<float> inner = n.transpose() * M_ * n - d;
    std::complex<float> sqrtinv = (std::complex<float>)pow(sqrt(inner),-1);


    // Calculate final transformation matrix
    Serial << "Calculate final transformation matrix...\n";
    Matrix3cf A_1;
    A_1 << (sqrtinv * M_sqrt).array();
    Matrix3f A_1real = A_1.array().real();

    Serial << "Create output vector...\n";
    // Return as a vector
    Map<VectorXf> v1(A_1real.data(),9);
    Map<VectorXf> v2(b.data(), 3);
    Vector<float,12> V;
    V << v1, v2;
    //A_1 << A_1.inverse();
    // std::cout << "Transformation matrix: \n" << v1.reshaped(3,3) << "\n\n";
    // std::cout << "Bias vector: \n" << v2.reshaped(3,1) << "\n\n";
    Serial << "Returning...\n";
    return V;
}

Vector3f NormalVec(const MatrixXf &point_cloud){
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

float StdDev(MatrixXf m)
{
    VectorXf mean = m.rowwise().mean();
    return (m.colwise() - mean).colwise().norm().array().sum();
}

// ------------------------------------------------ ALIGNMENT FUNCTIONS  ------------------------------------------------

Vector<float,10> AlignMagAcc(const MatrixXf &g, const MatrixXf &m) {
    /**************************************************************
     * Given a set of calibrated magnetometer and accelerometer data, this function
     * finds the least squares best fit for the alignment of the sensor axis and outputs
     * a rotation matrix for correcting the magnetometer and the magnetic inclination at
     * the location of measurement.
     * 
     * 1. Calculate the design matirx, A
     * 2. Solve Ax=b with least squares
     * 3. Find SVD of least squares solution
     * 4. Calculate U_hat and R_hat
     * 5. Calculate s_hat
    */
    #ifdef NUMERICAL_METHODS_STANDALONE
    int K = g.cols();
    static MatrixXf A(K,9);
    #else
    int K = N_INERTIAL_ALIGNMENT;
    static Matrix<float,N_INERTIAL_ALIGNMENT,9> A(K,9);
    #endif
    static Vector<float,10> out;
    static RowVector<float,9> vecR;
    

    Vector3f mk;
    Vector3f gk;

    // Step 1 - Form the design matrix
    for (int i=0; i<K; i++)
    {
        mk = m.col(i);
        gk = g.col(i);
        A.row(i) << kron(mk,gk).transpose();
    }

    // Step 2 - Solve lstsq
    Matrix3f H = ((A.transpose()*A).inverse() * A.transpose() * MatrixXf::Ones(K,1)).reshaped(3,3);
;
    // Step 3 - 
    JacobiSVD<MatrixXf> svd(H, ComputeThinU | ComputeThinV);
    Matrix3f U = svd.matrixU();
    Matrix3f V = svd.matrixV();
    Matrix3f Sig = svd.singularValues().asDiagonal();

    // Step 4
    Matrix3f Uhat = sign(H.determinant()) * U;
    Matrix3f Rhat = Uhat * V.transpose();

    // Step 5
    float shat = 0;
    for (int i=0; i<K; i++)
    {
        gk = g.col(i);
        mk = m.col(i);
        shat += gk.transpose() * Rhat * mk;
    }
    shat = shat * 1/K;

    out.segment(0,9) << Rhat.reshaped(9,1);
    out(9) = shat;

    return out;
}