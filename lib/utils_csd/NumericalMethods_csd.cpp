#include "NumericalMethods_csd.h"

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

RowVector<float,10> fit_ellipsoid(const MatrixXf &samples)
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

    Serial << "assigning data\n";

    const VectorXf &x = samples.row(0);
    const VectorXf &y = samples.row(0);
    const VectorXf &z = samples.row(0);

    MatrixXf D_T(10,samples.cols());

    // Create design matrix
    D_T.setZero();
    C.setZero();

    Serial << "Getting transpose1\n";
    D_T.col(0) << x.array().pow(2);
    D_T.col(1) << y.array().pow(2);
    D_T.col(2) << z.array().pow(2);

    Serial << "Getting transpose2\n";
    D_T.col(3) << 2*y.array()*z.array();
    D_T.col(4) << 2*x.array()*z.array();
    D_T.col(5) << 2*x.array()*y.array();

    Serial << "Getting transpose3\n";
    D_T.col(6) << 2*x.array();
    D_T.col(7) << 2*y.array();
    D_T.col(8) << 2*z.array();

    Serial << "Getting transpose4\n";
    D_T.col(9) << VectorXf::Ones(samples.cols());

    int k = 4;

    // Create constrain matrix C - Eq(7)
    C <<   -1, 0.5*k-1, 0.5*k-1, 0, 0, 0,
            0.5*k-1, -1, 0.5*k-1, 0, 0, 0,
            0.5*k-1, 0.5*k-1, -1, 0, 0, 0,
            0, 0, 0, -k, 0, 0,
            0, 0, 0, 0, -k, 0,
            0, 0, 0, 0, 0, -k;

    // Create S matrix from D*D.T - Eqn(11)
    Serial << "Setting S\n";
    S = D_T.transpose() * D_T;
    S11 = S.block<6,6>(0,0);
    S12 = S.block<6,4>(0,6);
    S21 = S.block<4,6>(6,0);
    S22 = S.block<4,4>(6,6);

    Serial << "Solving...\n";
    // Solve least squares - Eqn(14) and Eqn(15)
    M  = C.inverse() * (S11 - S12*S22.inverse() * S21);
    es.compute(M);
    eigenvalues = es.eigenvalues();
    eigenvectors = es.eigenvectors();

    eval = eigenvalues.array().real();
    evec = eigenvectors.array().real();
    Serial << "evals: \n";
    displayVec(eval);
    Serial << "\n";
    Serial << "evecs: \n";
    displayMat(evec);
    Serial << "\n";
//    std::cout << "Matrix to decompose: \n" << M << "\n";
//    std::cout << "Eigenvalues: \n" << eval << "\n";
//    std::cout << "Eigenvectors: \n" << evec << "\n\n";

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

    Serial << "Output coefficients:\n";
    displayVec(U);
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

float J(const MatrixXf &f, const MatrixXf &m, const Vector<float, 10> &X)
{
    float J = 0;
    int n = m.cols();
    Vector<float, 9> dJ_dR;
    Matrix3f R;
    R << X.segment(0,9).reshaped(3,3);
    float d = X(9);

    Vector3f mk;
    Vector3f fk;

    for(int k=0;k<n;k++)
    {
        mk = m.col(k);
        fk = f.col(k);
        J += pow( (sin(d)-fk.transpose()*R*mk) / (fk.norm() * mk.norm()),2);
    }
    J += pow((R*R.transpose()-Matrix3f::Identity()).norm(),2);
    J += pow((R.determinant() - 1),2);
    return J;
}

Vector<float, 9> dJ_dR (const MatrixXf &f, const MatrixXf &m, const Vector<float, 10> &X)
{
    int n = m.cols();
    Vector<float, 9> dJ_dR;
    Matrix3f R;
    R << X.segment(0,9).reshaped(3,3);
    float d = X(9);

    dJ_dR.setZero();
    Vector3f mk;
    Vector3f fk;
    float norms;
    float tmp;

    for(int k=0;k<n;k++)
    {
        mk = m.col(k);
        fk = f.col(k);
        tmp = fk.transpose()*R*mk;
        norms = (fk.norm() * mk.norm());
        dJ_dR += -2*(   (sin(d) - tmp / norms) * (kron(mk,fk) / norms) );
    }

    dJ_dR += 4* (R*R.transpose()*R - R).reshaped(9,1);
    dJ_dR += 2* (R.determinant()-1)*R.adjoint().transpose().reshaped(9,1);

    return dJ_dR;
}

float dJ_dd (const MatrixXf &f, const MatrixXf &m, const Vector<float, 10> &X)
{
    int n = m.cols();
    float dJ_dd = 0;
    Matrix3f R;
    R << X.segment(0,9).reshaped(3,3);
    float d = X(9);
    Vector3f mk;
    Vector3f fk;
    float tmp;
    for(int k=0;k<n;k++)
    {
        mk = m.col(k);
        fk = f.col(k);
        tmp = (fk.transpose() * R * mk);
        tmp = tmp / (fk.norm() * mk.norm());
        dJ_dd += (sin(d) - tmp);
    }
    dJ_dd = dJ_dd *  2 * cos(d);
    return dJ_dd;

}

Vector<float,10> GradJ(const MatrixXf &f, const MatrixXf &m, const Vector<float, 10> &X)
{
    Vector<float,10> grad_J;
    grad_J.segment(0,9) << dJ_dR(f,m,X).reshaped(9,1);
    grad_J(9) = dJ_dd(f,m,X);

    return grad_J;
}

Vector<float,10> Align(const MatrixXf &f, const MatrixXf &m)
{
    Vector<float,10> X;
    Vector<float,10> dx;
    float alpha = 0.9; // Loss scaling parameter (how much smaller does next step have to be compared to current step?)
    float beta = 0.75; // Line search scaling parameter
    float t  = 10; // Step size scale
    float cost = J(f,m,X);

    X.segment(0,9) << Matrix3f::Identity().reshaped(9,1);
    X(9) = Deg2Rad(54);

    while (cost > 0.1)
    {
        // Calculate dJ
        dx = -GradJ(f,m,X);

        // Find scale over which to operate gradient descent at a given step
        t = 1;
        while (    J(f,m, X + t*dx) > J(f,m,X) + alpha*t*-dx.transpose()*dx   )
        {
            t = t * beta;
        }

        // Move to next step
        X = X + dx * t;
        cost = J(f, m, X);
        // std::cout << "Cost: " << cost << "\n";

        // If timestep too small
        if(t < 0.00000001)
        {
            break;
        }
    }
    return X;
}