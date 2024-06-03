#include "FittingFuncs.h"


namespace NumericalMethods {

Vector3f normalVec(const Ref<const MatrixXf> &point_cloud)
{
    Vector3f normal;
    MatrixXf left_singular_mat;

    // Subtract mean from each point otherwise its wrong XD
    // https://www.ltu.se/cms_fs/1.51590!/svd-fitting.pdf
    MatrixXf mean_adj_point_cloud = point_cloud;
    mean_adj_point_cloud = mean_adj_point_cloud.colwise()-mean_adj_point_cloud.rowwise().mean();

    JacobiSVD<MatrixXf> svd(mean_adj_point_cloud, ComputeThinU | ComputeThinV);
    left_singular_mat = svd.matrixU();
    // U_cols = left_singular_mat.cols();
    // 3rd col of U contains normal vec
    normal << left_singular_mat(0,2), left_singular_mat(1,2), left_singular_mat(2,2);

    return normal;
};


RowVector<float,10> fitEllipsoid(const Ref<const MatrixXf> &samples)
{
    // Uses ~400 floats
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
    static Vector<float,6> u1;
    static Vector<float,4> u2;
    static Vector<float,10> U;

    static int n_samples;
    n_samples = samples.cols();

    const VectorXf &x = samples.row(0).segment(0,n_samples);
    const VectorXf &y = samples.row(1).segment(0,n_samples);
    const VectorXf &z = samples.row(2).segment(0,n_samples);

    // This could end up being very large! Be careful!
    // Check limits on available memory!
    // Remaining stack size MUST be greater than  n_samples x 10 x 4bytes!
    MatrixXf D_T(n_samples,10);

    // Create design matrix
    D_T.setZero();
    C.setZero();

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
    float max_eval = eval[0];
    for (int i=0;i<6;i++)
    {
        if(eval[i] >= max_eval) {
            max_eval = eval[i];
            u1 = evec.col(i);
        }
    }
    if (max_eval < 0.0)
    {
        Serial.println("No positive eigenvalues found!");
    }

    // To preserve stabikity of calculations. Use pseudoinverse it determinant too small
    if (S22.determinant() < 0.05)
    {
        u2 = -(pseudoInverse(S22) * S21) * u1;
    } else {
        u2 = (-(S22.inverse() * S21) * u1);
    }

    // Form output vector
    U << u1, u2;
    return U;
}

void fitEllipsoid(const Ref<const MatrixXf> &samples, Matrix3f &M_out, Vector3f &n_out, float &d_out)
{
    static Vector<float,10> U;
    U << fitEllipsoid(samples);

    // Form output vector
    M_out << U[0], U[5], U[4], U[5], U[1], U[3], U[4], U[3], U[2];
    n_out << U[6], U[7], U[8];
    d_out = U[9];
}


Vector<float,12> calculateEllipsoidTransformation(const Matrix3f &M, const Vector3f &n, const float &d)
{
    /*
    * Statics are not used here as the fit_ellipsoid function executing will ensure that there is enough memory left on the stack.
    * If for some reason there is not enough, one should consider running this immediately after calculating the fitting parameters
    */ 
    

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

    return V;
}

Vector<float,12> calculateEllipsoidTransformation(const RowVector<float,10> &U)
{
    Matrix3f M;
    Vector3f n;
    float d;

    M << U[0], U[5], U[4], U[5], U[1], U[3], U[4], U[3], U[2];
    n << U[6], U[7], U[8];
    d = U[9];

    return calculateEllipsoidTransformation(M,n,d);
}


void calculateEllipsoidTransformation(const RowVector<float,10> &U, Matrix3f &R_out, Vector3f &b_out)
{
   Vector<float,12> V = calculateEllipsoidTransformation(U);

    R_out << V[0], V[1], V[2], V[3], V[4], V[5], V[6], V[7], V[8];
    b_out << V[9], V[10], V[11];
}

void calculateEllipsoidTransformation(const Matrix3f &M, const Vector3f &n, const float &d, Matrix3f &R_out, Vector3f &b_out)
{
   Vector<float,12> V = calculateEllipsoidTransformation(M,n,d);

    R_out << V[0], V[1], V[2], V[3], V[4], V[5], V[6], V[7], V[8];
    b_out << V[9], V[10], V[11];
}

}