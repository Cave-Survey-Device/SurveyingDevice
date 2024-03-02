#include "InertialAlignment.h"

namespace NumericalMethods{

Vector<float,10> alignMagAcc(const Matrix<float,3,N_ALIGN_MAG_ACC> &g_in, const Matrix<float,3,N_ALIGN_MAG_ACC> &m_in) {
    /************************************************************************************
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
     * 
     * R_hat and s_hat could then be used for an iterative approach but this is not used
    ************************************************************************************/

    // Used MAG.I.CAL alignment so only 12 inputs
    static int K = N_ALIGN_MAG_ACC;
    static Matrix<float,N_ALIGN_MAG_ACC,9> A;
    static Matrix<float,3,N_ALIGN_MAG_ACC> m, g;

    m = m_in;
    g = g_in;
    m.colwise().normalize();
    g.colwise().normalize();

    // Statically allocate variables of known size
    static Vector<float,10> out;
    static RowVector<float,9> vecR;
    static Vector3f mk, gk;
    static Matrix3f H, U, V, Sig, Uhat, Rhat;

    // Step 1 - Form the design matrix, A
    for (int i=0; i<K; i++)
    {
        mk = m.col(i);
        gk = g.col(i);
        A.row(i) << kron(mk,gk).transpose();
    }

    // Step 2 - Form H
    H = ((A.transpose()*A).inverse() * A.transpose() * MatrixXf::Ones(K,1)).reshaped(3,3);

    // Step 3 - Solve lstsq
    static JacobiSVD<Matrix3f> svd;

    svd.compute(H,ComputeFullU | ComputeFullV);
    U = svd.matrixU();
    V = svd.matrixV();
    Sig = svd.singularValues().asDiagonal();

    // Step 4
    Uhat = sign(H.determinant()) * U;
    Rhat = Uhat * V.transpose();

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
    out(9) = asin(shat);
    return out;
}

void alignMagAcc(const Matrix<float,3,N_ALIGN_MAG_ACC> &g_in, const Matrix<float,3,N_ALIGN_MAG_ACC> &m_in, 
                            Matrix3f &R_align, float &inclination_angle) {

    
    Vector<float,10> U = alignMagAcc(g_in, m_in);
    R_align = U.segment(0,9).reshaped(3,3);
    inclination_angle = U(9);
}


}