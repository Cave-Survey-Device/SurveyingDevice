 // Allocated extra data as static as size will never change between calls
#define ALIGN_MAG_ACC_FIXED_SIZE

/**
 * @brief Given a set of calibrated magnetometer and accelerometer data, this function
     * finds the least squares best fit for the alignment of the sensor axis and outputs
     * a rotation matrix for correcting the magnetometer and the magnetic inclination at
     * the location of measurement.
 * 
 * @param g_in 
 * @param m_in 
 * @return Vector<float,10> 
 */
Vector<float,10> AlignMagAcc(const MatrixXf &g_in, const MatrixXf &m_in) {
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

    // Normalise inout data - required assumption
    // This is not very emmory efficient, consider changing...



    // To increase efficieny these use a directive to check whether the input will be the same
    // size as is likely in an embedded system. This is set with the ALIGN_MAG_ACC_FIXED_SIZE directive
    #ifdef ALIGN_MAG_ACC_FIXED_SIZE
        static int K = g.cols();
        static MatrixXf A(K,9);
        static MatrixXf m = m_in;
        static MatrixXf g = m_in;
    #else
        int K = g.cols();
        MatrixXf A(K,9);
        MatrixXf m, g;
    #endif

    m = m_in;
    g = g_in;
    m.normalize();
    g.normalize();

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
    JacobiSVD<MatrixXf> svd(H, ComputeThinU | ComputeThinV);
    U = svd.matrixU();
    V = svd.matrixV();
    Sig = svd.singularValues().asDiagonal();

//    // From single step paper...
//    float shat = pow(3/(Sig(0)*Sig(0)+Sig(1)*Sig(1)+Sig(2)*Sig(2)),0.5);
//    Matrix3f Rhat = sign(H.determinant())*U*V.transpose();


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
    out(9) = shat;

    return out;
}
