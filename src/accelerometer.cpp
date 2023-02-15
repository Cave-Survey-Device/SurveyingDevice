#include "accelerometer.h"

Accelerometer::Accelerometer()
{
    correction_transformation << 1, 0, 0,
                                 0, 1, 0,
                                 0, 0, 1;
};

void Accelerometer::update(){
    get_raw_data();
    corrected_gravity_data = correction_transformation*raw_gravity_data;
}

Vector3d Accelerometer::get_grav_vec()
{
    return corrected_gravity_data;
};


MatrixXd kronecker(MatrixXd a, MatrixXd b)
{
    int row;
    int col;
    int m = a.rows();
    int n = a.cols();
    int p = b.cols();
    int q = b.rows();

    MatrixXd out (p*m,q*n);
    
    for (row=0;row<m;row++)
    {
        for (col=0;col<n;col++)
        {
            // Block size pxq starting at (row*p, col*q)
            out.block(p,q,row*p,col*q) = a(row,col) * b;
        }
    }
    return out;
}



bool Accelerometer::calibrate()
{
    update();
    samples_mat << samples_mat.block(0,1,3,N_ACC_SAMPLES-1), get_grav_vec();
    sample_num++;

    // Check standard deviation
    Vector3d mean_acceleration = samples_mat.rowwise().mean();
    Matrix<double,3,N_ACC_SAMPLES> minus_mean = samples_mat.colwise()-mean_acceleration;
    minus_mean = minus_mean.array()/3;
    double sd = minus_mean.norm();
    if (sd < CALIB_VAR_LIM)
    {
        calib_data.col(calibration_num) = samples_mat.rowwise().mean();
        sample_num = 0;
        calibration_num++;
    }

    // if all data collected
    if (calibration_num == 23)
    {

        // RUN NEWTON SCHEME A LOAD OF TIMES

        calibration_num = 0;
        // Run calibartion code...
        return true;
        
    }
    return false;
}



void calculate_newton_iteration()
{
    /********************************************************************************************************************
     * The goal here is to run the newton interation method described in https://doi.org/10.1155/2020/4617365.
     * To do this, the Jacobian and Hessian matrices must be formulated for the function INSERT NUM in the documentation
     * 
     * When using Eigen, Matrix<double,3,1> produces Vector3d and Matrix<double,1,3> produces Rowector3d
    *********************************************************************************************************************/
    const int N =  ACCEL_CALIBRATION_N;
    Matrix<double,3,N> yb;
    Matrix<double,3,N> ym;
    Matrix<double,3,N> d;
    Vector<double,N> lambda;

    Vector3d b_a;
    
    Vector3d ym_k;
    Vector3d yb_k;
    Vector3d d_k;

    Matrix3d T;
    Matrix3d T_a;
    
    double lambda_k;

    RowVector<double, 12+4*N> jacobian;
    Matrix<double, 12+4*N, 12+4*N> hessian;
    

    /*********************************************************************
     * Construct Jacobian which is a 1x(4*ACCEL_CALIBRATION_N+12) matrix
    **********************************************************************/
    // df/dT_a 9x1
    int k;
    Vector<double,9> J_1;
    J_1.setZero();

    for (k=0; k<N;k++)
    {
        yb_k = yb.col(k);
        J_1 = J_1 + kronecker(yb_k,(d_k - T*yb_k));
    }
    J_1 = -2*J_1;

    // df/db_a 3x1
    Vector3d J_2 = 2 * (T*yb - d).rowwise().sum();

    // df/dyb_k 3kx1
    Matrix<double,3,N> J_3_mat;
    for (k=0; k<N;k++)
    {
        yb_k = yb.col(k);
        J_3_mat.col(k) = -2 * T_a.transpose() * (d_k - T*yb_k) + 2*lambda_k*yb_k;
    }
    // Convert to Vector << col1(0), col1(1), col1(2), col2(0), col2(1), col2(2), ... , coln(0), coln(1), coln(2)
    RowVector<double,3*N> J_3 = J_3_mat.reshaped();

    // df/dlambda_k kx1
    RowVector<double,N> J_4 = (yb.colwise().norm().array().pow(2)) - 1;

    // Jacobian
    jacobian << J_1.transpose(), J_2.transpose(), J_3, J_4;


    /*********************************************************************************************
     * Construct Hessian which is a (4*ACCEL_CALIBRATION_N+12)x(4*ACCEL_CALIBRATION_N+12) matrix
    **********************************************************************************************/
    Vector<double,9> tempV;

    // H11 9x9
    Matrix<double,9,9> H11;
    H11.setZero();

    for (k=0;k<N;k++)
    {
        //yb_k
        yb_k = yb.col(k);

        //yb_k * yb_k.T
        Matrix3d ykyk;
        ykyk = kronecker(yb_k, yb_k.transpose());

        // ykyk kronecker I
        H11 = H11 + kronecker(ykyk, Matrix3d::Identity(3,3));
    }
    H11 = 2*H11;




    // H12 9x3
    Matrix<double,9,3> H12;
    H12.setZero();

    for (k=0;k<N;k++)
    {
        
        yb_k = yb.col(k); //yb_k
        H12 = H12 + kronecker(yb_k, Matrix3d::Identity(3,3)); // yb_k kronecker I
    }
    H12 = 2*N*H12;

    // H21 3x9
    Matrix<double,3,9> H21 = H12.transpose();


    // H13 9x3N
    Matrix<double,9,3*N> H13;
    for (k=0;k<N;k++)
    {
        ym_k = ym.col(k);
        d_k = ym_k - b_a;
        yb_k = yb.col(k); //yb_k
        H13.block(k*3,0,3,3) = 2*kronecker(yb_k,Matrix3d::Identity(3,3))*T - kronecker(Matrix3d::Identity(3,3), (d_k-T*yb_k));
    }

    // H31 3Nx9
    Matrix<double,3*N,9> H31 = H13.transpose(); 


    // H14 9xk
    Matrix<double,9,N> H14;
    H14.setZero();
    // H41 kx9
    Matrix<double,N,9> H41 = H14.transpose(); 


    // H22 2x2
    Matrix3d H22 = Matrix3d::Identity(3,3);
    H22 = H22*2*N;


    // H23 3x3k
    Matrix<double,3,3*N> H23;
    for (k=0;k<N;k++)
    {
        H23.block(k*3,0,3,3) = 2*T;
    }
    // H32 3kx3
    Matrix<double,3*N,3> H32 = H23.transpose(); 


    // H24 3xk
    Matrix<double,3,N> H24;
    H24.setZero();
    // H42 kx3
    Matrix<double,N,3> H42 = H24.transpose(); 


    // H33 k3x3
    Matrix<double,3*N,3> H33;
    for (k=0;k<N;k++)
    {
        lambda_k = lambda(k);
        H33.block(k*3,0,3,3) = 2*T.transpose() * T + 2*MatrixX3d::Identity(3,3)*lambda_k;
    }


    // H34 3kx1 
    Matrix<double,1,3*N> H34;
    for (k=0;k<N;k++)
    {
        yb_k = yb.col(k);
        H34.block(k*3,0,3,3) = 2*yb_k;
    }
    // H43 1x3k
    Matrix<double,1,3*N> H43 = H34.transpose(); 


    // H44 kxk
    Matrix<double,N,N> H44;
    H44.setZero();

    // Hessian
    hessian.setZero();
    hessian.block(0,0,9,9) = H11;
    hessian.block(0,9,9,3) = H12;
    hessian.block(0,12,9,3*N) = H13;
    hessian.block(0,12+3*N,9,N) = H14;

    hessian.block(9,0,9,9) = H21;
    hessian.block(9,9,9,3) = H22;
    hessian.block(9,12,9,3*N) = H23;
    hessian.block(9,12+3*N,9,N) = H24;

    hessian.block(12,0,9,9) = H31;
    hessian.block(12,9,9,3) = H32;
    hessian.block(12,12,9,3*N) = H33;
    hessian.block(12,12+3*N,9,N) = H34;

    hessian.block(12+3*N,0,9,9) = H41;
    hessian.block(12+3*N,9,9,3) = H42;
    hessian.block(12+3*N,12,9,3*N) = H43;
    
    hessian.block(12+3*N,12+3*N,N,N) = H44;


    // Multiply Jacobian by inverse of hessian

}