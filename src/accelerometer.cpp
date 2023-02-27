#include "accelerometer.h"

// PRINT MATRIX (float type)
// By: randomvibe
//-----------------------------
void print_mtxf(const Eigen::MatrixXf& X)  
{
   int i, j, nrow, ncol;
   nrow = X.rows();
   ncol = X.cols();
   Serial.print("nrow: "); Serial.println(nrow);
   Serial.print("ncol: "); Serial.println(ncol);       
   Serial.println();
   for (i=0; i<nrow; i++)
   {
       for (j=0; j<ncol; j++)
       {
           Serial.print(X(i,j), 6);   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println();
}

MatrixXf kronecker(MatrixXf a, MatrixXf b)
{
    int row;
    int col;
    int m = a.rows();
    int n = a.cols();
    int p = b.cols();
    int q = b.rows();

    MatrixXf out (p*m,q*n);
    
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

Vector3f Accelerometer::get_grav_vec()
{
    return corrected_gravity_data;
};





bool Accelerometer::calibrate()
{
    update();
    samples_mat << samples_mat.block(0,1,3,N_ACC_SAMPLES-1), get_grav_vec();
    sample_num++;

    // Check standard deviation
    Vector3f mean_acceleration = samples_mat.rowwise().mean();
    Matrix<float,3,N_ACC_SAMPLES> minus_mean = samples_mat.colwise()-mean_acceleration;
    minus_mean = minus_mean.array()/3;
    float sd = minus_mean.norm();
    if (sd < CALIB_VAR_LIM)
    {
        calib_data.col(calibration_num) = samples_mat.rowwise().mean();
        sample_num = 0;
        calibration_num++;
    }

    // if all data collected
    if (calibration_num >= ACCEL_CALIBRATION_N)
    {

        // RUN NEWTON SCHEME A LOAD OF TIMES

        calibration_num = 0;
        // Run calibartion code...
        return true;
        
    }
    return false;
}

bool Accelerometer::test_calibration()
{
    // Generate set of mock calibration data
    // Define test parameters:
    Matrix<float, 3,ACCEL_CALIBRATION_N> ym; // Sensor Readings

    Vector3f b_a; // Zero-bias error
    b_a << 0.1, -0.1, 0.25;

    Matrix3f Cb_n; // Transformation from body frame to navigation frame 
    Cb_n << 1,0,0,
            0,1,0,
            0,0,1;

    Matrix3f C_s; // Scale factor deviation
    float S_x, S_y, S_z;
    S_x = 0.95;
    S_y = 0.975;
    S_z = 1;
    C_s <<  1+S_x, 0, 0,
            0, 1+S_y, 0,
            0, 0, 1+S_z;

    Matrix3f C_n; // Non-orthogonal Error
    float a,b,c;
    a = PI/10;
    b = -PI/10;
    c = 0;
    C_n <<  1, sin(a), -sin(b),
            0, cos(a), sin(c)*cos(b),
            0, 0, cos(c)*cos(b);

    Vector3f epsilon = {0,0,0}; // Gaussian white noise

    int x,y;
    Vector3f g_vec = {0, 0, -9.81};
    float x_ang, y_ang;
    Matrix3f rotation_mat;
    for (x=0;x<4;x++)
    {
        
        x_ang = x*PI/2;
        rotation_mat <<   1, 0, 0,
                        0, cos(x_ang), -sin(x_ang),
                        0, sin(x_ang), cos(x_ang);
        ym.col(x) << C_s*C_n*Cb_n*rotation_mat*g_vec + b_a + epsilon;

    }
    for (y=0;y<2;y++)
    {
        y_ang = y*PI;
        rotation_mat <<   cos(y_ang), 0, sin(y_ang),
                        0, 1 ,0,
                        -sin(y_ang), 0, cos(y_ang);
        ym.col(3+y) << C_s*C_n*Cb_n*rotation_mat*g_vec + b_a + epsilon;
    }

    Serial.println("T_a:");
    print_mtxf(C_s*C_n);

    Serial.println("b_a:");
    print_mtxf(b_a);

    run_newton(ym);

    return true;
}

void Accelerometer::run_newton(Matrix<float,3,ACCEL_CALIBRATION_N> ym)
{
    /**********************************************
     * Newton scheme so requires an initial guess,
     * guess calculated below...   
    **********************************************/
    // Initial estimate
    Matrix3f R_a;
    Matrix3f b_a;
    int i;
    Vector3f ym_i;

    // Construct Ax = b (equation has 10 parameters see Eq. 17)
    Matrix<float,ACCEL_CALIBRATION_N,10> A; 
    Vector<float,10> x;
    Vector<float,ACCEL_CALIBRATION_N> b;
    b.setZero();

    for (i=0;i<ACCEL_CALIBRATION_N;i++)
    {
        ym_i = ym.col(i);
        A.row(i) <<
        ym_i(0)*ym_i(0), ym_i(1)*ym_i(1), ym_i(2)*ym_i(2), // Squares
        2*ym_i(0)*ym_i(1), 2*ym_i(0)*ym_i(2), 2*ym_i(1)*ym_i(2), // Combinations
        2*ym_i(0), 2*ym_i(1), 2*ym_i(2), // Individuals
        1; // Constant
    }

    // Solve Ax = b
    x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    // Assign E, F, and G
    Matrix3f E;
    Vector3f F;
    float G;

    E << x(0), x(3), x(4),
         x(3), x(1), x(5),
         x(4), x(5), x(2);

    F << x(6), x(7), x(8);

    G = x(9);

    // Compute R0_a and b0_a
    Matrix3f R0_a = E.llt();
    Vector3f b0_a = E.inverse() * F;

    /**********************************************
     * Run the newton iteration scheme...   
    **********************************************/
    Vector<float,12+4*ACCEL_CALIBRATION_N> theta;

    Matrix3f T_a = R0_a.transpose();
    Matrix<float, 3,  ACCEL_CALIBRATION_N> yb;
    RowVector<float, ACCEL_CALIBRATION_N> lambda;

    Serial.println("Initial guess for T_a:");
    print_mtxf(T_a);
    Serial.println("Initial guess for b_a:");
    print_mtxf(b_a);
    theta << T_a, b_a, yb, lambda;

    int N_ITERS = 10;
    for (i=0;i<N_ITERS;i++)
    {
        theta = calculate_newton_iteration(theta);
    }

    T_a, b_a, yb, lambda << theta;

    Serial.println("Calculated T_a:");
    print_mtxf(T_a);

    Serial.println("Calculated b_a:");
    print_mtxf(b_a);
}

RowVector<float,12+4*ACCEL_CALIBRATION_N> Accelerometer::calculate_newton_iteration(Vector<float,12+4*ACCEL_CALIBRATION_N> theta)
{
    /********************************************************************************************************************
     * The goal here is to run the newton interation method described in https://doi.org/10.1155/2020/4617365.
     * To do this, the Jacobian and Hessian matrices must be formulated for the function INSERT NUM in the documentation
     * 
     * When using Eigen, Matrix<float,3,1> produces Vector3f and Matrix<float,1,3> produces Rowector3f
    *********************************************************************************************************************/
    const int N =  ACCEL_CALIBRATION_N;

    // Init theta-based variables
    Matrix3f T_a;
    T_a << theta.segment(0,9);
    Vector3f b_a;
    b_a << theta.segment(9,3);
    Matrix<float,3,N> ym;
    ym << theta.segment(12,N);
    Vector<float,N> lambda;
    lambda << theta.segment(12+N,N);

    Matrix<float,3,N> d;
    d << ym.colwise()  - b_a;
    
    // -----------------------------------------------------------------------------------------
    // This needs checking!
    Matrix<float,3,N> yb;
    yb << T_a.inverse() * (ym.colwise() - b_a);
    // -----------------------------------------------------------------------------------------


    // Declare iterator variables
    Vector3f ym_k;
    Vector3f yb_k;
    Vector3f d_k;
    float lambda_k;

    // Declare jacobian and hessian variables
    RowVector<float, 12+4*N> jacobian;
    Matrix<float, 12+4*N, 12+4*N> hessian;
    
    

    /*********************************************************************
     * Construct Jacobian which is a 1x(4*ACCEL_CALIBRATION_N+12) matrix
    **********************************************************************/
    // df/dT_a 9x1
    int k;
    Vector<float,9> J_1;
    J_1.setZero();

    for (k=0; k<N;k++)
    {
        yb_k = yb.col(k);
        J_1 = J_1 + kronecker(yb_k,(d_k - T_a*yb_k));
    }
    J_1 = -2*J_1;

    // df/db_a 3x1
    Vector3f J_2 = 2 * (T_a*yb - d).rowwise().sum();

    // df/dyb_k 3kx1
    Matrix<float,3,N> J_3_mat;
    for (k=0; k<N;k++)
    {
        yb_k = yb.col(k);
        J_3_mat.col(k) = -2 * T_a.transpose() * (d_k - T_a*yb_k) + 2*lambda_k*yb_k;
    }
    // Convert to Vector << col1(0), col1(1), col1(2), col2(0), col2(1), col2(2), ... , coln(0), coln(1), coln(2)
    RowVector<float,3*N> J_3 = J_3_mat.reshaped();

    // df/dlambda_k kx1
    RowVector<float,N> J_4 = (yb.colwise().norm().array().pow(2)) - 1;

    // Jacobian
    jacobian << J_1.transpose(), J_2.transpose(), J_3, J_4;


    /*********************************************************************************************
     * Construct Hessian which is a (4*ACCEL_CALIBRATION_N+12)x(4*ACCEL_CALIBRATION_N+12) matrix
    **********************************************************************************************/
    Vector<float,9> tempV;

    // ---------------------------------------- H11 9x9  ----------------------------------------
    Matrix<float,9,9> H11;
    H11.setZero();

    for (k=0;k<N;k++)
    {
        //yb_k
        yb_k = yb.col(k);

        //yb_k * yb_k.T
        Matrix3f ykyk;
        ykyk = kronecker(yb_k, yb_k.transpose());

        // ykyk kronecker I
        H11 = H11 + kronecker(ykyk, Matrix3f::Identity(3,3));
    }
    H11 = 2*H11;


    //  ---------------------------------------- H12 9x3 ----------------------------------------
    Matrix<float,9,3> H12;
    H12.setZero();

    for (k=0;k<N;k++)
    {
        
        yb_k = yb.col(k); //yb_k
        H12 = H12 + kronecker(yb_k, Matrix3f::Identity(3,3)); // yb_k kronecker I
    }
    H12 = 2*N*H12;
    // ---------------------------------------- H21 3x9 ----------------------------------------
    Matrix<float,3,9> H21 = H12.transpose();


    // ---------------------------------------- H13 9x3N ----------------------------------------
    Matrix<float,9,3*N> H13;
    for (k=0;k<N;k++)
    {
        ym_k = ym.col(k);
        d_k = ym_k - b_a;
        yb_k = yb.col(k); //yb_k
        H13.block(k*3,0,3,3) = 2*kronecker(yb_k,Matrix3f::Identity(3,3))*T_a - kronecker(Matrix3f::Identity(3,3), (d_k-T_a*yb_k));
    }
    // ---------------------------------------- H31 3Nx9 ----------------------------------------
    Matrix<float,3*N,9> H31 = H13.transpose(); 


    // ---------------------------------------- H14 9xk ----------------------------------------
    Matrix<float,9,N> H14;
    H14.setZero();
    // ---------------------------------------- H41 kx9 ----------------------------------------
    Matrix<float,N,9> H41 = H14.transpose(); 


    // ---------------------------------------- H22 2x2 ----------------------------------------
    Matrix3f H22 = Matrix3f::Identity(3,3);
    H22 = H22*2*N;


    // ---------------------------------------- H23 3x3k ----------------------------------------
    Matrix<float,3,3*N> H23;
    for (k=0;k<N;k++)
    {
        H23.block(k*3,0,3,3) = 2*T_a;
    }
    // ---------------------------------------- H32 3kx3  ----------------------------------------
    Matrix<float,3*N,3> H32 = H23.transpose(); 


    // ---------------------------------------- H24 3xk ----------------------------------------
    Matrix<float,3,N> H24;
    H24.setZero();
    // ---------------------------------------- H42 kx3 ----------------------------------------
    Matrix<float,N,3> H42 = H24.transpose(); 


    // ---------------------------------------- H33 3kx3k ----------------------------------------
    // All non-diagonal elements of H33' are zero(3,3) where H33' is a NxN matrix of 3x3 matrices
    Matrix<float,3*N,3*N> H33;
    int row, col;
    H33.setZero();
    for (row=0;row<N;row++)
    {
        for (col=0;col<N;col++)
        {
            if(row==col)
            {
                lambda_k = lambda(k);
                H33.block(row*3,col*3,3,3) = 2*T_a.transpose() * T_a + 2*MatrixX3f::Identity(3,3)*lambda_k;
            }
        }
    }


    // ---------------------------------------- H34 3kxk ----------------------------------------
    // All non-diagonal elements of H32' are 0 where H32' is a NxN matrix of 3vectors
    Matrix<float, 3*N, N> H34;
    H32.setZero();
    for (row=0;row<N;row++)
    {
        for (col=0;col<N;col++)
        {
            if(row==col)
            {
                yb_k = yb.col(k);
                H34.block(row*3,col,3,1) = 2*yb_k;
            }
            
        }
    }
    // ---------------------------------------- H43 1x3k ----------------------------------------
    Matrix<float, N, 3*N> H43 = H34.transpose();


    // ---------------------------------------- H44 kxk ----------------------------------------
    Matrix<float,N,N> H44;
    H44.setZero();


    // ---------------------------------------- Hessian ----------------------------------------
    hessian.setZero();
    hessian.block(0,0,      9,9)    = H11;
    hessian.block(0,9,      9,3)    = H12;
    hessian.block(0,12,     9,3*N)  = H13;
    hessian.block(0,12+3*N, 9,N)    = H14;

    hessian.block(9,0,      9,9)    = H21;
    hessian.block(9,9,      9,3)    = H22;
    hessian.block(9,12,     9,3*N)  = H23;
    hessian.block(9,12+3*N, 9,N)    = H24;

    hessian.block(12,0,     9,9)    = H31;
    hessian.block(12,9,     9,3)    = H32;
    hessian.block(12,12,    3*N,3*N)= H33;
    hessian.block(12,12+3*N,3*N,N)  = H34;

    hessian.block(12+3*N,0, 9,9)    = H41;
    hessian.block(12+3*N,9, 9,3)    = H42;
    hessian.block(12+3*N,12,N,3*N)  = H43;
    
    hessian.block(12+3*N,12+3*N,N,N) = H44;


    // Multiply Jacobian by inverse of hessian

    return theta - (hessian.inverse() * jacobian.transpose());

}