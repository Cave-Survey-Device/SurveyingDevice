#include "accelerometer.h"


// PRINT MATRIX (float type)
// By: randomvibe
//-----------------------------
void print_mtxf(const Eigen::MatrixXf& X)  
{
   int i, j, nrow, ncol;
   nrow = X.rows();
   ncol = X.cols();
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
    int p = b.rows();
    int q = b.cols();

    MatrixXf out (p*m,q*n);
    
    for (row=0;row<m;row++)
    {
        for (col=0;col<n;col++)
        {
            // Block size pxq starting at (row*p, col*q)
            out.block(row*p,col*q,p,q) = a(row,col) * b;
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

    Vector3f b_a; // Zero-bias error
    b_a << 0.1,0.15,0.2;

    Matrix3f Cb_n; // Transformation from body frame to navigation frame 
    Cb_n << 1,0,0,
            0,1,0,
            0,0,1;

    Matrix3f C_s; // Scale factor deviation
    float S_x, S_y, S_z;
    S_x = 0;
    S_y = 0;
    S_z = 0;
    C_s <<  1+S_x, 0, 0,
            0, 1+S_y, 0,
            0, 0, 1+S_z;

    Matrix3f C_n; // Non-orthogonal Error
    float a,b,c;
    a = 0;
    b = 0;
    c = 0;
    C_n <<  1, sin(a), -sin(b),
            0, cos(a), sin(c)*cos(b),
            0, 0, cos(c)*cos(b);

    T_a << C_s*C_n;
    T_a << 1.15,0.0602,-0.401,0,1.0985,0.0575,0,0,1.0978;

    Vector3f epsilon = {0,0,0}; // Gaussian white noise


    ym.setZero();
    // for(x=0;x<xrots;x++)
    // {
    //     x_ang = x*M_PI/xrots;
    //     xrotation_mat <<    1, 0, 0,
    //                         0, cos(x_ang), -sin(x_ang),
    //                         0, sin(x_ang), cos(x_ang);
    //     for(y=0;y<yrots;y++)
    //     {
    //     y_ang = y*2*M_PI/yrots;
    //     yrotation_mat <<    cos(x_ang), 0, sin(x_ang),
    //                         0, 1, 0,
    //                         -sin(x_ang), 0, cos(x_ang);
    //         for(i=0;i<N_samples;i++)
    //         {
    //             noise << norm_dist(gen), norm_dist(gen), norm_dist(gen);
    //             ym.col(x*yrots*N_samples+y*N_samples+i)= T_a * xrotation_mat * yrotation_mat * g_vec + b_a + noise;
    //         }
    //     }
    // }


    Serial.println("T_a:");
    print_mtxf(T_a);

    Serial.println("b_a:");
    print_mtxf(b_a);

    Serial.println("Initial data:");
    print_mtxf(ym);

    // run_newton(ym);

    return true;
}

// void Accelerometer::run_newton(Matrix<float,3,ACCEL_CALIBRATION_N> ym)
// {
//     /**********************************************
//      * Newton scheme so requires an initial guess,
//      * guess calculated below...   
//     **********************************************/
//     // Initial estimate
//     Matrix3f R_a;
//     int i;
//     Vector3f ym_i;

//     // Construct Ax = b (equation has 10 parameters see Eq. 17)
//     Serial.println("Calculating least squares...");

//     MatrixXf A = MatrixXf::Ones(ACCEL_CALIBRATION_N,10); 
//     for (i=0;i<ACCEL_CALIBRATION_N;i++)
//     {
//         ym_i = ym.col(i);
//         A.row(i) <<
//         ym_i(0)*ym_i(0), ym_i(1)*ym_i(1), ym_i(2)*ym_i(2), // Squares
//         2*ym_i(0)*ym_i(1), 2*ym_i(0)*ym_i(2), 2*ym_i(1)*ym_i(2), // Combinations
//         2*ym_i(0), 2*ym_i(1), 2*ym_i(2), // Individuals
//         1; // Constant
//     }


//     Serial.println("Calculated A matrix:");
//     print_mtxf(A);

//     VectorXf b = VectorXf::Ones(ACCEL_CALIBRATION_N);

//     // VectorXf x = A.fullPivHouseholderQr().solve(b);
//     // Serial.println("Calculated x vector:");
//     // print_mtxf(x);
//     VectorXf x = A.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);
//     Serial.println("Calculated x vector:");
//     print_mtxf(x);

//     // Assign E, F, and G
//     Serial.println("Calculating parameters...");
//     Matrix3f E;
//     Vector3f F;
//     float G;

//     E << x(0), x(3), x(4),
//          x(3), x(1), x(5),
//          x(4), x(5), x(2);

//     F << x(6), x(7), x(8);

//     G = x(9)-1;

//     Serial.println("Calculated E matrix:");
//     print_mtxf(E);
//     // Compute R0_a and b0_a
//     Serial.println("Computing LLT...");
//     Matrix3f R0_a = E.llt().matrixL().transpose();
//     Serial.println("Computing inverse...");
//     Vector3f b0_a = -E.inverse() * F;

//     /**********************************************
//      * Run the newton iteration scheme...   
//     **********************************************/
//     Serial.println("Computing initial guess...");

//     Matrix3f T0_a = R0_a.inverse();

//     Serial.println("Initial guess for R_a:");
//     print_mtxf(R0_a);
//     Serial.println("Initial guess for T_a:");
//     print_mtxf(T0_a);
//     Serial.println("Initial guess for b_a:");
//     print_mtxf(b0_a);

//     Serial.println("ym data:");
//     print_mtxf(ym);

//     T0_a << 1.0673, 0.0668, -0.0305, 0, 1.1283, 0.0410, 0, 0, 1.1990;
//     b0_a << 0.0869, 0.1562, 0.1790;

//     theta.segment(0,9) = T0_a.reshaped();
//     theta.segment(9,3) = b0_a.reshaped();
//     theta.segment(12,3*ACCEL_CALIBRATION_N) = ym.reshaped();
//     theta.segment(12+3*ACCEL_CALIBRATION_N,ACCEL_CALIBRATION_N) = lambda.reshaped();

//     Serial.println("Beginning newton scheme...");
//     int N_ITERS = 10;
//     for (i=0;i<N_ITERS;i++)
//     {
//         Serial.printf("Current iteration: %i\n", i);
//         calculate_newton_iteration();
//     }

//     Serial.println("Finished iterating");
//     T_a = theta(seqN(0,9)).reshaped(3,3);
//     b_a = theta(seqN(9,3)).reshaped(3,1);
//     lambda =  theta(seqN(12+3*ACCEL_CALIBRATION_N,ACCEL_CALIBRATION_N)).reshaped(ACCEL_CALIBRATION_N,1);

//     Serial.println("Calculated T_a:");
//     print_mtxf(T_a);

//     Serial.println("Calculated b_a:");
//     print_mtxf(b_a);
// }

 void Accelerometer::calculate_newton_iteration()
{
    /********************************************************************************************************************
     * The goal here is to run the newton interation method described in https://doi.org/10.1155/2020/4617365.
     * To do this, the Jacobian and Hessian matrices must be formulated for the function INSERT NUM in the documentation
     * 
     * When using Eigen, Matrix<float,3,1> produces Vector3f and Matrix<float,1,3> produces Rowector3f
    *********************************************************************************************************************/
    const int N =  ACCEL_CALIBRATION_N;
        // Calibration variables
    Vector<float,9> J_1;
    Vector3f J_2;
    RowVector<float,3*ACCEL_CALIBRATION_N> J_3;
    RowVector<float,ACCEL_CALIBRATION_N> J_4;
    Matrix<float,9,9> H11;
    Matrix<float,9,3> H12;
    Matrix<float,3,9> H21;
    Matrix<float,9,3*ACCEL_CALIBRATION_N> H13;
    Matrix<float,3*ACCEL_CALIBRATION_N,9> H31;
    Matrix<float,9,ACCEL_CALIBRATION_N> H14;
    Matrix<float,ACCEL_CALIBRATION_N,9> H41;
    Matrix3f H22;
    Matrix<float,3,3*ACCEL_CALIBRATION_N> H23;
    Matrix<float,3*ACCEL_CALIBRATION_N,3> H32;
    Matrix<float,3,ACCEL_CALIBRATION_N> H24;
    Matrix<float,ACCEL_CALIBRATION_N,3> H42;
    Matrix<float,3*ACCEL_CALIBRATION_N,3*ACCEL_CALIBRATION_N> H33;
    Matrix<float,3*ACCEL_CALIBRATION_N, ACCEL_CALIBRATION_N> H34;
    Matrix<float,ACCEL_CALIBRATION_N, 3*ACCEL_CALIBRATION_N> H43;
    Matrix<float,ACCEL_CALIBRATION_N,ACCEL_CALIBRATION_N> H44;
    RowVector<float, 12+4*ACCEL_CALIBRATION_N> jacobian;
    Matrix<float, 12+4*ACCEL_CALIBRATION_N, 12+4*ACCEL_CALIBRATION_N> hessian;

    // Init theta-based variables
    T_a = theta(seqN(0,9)).reshaped(3,3);
    Serial.println("T_a:");
    print_mtxf(T_a);

    b_a = theta(seqN(9,3)).reshaped(3,1);
    Serial.println("b_a:");
    print_mtxf(b_a);

    ym =  theta(seqN(12,3*N)).reshaped(3,N);
    Serial.println("ym:");
    print_mtxf(ym);

    lambda =  theta(seqN(12+3*N,N)).reshaped(N,1);
    Serial.println("lambda:");
    print_mtxf(lambda);

    d = ym.colwise() - b_a;
    Serial.println("d:");
    print_mtxf(d);
    
    // -----------------------------------------------------------------------------------------
    // This needs checking!
    yb = T_a.inverse() * d;
    // -----------------------------------------------------------------------------------------



    /*********************************************************************
     * Construct Jacobian which is a 1x(4*ACCEL_CALIBRATION_N+12) matrix
    **********************************************************************/
    // df/dT_a 9x1
    int k;

    J_1.setZero();
    for (k=0; k<N;k++)
    {
        yb_k = yb.col(k);
        d_k = d.col(k);
        J_1 = J_1 + kronecker(yb_k,(d_k - T_a*yb_k));
    }
    J_1 = -2*J_1;

    // df/db_a 3x1
    J_2.setZero();
    for (k=0; k<N;k++)
    {
        yb_k = yb.col(k);
        d_k = d.col(k);
        J_2 = J_2 + d_k - T_a*yb_k;
    }

    // df/dyb_k 3kx1
    for (k=0; k<N;k++)
    {
        yb_k = yb.col(k);
        d_k = d.col(k);
        J_3_mat.col(k) = -2 * T_a.transpose() * (d_k - T_a*yb_k) + 2*lambda_k*yb_k;
    }
    // Convert to Vector << col1(0), col1(1), col1(2), col2(0), col2(1), col2(2), ... , coln(0), coln(1), coln(2)
    J_3 = J_3_mat.reshaped();


    // df/dlambda_k kx1
    for (k=0; k<N;k++)
    {
        yb_k = yb.col(k);
        J_4(k) = yb_k.norm() * yb_k.norm() - 1;
    }

    // Jacobian
    Serial.println("Generating jacobian");
    jacobian << J_1.transpose(), J_2.transpose(), J_3, J_4;


    /*********************************************************************************************
     * Construct Hessian which is a (4*ACCEL_CALIBRATION_N+12)x(4*ACCEL_CALIBRATION_N+12) matrix
    **********************************************************************************************/

    // ---------------------------------------- H11 9x9  ----------------------------------------
    H11.setZero();

    for (k=0;k<N;k++)
    {
        //yb_k
        yb_k = yb.col(k);
        // Serial.println("yb_k:");
        // print_mtxf(yb_k);

        //yb_k * yb_k.T
        ykyk = kronecker(yb_k, yb_k.transpose());
        // Serial.println("ykyk:");
        // print_mtxf(ykyk);


        // ykyk kronecker I
        H11 = H11 + kronecker(ykyk, Matrix3f::Identity(3,3));
    }
    H11 = 2*H11;
    // Serial.println("H11:");
    // print_mtxf(H11);


    //  ---------------------------------------- H12 9x3 ----------------------------------------
    H12.setZero();

    for (k=0;k<N;k++)
    {
        
        yb_k = yb.col(k); //yb_k
        H12 = H12 + kronecker(yb_k, Matrix3f::Identity(3,3)); // yb_k kronecker I
    }
    H12 = 2*H12;
    // ---------------------------------------- H21 3x9 ----------------------------------------
    H21 = H12.transpose();


    // ---------------------------------------- H13 9x3N ----------------------------------------
    for (k=0;k<N;k++)
    {
        ym_k = ym.col(k);
        d_k = d.col(k);
        yb_k = yb.col(k); //yb_k
        // k sets of 9x3
        H13.block(0,k*3,9,3) = 2*(kronecker(yb_k,Matrix3f::Identity(3,3))*T_a - kronecker(Matrix3f::Identity(3,3), (d_k-T_a*yb_k)));
    }
    // ---------------------------------------- H31 3Nx9 ----------------------------------------
    H31 = H13.transpose(); 


    // ---------------------------------------- H14 9xk ----------------------------------------
    H14.setZero();
    // ---------------------------------------- H41 kx9 ----------------------------------------
    H41 = H14.transpose(); 


    // ---------------------------------------- H22 2x2 ----------------------------------------
    H22 = Matrix3f::Identity(3,3);
    H22 = 2*N*H22;


    // ---------------------------------------- H23 3x3k ----------------------------------------
    for (k=0;k<N;k++)
    {
        H23.block(0,k*3,3,3) = 2*T_a;
    }
    // ---------------------------------------- H32 3kx3  ----------------------------------------
    H32 = H23.transpose(); 


    // ---------------------------------------- H24 3xk ----------------------------------------
    H24.setZero();
    // ---------------------------------------- H42 kx3 ----------------------------------------
    H42 = H24.transpose(); 


    // ---------------------------------------- H33 3kx3k ----------------------------------------
    // All non-diagonal elements of H33' are zero(3,3) where H33' is a NxN matrix of 3x3 matrices

    int row, col;
    H33.setZero();
    for (row=0;row<N;row++)
    {
        for (col=0;col<N;col++)
        {
            if(row==col)
            {
                lambda_k = lambda(row);
                H33.block(row*3,col*3,3,3) = 2*T_a.transpose() * T_a + 2*lambda_k*MatrixX3f::Identity(3,3);
            }
        }
    }


    // ---------------------------------------- H34 3kxk ----------------------------------------
    // All non-diagonal elements of H32' are 0 where H32' is a NxN matrix of 3vectors
    H32.setZero();
    for (row=0;row<N;row++)
    {
        for (col=0;col<N;col++)
        {
            if(row==col)
            {
                yb_k = yb.col(col);
                H34.block(row*3,col,3,1) = 2*yb_k;
            }
            
        }
    }
    // ---------------------------------------- H43 1x3k ----------------------------------------
    H43 = H34.transpose();


    // ---------------------------------------- H44 kxk ----------------------------------------
    H44.setZero();


    // ---------------------------------------- Hessian ----------------------------------------
    Serial.println("Generating hessian");
    hessian.setZero();
    hessian.block(0,0,      9,9)    = H11;
    hessian.block(0,9,      9,3)    = H12;
    hessian.block(0,12,     9,3*N)  = H13;
    hessian.block(0,12+3*N, 9,N)    = H14;

    hessian.block(9,0,      3,9)    = H21;
    hessian.block(9,9,      3,3)    = H22;
    hessian.block(9,12,     3,3*N)  = H23;
    hessian.block(9,12+3*N, 3,N)    = H24;

    hessian.block(12,0,     3*N,9)  = H31;
    hessian.block(12,9,     3*N,3)  = H32;
    hessian.block(12,12,    3*N,3*N)= H33;
    hessian.block(12,12+3*N,3*N,N)  = H34;

    hessian.block(12+3*N,0, N,9)    = H41;
    hessian.block(12+3*N,9, N,3)    = H42;
    hessian.block(12+3*N,12,N,3*N)  = H43;
    
    hessian.block(12+3*N,12+3*N,N,N) = H44;


    // Multiply Jacobian by inverse of hessian
    Serial.println("Generating new theta");
    Serial.println("    Inverting hessian...");
    fplu = hessian.fullPivLu();
    Serial.print("hessian invertible?");
    if (fplu.isInvertible())
    {
        Serial.print("  YES\n");
        hessian = fplu.inverse();
    } else {
        Serial.print("  NO\n");
        print_mtxf(jacobian);
        print_mtxf(hessian);
    }

    
    Serial.println("    Transposing jacobian and multiplying...");
    theta2 = hessian * jacobian.transpose();
    Serial.println("Correcting theta");
    theta = theta - theta2;

}