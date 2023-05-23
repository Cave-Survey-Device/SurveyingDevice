//
// Created by chris on 10/05/2023.
//

#ifndef MAGNETOMETER_CALIBRATION_ALIGNMENT2_H
#define MAGNETOMETER_CALIBRATION_ALIGNMENT2_H

#include "Eigen/Dense"
#include <random>
#include <stdexcept>
#include "util.h"

using namespace Eigen;

Vector<float, 9> dJ_dvecR (const MatrixXf &g, const MatrixXf &m, const Vector<float, 10> &X)
{
    int K = m.cols();
    Vector<float, 9> dJ_dvecR;
    Matrix3f R =  X.segment(0,9).reshaped(3,3);
    float s = X(9);

    dJ_dvecR.setZero();
    Vector3f mk;
    Vector3f gk;

    for(int k=0;k<K;k++)
    {
        mk = m.col(k);
        gk = g.col(k);
        dJ_dvecR += -2*(   (s - gk.transpose()*R*mk) * (kron(mk,gk)) );
    }
    dJ_dvecR += 4 * (R.determinant()*R.transpose()*R-R).reshaped(9,1);

    return dJ_dvecR;
}

float dJ_ds (const MatrixXf &g, const MatrixXf &m, const Vector<float, 10> &X)
{
    int K = m.cols();
    float dJ_ds;
    Matrix3f R = X.segment(0,9).reshaped(3,3);
    float s = X(9);

    dJ_ds = 0;
    Vector3f mk;
    Vector3f gk;

    for(int k=0;k<K;k++)
    {
        mk = m.col(k);
        gk = g.col(k);
        dJ_ds += 2 * (s - gk.transpose()*R*mk);
    }

    return dJ_ds;
}

float J2(const MatrixXf &g, const MatrixXf &m, const Vector<float, 10> &X)
{
    int K = m.cols();
    float j2;
    Matrix3f R = X.segment(0,9).reshaped(3,3);
    float s = X(9);

    j2 = pow((R*R.transpose() - Matrix3f::Identity()).norm(),2);
    Vector3f mk;
    Vector3f gk;

    for(int k=0;k<K;k++)
    {
        mk = m.col(k);
        gk = g.col(k);
        j2 += 2 * pow((s - gk.transpose()*R*mk),2);
    }
    return j2;
}


Vector<float,10> GradJ2(const MatrixXf &g, const MatrixXf &m, const Vector<float, 10> &X)
{
    Vector<float,10> grad_J2;
    grad_J2.segment(0,9) << dJ_dvecR(g,m,X);
    grad_J2(9) = dJ_ds(g,m,X);

    return grad_J2;
}

Vector<float,10> Align2_gd(const MatrixXf &g, const MatrixXf &m, const Matrix3f &Rhat, const float shat)
{
    std::cout << "Beginning GD\n";
    Vector<float,10> X;
    Vector<float,10> dx;
    float alpha = 0.1; // Loss scaling parameter (how much smaller does next step have to be compared to current step?)
    float beta = 0.75; // Line search scaling parameter
    float t  = 1; // Step size scale
    float cost;
    int i = 0;

    std::cout << "Setting initial values\n";
    X.segment(0,9 ) << Rhat.reshaped(9,1);
    X(9) = shat;

    std::cout << "Finding initial cost\n";
    cost = J2(g,m,X);
    std::cout << "Cost: " << cost << "\n";

    std::cout << "Entering loop\n";
    while (cost > 0.05)
    {
        i++;
        // Calculate dJ
        dx << -GradJ2(g,m,X);

        // Find scale over which to operate gradient descent at a given step
        //t = 1;
        while (    J2(g,m, X + t*dx) > J2(g,m,X) + alpha*t*dx.transpose()*dx   )
        {
            t = t * beta;
        }

        // Move to next step
        X = X + dx * t;
        cost = J2(g, m, X);
        std::cout << "Cost: " << cost << "\n";

        // If timestep too small
        if(t < 0.00000001 || i > 300)
        {
            break;
        }
    }
    return X;
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

Vector<float,10> Align2(const MatrixXf &g, const MatrixXf &m) {
    int K = m.cols();
    Vector<float,10> out;
    RowVector<float,9> vecR;

    MatrixXf A(K,9);
    Vector3f mk;
    Vector3f gk;

    // Step 1
    std::cout << "Step 1\n";
    for (int i=0; i<K; i++)
    {
        mk = m.col(i);
        gk = g.col(i);
        A.row(i) << kron(mk,gk).transpose();
    }
    std::cout << "A: \n" << A << "\n";

    // Step 2 - solve lstsq
    std::cout << "Step 2\n";
    Matrix3f H = ((A.transpose()*A).inverse() * A.transpose() * MatrixXf::Ones(K,1)).reshaped(3,3);
    std::cout << "H: \n" << H << "\n";

    // Step 3
    std::cout << "Step 3\n";
    JacobiSVD<MatrixXf> svd(H, ComputeThinU | ComputeThinV);
    Matrix3f U = svd.matrixU();
    Matrix3f V = svd.matrixV();
    Matrix3f Sig = svd.singularValues().asDiagonal();
    std::cout << "Sig: \n" << Sig << "\n";

    // Step 4
    std::cout << "Step 4\n";
    Matrix3f Uhat = sign(H.determinant()) * U;
    Matrix3f Rhat = Uhat * V.transpose();
    std::cout << "Uhat: \n" << Uhat << "\n";
    std::cout << "Rhat: \n" << Rhat << "\n";

    // Step 5
    std::cout << "Step 5\n";
    float shat = 0;
    for (int i=0; i<K; i++)
    {
        gk = g.col(i);
        mk = m.col(i);
        shat += gk.transpose() * Rhat * mk;
    }
    shat = shat * 1/K;

    std::cout << "Form out\n";
    out.segment(0,9) << Rhat.reshaped(9,1);
    out(9) = shat;

    // std::cout << "Begin Align2_gd\n";
    // out = Align2_gd(g,m,Rhat,shat);

    return out;
}

#endif //MAGNETOMETER_CALIBRATION_ALIGNMENT2_H
