//
// Created by chris on 13/04/2023.
//

#ifndef MAGNETOMETER_CALIBRATION_JCAA_H
#define MAGNETOMETER_CALIBRATION_JCAA_H

#include "Eigen/Eigen"
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include <random>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace Eigen;

const int N_samples = 100;

struct input_vec
{
    Matrix<float,3,N_samples> ya;   // Raw accelerometer measurements
    Matrix<float,3,N_samples> ym;   // Raw magnetometer measurements
    Matrix3f Ha;
    Vector3f va;
    Matrix<float,3,N_samples> f;    // Actual accelerometer measurements
    Matrix3f Hm;
    Vector3f vm;
    Matrix<float,3,N_samples> m;    // Actual magnetometer measurements
    Matrix3f R;
    float d;
};

MatrixXf kron(MatrixXf m1, MatrixXf m2)
{
    int m = m1.rows();
    int n = m1.cols();
    int p = m2.rows();
    int q = m2.cols();

    MatrixXf out(m*p, n*q);
    for(int i=0; i<m; i++)
    {
        for(int j=0; i<n; i++) {
            out.block(i*p,i*q,p,q) = m1(i,j) * m2;
        }
    }
    return out;
}

Matrix3f dJ_Ha(input_vec X)
{
    Matrix3f dJ_Ha;
    dJ_Ha.setZero();
    Vector3f yak;
    Vector3f fk;

    for(int k=0; k<N_samples; k++)
    {
        yak = X.ya.col(k);
        fk = X.f.col(k);
        dJ_Ha += 2 * kron(yak,X.Ha*yak-fk-X.va);
    }
    return dJ_Ha;
}

Vector3f dJ_va(input_vec X)
{
    Vector3f dJ_va;
    dJ_va.setZero();
    Vector3f yak;
    Vector3f fk;

    for(int k=0; k<N_samples; k++)
    {
        yak = X.ya.col(k);
        fk = X.f.col(k);
        dJ_va += 2 * -X.Ha * yak + X.va + fk;
    }
    return dJ_va;
}

Matrix<float,3,N_samples> dJ_f(input_vec X)
{
    Matrix<float,3,N_samples> dJ_f;
    Vector3f yap;
    Vector3f fp;
    Vector3f mp;

    for(int p=0; p<N_samples;p++)
    {
        yap = X.ya.col(p);
        fp = X.f.col(p);
        mp = X.m.col(p);
        dJ_f.block(0,p,3,1) = -2 * (X.Ha * yap + fp - X.va)
                + 4*fp*pow(fp.norm(),2)
                - 2 * ((X.d - fp.transpose() * X.R * mp) * mp.transpose() * X.R.transpose()).transpose();
    }
    return dJ_f;
}

Matrix3f dJ_Hm(input_vec X)
{
    Matrix3f dJ_Hm;
    dJ_Hm.setZero();
    Vector3f ymk;
    Vector3f mk;

    for(int k=0; k<N_samples; k++)
    {
        ymk = X.ym.col(k);
        mk = X.m.col(k);
        dJ_Hm += 2 * kron(ymk,X.Hm*ymk-mk-X.vm);
    }
    return dJ_Hm;
}

Vector3f dJ_vm(input_vec X)
{
    Vector3f dJ_vm;
    dJ_vm.setZero();
    Vector3f mak;
    Vector3f mk;

    for(int k=0; k<N_samples; k++)
    {
        mak = X.ym.col(k);
        mk = X.m.col(k);
        dJ_vm += 2 * -X.Hm * mak + X.vm + mk;
    }
    return dJ_vm;
}

Matrix<float,3,N_samples> dJ_m(input_vec X)
{
    Matrix<float,3,N_samples> dJ_f;
    Vector3f ymp;
    Vector3f mp;
    Vector3f fp;


    for(int p=0; p<N_samples;p++)
    {
        ymp = X.ym.col(p);
        mp = X.m.col(p);
        fp = X.f.col(p);
        dJ_f.block(0,p,3,1) = -2 * (X.Hm * ymp + mp - X.vm)
                              + 4*mp*pow(mp.norm(),2)
                              - 2 * ((X.d - fp.transpose() * X.R * mp) * fp.transpose() * X.R.transpose()).transpose();
    }
    return dJ_f;
}

Matrix3f dJ_R(input_vec X)
{
}

float dJ_d(input_vec X)
{

}

#endif //MAGNETOMETER_CALIBRATION_JCAA_H
