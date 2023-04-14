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
#include "mag_cal.h"
#include "data_generation.h"

using namespace Eigen;

struct StructUnknowns
{
    Matrix3f Ha;
    Vector3f va;
    Matrix<float,3,N_SAMPLES> f;    // Actual accelerometer measurements
    Matrix3f Hm;
    Vector3f vm;
    Matrix<float,3,N_SAMPLES> m;    // Actual magnetometer measurements
    Matrix3f R;
    float d;

    inline StructUnknowns operator+(StructUnknowns a) {
        StructUnknowns out;
        out.Ha = Ha + a.Ha;
        out.va = va + a.va;
        out.f = f + a.f;
        out.Hm = Hm + a.Hm;
        out.vm = vm + a.vm;
        out.m = m + a.m;
        out.R = R + a.R;
        out.d = d + a.d;
        return out;
    }

    inline StructUnknowns operator*(float val) {
        StructUnknowns out;
        out.Ha = Ha * val;
        out.va = va * val;
        out.f = f * val;
        out.Hm = Hm * val;
        out.vm = vm * val;
        out.m = m * val;
        out.R = R * val;
        out.d = d * val;
        return out;
    }

    inline StructUnknowns operator-(StructUnknowns a) {
        StructUnknowns out;
        out.Ha = Ha - a.Ha;
        out.va = va - a.va;
        out.f = f - a.f;
        out.Hm = Hm - a.Hm;
        out.vm = vm - a.vm;
        out.m = m - a.m;
        out.R = R - a.R;
        out.d = d - a.d;
        return out;
    }
};

struct StructSamples
{
    Matrix<float,3,N_SAMPLES> ya;   // Raw accelerometer measurements
    Matrix<float,3,N_SAMPLES> ym;   // Raw magnetometer measurements
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
        for(int j=0; j<n; j++) {
            out.block(i*p,j*q,p,q) = m1(i,j) * m2;
        }
    }
    return out;
}

// Cost function, J(x)
float J(StructUnknowns X, StructSamples data)
{
    float sum1 = 0;
    float sum2 = 0;
    float sum3 = 0;
    Vector3f fk;
    Vector3f mk;
    Vector3f yak;
    Vector3f ymk;

    float probe;

    for(int k=0;k<N_SAMPLES;k++)
    {
        fk = X.f.col(k);
        yak = data.ya.col(k);
        mk = X.m.col(k);
        ymk = data.ym.col(k);

        probe = (fk-X.Ha * yak + X.va).norm();

        sum1 += pow((fk-X.Ha * yak + X.va).norm(),2) + pow(pow(fk.norm(),2) - 1,2);
        sum2 += pow((mk-X.Hm * ymk + X.vm).norm(),2) + pow(pow(mk.norm(),2) - 1,2);
        sum3 += pow(X.d-fk.transpose()*X.R*mk,2) + pow((X.R*X.R.transpose() - Matrix3f::Identity()).norm(),2) + pow(X.R.determinant()-1,2);
    }
    return sum1 + sum2 + sum3;
}

Matrix3f dJ_Ha(StructUnknowns X, StructSamples data)
{
    Vector<float,9> dJ_Ha;
    dJ_Ha.setZero();
    Vector3f yak;
    Vector3f fk;

    for(int k=0; k<N_SAMPLES; k++)
    {
        yak = data.ya.col(k);
        fk = X.f.col(k);
        dJ_Ha += 2 * kron(yak,X.Ha*yak-fk-X.va);
    }
    return dJ_Ha.reshaped(3,3);
}

Vector3f dJ_va(StructUnknowns X, StructSamples data)
{
    Vector3f dJ_va;
    dJ_va.setZero();
    Vector3f yak;
    Vector3f fk;

    for(int k=0; k<N_SAMPLES; k++)
    {
        yak = data.ya.col(k);
        fk = X.f.col(k);
        dJ_va += 2 * (-X.Ha * yak + X.va + fk);
    }
    return dJ_va;
}

Matrix<float,3,N_SAMPLES> dJ_f(StructUnknowns X, StructSamples data)
{
    Matrix<float,3,N_SAMPLES> dJ_f;
    Vector3f yap;
    Vector3f fp;
    Vector3f mp;

    for(int p=0; p<N_SAMPLES;p++)
    {
        yap = data.ya.col(p);
        fp = X.f.col(p);
        mp = X.m.col(p);
        dJ_f.block(0,p,3,1) = -2 * (X.Ha * yap + fp - X.va)
                + 4*fp*pow(fp.norm(),2)
                - 2 * ((X.d - fp.transpose() * X.R * mp) * mp.transpose() * X.R.transpose()).transpose();
    }
    return dJ_f;
}

Matrix3f dJ_Hm(StructUnknowns X, StructSamples data)
{
    Vector<float,9> dJ_Hm;
    dJ_Hm.setZero();
    Vector3f ymk;
    Vector3f mk;

    for(int k=0; k<N_SAMPLES; k++)
    {
        ymk = data.ym.col(k);
        mk = X.m.col(k);
        dJ_Hm += 2 * kron(ymk,X.Hm*ymk-mk-X.vm);
    }
    return dJ_Hm.reshaped(3,3);
}

Vector3f dJ_vm(StructUnknowns X, StructSamples data)
{
    Vector3f dJ_vm;
    dJ_vm.setZero();
    Vector3f mak;
    Vector3f mk;

    for(int k=0; k<N_SAMPLES; k++)
    {
        mak = data.ym.col(k);
        mk = X.m.col(k);
        dJ_vm += 2 * (-X.Hm * mak + X.vm + mk);
    }
    return dJ_vm;
}

Matrix<float,3,N_SAMPLES> dJ_m(StructUnknowns X, StructSamples data)
{
    Matrix<float,3,N_SAMPLES> dJ_f;
    Vector3f ymp;
    Vector3f mp;
    Vector3f fp;


    for(int p=0; p<N_SAMPLES;p++)
    {
        ymp = data.ym.col(p);
        mp = X.m.col(p);
        fp = X.f.col(p);
        dJ_f.block(0,p,3,1) = -2 * (X.Hm * ymp + mp - X.vm)
                              + 4*mp*pow(mp.norm(),2)
                              - 2 * ((X.d - fp.transpose() * X.R * mp) * fp.transpose() * X.R.transpose()).transpose();
    }
    return dJ_f;
}

Matrix3f dJ_R(StructUnknowns X, StructSamples data)
{
    Matrix3f dJ_R;
    dJ_R.setZero();
    Vector3f mk;
    Vector3f fk;
    for(int k=0;k<N_SAMPLES;k++)
    {
        mk = X.m.col(k);
        fk = X.f.col(k);
        dJ_R += -2*((X.d-fk.transpose()*X.R*mk) * kron(mk,fk)).reshaped(3,3);
    }
    dJ_R += 4* (X.R*X.R.transpose()*X.R - X.R)
            + 2* (X.R.determinant()-1)*X.R.adjoint().transpose();

    return dJ_R;
}

float dJ_d(StructUnknowns X, StructSamples data)
{
    float dJ_d;
    dJ_d = 0;
    Vector3f mk;
    Vector3f fk;
    for(int k=0;k<N_SAMPLES;k++)
    {
        mk = X.m.col(k);
        fk = X.f.col(k);
        dJ_d += 2 * X.d - fk.transpose() * X.R * mk;
    }
    return dJ_d;
}

StructUnknowns jcaa(Matrix<float,3,N_SAMPLES> m_samples, Matrix<float,3,N_SAMPLES> a_samples)
{
    StructUnknowns X;
    StructUnknowns grad_J;
    StructSamples data;
    data.ya = a_samples;
    data.ym = m_samples;

    // Calculate magnetometer initial parameters
    Matrix<float, 3, 3> M;
    Vector<float, 3> n;
    float d;

    Vector<float, 10> U = fit_ellipsoid(m_samples);
    M << U[0], U[5], U[4], U[5], U[1], U[3], U[4], U[3], U[2];
    n << U[6], U[7], U[8];
    d = U[9];

    Vector<float, 12> out = calculate_transformation(M, n, d);

    // Calibration matrix (T^-1)
    Matrix<float, 3, 3> Hm;
    Hm << out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7], out[8];
    // Bias vector
    Vector<float, 3> b;
    b << out[9], out[10], out[11];

    // Set initial guess for magnetometer
    X.Hm = Hm;
    X.vm = Hm * b;

    // Set initial guess for accelerometer
    X.Ha = Matrix3f::Identity();
    X.va = Vector3f::Zero();

    // Set other initial guesses
    X.R = Matrix3f::Identity();
    X.d = sin(Deg2Rad(54));

    X.f = data.ya;
    X.m = Hm * (data.ym.colwise() - b);

    // Gradient descent
    float cost = J(X,data);
    float alpha = 0.75;
    float t  = 10;
    while (cost > 0.1*N_SAMPLES)
    {
        // Calculate dJ
        grad_J.Ha = dJ_Ha(X, data);
        grad_J.va = dJ_va(X, data);
        grad_J.f = dJ_f(X, data);
        grad_J.Hm = dJ_Hm(X, data);
        grad_J.vm = dJ_vm(X, data);
        grad_J.m = dJ_m(X, data);
        grad_J.R = dJ_R(X, data);
        grad_J.d = dJ_d(X, data);

        // Find scale over which to operate gradient descent at a given step
        t = 1;
        while (J(X, data) < J(X - grad_J*t, data))
        {
            t = t * alpha;
        }

        X = X - grad_J * t;
        cost = J(X,data);
        if(t < 0.0000001)
        {
            break;
        }
    }
    return X;

}

#endif //MAGNETOMETER_CALIBRATION_JCAA_H
