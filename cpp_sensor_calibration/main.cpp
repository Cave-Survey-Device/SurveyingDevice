#include <iostream>
#include "mag_cal.h"
#include "data_generation.h"
#include "include/gnuplot-iostream.h"
#include "jcaa.h"
#include <format>

using namespace Eigen;
#define BOOST_IOSTREAMS_NO_LIB

const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, "\t", "\n");

void writeToCSVfile(std::string name, MatrixXf matrix)
{
    std::ofstream file(name.c_str());
    file << matrix.format(CSVFormat);
}



int main() {

    // Define error parameters
    Matrix3f Tm;
    Tm <<  0.462,-0.0293,-0.037,
    0.0686,0.4379,0.0303,
    0.0427,-0.0336,0.4369;

    Vector3f hm;
    hm << -0.176,0.2214,0.0398;

    Matrix3f Ta;
    Ta <<  9.77,0.0018,-0.030,
    0.0019,9.7032,-0.0011,
    -0.0087, -0.0013,9.6927;
    Ta = Ta * 0.1;

    Vector3f ha;
    ha << -0.01472,-0.0011,-0.01274;

    // Generate sensor data
    Vector3f mag_vec = {1.,0.,0.};
    Vector3f accel_vec = {0.,0.,1.};

    MatrixXf mag_true_data = generate_true_data(mag_vec);
    MatrixXf true_accel_data = generate_true_data(accel_vec);

    MatrixXf mag_samples = generate_samples(mag_true_data, Tm, hm);
    MatrixXf accel_samples = generate_samples(true_accel_data, Ta, ha);

    // Save sensor data
    writeToCSVfile("mag_samples.txt", mag_samples.transpose());
    writeToCSVfile("accel_samples.txt", accel_samples.transpose());


    // Declare variables
    Matrix<float, 3, 3> M;
    Vector<float, 3> n;
    float d;
    Vector<float, 10> U;
    Matrix<float, 3, 3> R;
    Vector<float, 3> b;

    // Calculate magnetometer calibration
    U = fit_ellipsoid(mag_samples);
    M << U[0], U[5], U[4], U[5], U[1], U[3], U[4], U[3], U[2];
    n << U[6], U[7], U[8];
    d = U[9];
    Vector<float, 12> mag_transformation = calculate_transformation(M, n, d);

    R << mag_transformation[0], mag_transformation[1], mag_transformation[2], mag_transformation[3], mag_transformation[4], mag_transformation[5], mag_transformation[6], mag_transformation[7], mag_transformation[8];
    b << mag_transformation[9], mag_transformation[10], mag_transformation[11];
    MatrixXf mag_corrections = R * (mag_samples.colwise() - b);

    // Calculate accelerometer calibration
    U = fit_ellipsoid(mag_samples);
    M << U[0], U[5], U[4], U[5], U[1], U[3], U[4], U[3], U[2];
    n << U[6], U[7], U[8];
    d = U[9];
    Vector<float, 12> accel_transformation = calculate_transformation(M, n, d);

    R << mag_transformation[0], mag_transformation[1], mag_transformation[2], mag_transformation[3], mag_transformation[4], mag_transformation[5], mag_transformation[6], mag_transformation[7], mag_transformation[8];
    b << mag_transformation[9], mag_transformation[10], mag_transformation[11];
    MatrixXf accel_corrections = R * (mag_samples.colwise() - b);

    writeToCSVfile("accel_corrections.txt", accel_corrections.transpose());
    writeToCSVfile("mag_corrections.txt", mag_corrections.transpose());

//    // Run joint accelerometer and magnetometer calibration and alignment
//    StructUnknowns correction_parameters = jcaa(mag_samples, accel_samples);
//    MatrixXf mag_corrections = correction_parameters.R * correction_parameters.Hm * (mag_samples.colwise() - correction_parameters.Hm.inverse()*correction_parameters.vm);
//    MatrixXf accel_corrections = correction_parameters.Ha * (accel_samples.colwise() - correction_parameters.Ha.inverse()*correction_parameters.va);
//
//    writeToCSVfile("mag_partial_corrections.txt", mag_partial_corrections.transpose());
//    writeToCSVfile("accel_corrections.txt", accel_corrections.transpose());
//    writeToCSVfile("mag_corrections.txt", mag_corrections.transpose());
//
//
//    // MAG GNU PLOTTING
//    Gnuplot gp;
//    auto plots = gp.splotGroup();
//    plots.add_plot1d_colmajor(mag_true_data, "with points title 'True Vector'");
//    plots.add_plot1d_colmajor(mag_samples, "with points title 'Samples'");
//    plots.add_plot1d_colmajor(mag_corrections, "with points title 'Corrected data'");
//    gp << plots;
//
//#ifdef _WIN32
//    // For Windows, prompt for a keystroke before the Gnuplot object goes out of scope so that
//    // the gnuplot window doesn't get closed.
//    std::cout << "Press enter to exit." << std::endl;
//    std::cin.get();
//#endif
    return 0;
}
