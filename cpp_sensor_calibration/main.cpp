#include <iostream>
#include "mag_cal.h"
#include "data_generation.h"
#include "include/gnuplot-iostream.h"
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
    Tm <<  0.7, -0.8, 0.4,
            1.1, 0.3, -0.1,
            -0.3, 0.6, 0.7;

    Vector3f hm;
    hm << 0.5,1.7,2.6;

    Matrix3f Ta;
    Ta <<  0.7, -0.8, 0.4,
            1.1, 0.3, -0.1,
            -0.3, 0.6, 0.7;

    Vector3f ha;
    ha << 0.5,1.7,2.6;

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


    // Create initial guess for magnetometer calibration
    Matrix<float, 3, 3> M;
    Vector<float, 3> n;
    float d;

    Vector<float, 10> U = fit_ellipsoid(mag_samples);
    M << U[0], U[5], U[4], U[5], U[1], U[3], U[4], U[3], U[2];
    n << U[6], U[7], U[8];
    d = U[9];

    Vector<float, 12> out = calculate_transformation(M, n, d);

    // Calibration matrix (T^-1)
    Matrix<float, 3, 3> R;
    R << out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7], out[8];
    // Bias vector
    Vector<float, 3> b;
    b << out[9], out[10], out[11];

    MatrixXf mag_corrections = R * (mag_samples.colwise() - b);

    // MAG GNU PLOTTING
    Gnuplot gp;

    auto plots = gp.splotGroup();
    plots.add_plot1d_colmajor(mag_true_data, "with points title 'True Vector'");
    plots.add_plot1d_colmajor(mag_samples, "with points title 'Samples'");
    plots.add_plot1d_colmajor(mag_corrections, "with points title 'Corrected data'");
    gp << plots;


    // Run joint accelerometer and magnetometer calibration and alignment


#ifdef _WIN32
    // For Windows, prompt for a keystroke before the Gnuplot object goes out of scope so that
    // the gnuplot window doesn't get closed.
    std::cout << "Press enter to exit." << std::endl;
    std::cin.get();
#endif
    return 0;
}
