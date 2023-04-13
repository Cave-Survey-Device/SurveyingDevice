#include <iostream>
#include "fit_ellipse.h"
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

    Matrix3f Tm;
    Tm <<  0.7, -0.8, 0.4,
            1.1, 0.3, -0.1,
            -0.3, 0.6, 0.7;

    Vector3f hm;
    hm << 0.5,1.7,2.6;

    MatrixXf true_vec = generate_true_data();
    //std:: cout << "True vec: " << true_vec.rows() << "x" << true_vec.cols() << "\n" << true_vec << "\n\n";
    MatrixXf samples = generate_samples(true_vec, Tm, hm);

    writeToCSVfile("data.txt",samples.transpose());

    Matrix<float, 3, 3> M;
    Vector<float, 3> n;
    float d;

    Vector<float, 10> U = fit_ellipsoid(samples);
    M << U[0], U[5], U[4], U[5], U[1], U[3], U[4], U[3], U[2];
    n << U[6], U[7], U[8];
    d = U[9];

//    std::cout << "\n\n";
//    std::cout << "M: \n" << M << "\n\n";
//    std::cout << "n: \n" << n << "\n\n";
//    std::cout << "d: \n" << d << "\n\n";

    Vector<float, 12> out = calculate_transformation(M, n, d);

    // Calibration matrix (T^-1)
    Matrix<float, 3, 3> R;
    R << out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7], out[8];
    // Bias vector
    Vector<float, 3> b;
    b << out[9], out[10], out[11];

    MatrixXf corrections = R * (samples.colwise() - b);

    // GNU PLOTTING
    Gnuplot gp;

    auto plots = gp.splotGroup();
    plots.add_plot1d_colmajor(true_vec, "with points title 'True Vector'");
    plots.add_plot1d_colmajor(samples, "with points title 'Samples'");
    plots.add_plot1d_colmajor(corrections, "with points title 'Corrected data'");
    gp << plots;

#ifdef _WIN32
    // For Windows, prompt for a keystroke before the Gnuplot object goes out of scope so that
    // the gnuplot window doesn't get closed.
    std::cout << "Press enter to exit." << std::endl;
    std::cin.get();
#endif
    return 0;
}
