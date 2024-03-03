#ifndef NUMERICAL_METHODS_CONFIG_H
#define NUMERICAL_METHODS_CONFIG_H

const int N_ORIENTATIONS = 12; // Number of orientations used in static calibration
const int N_SAMPLES_PER_ORIENTATION = 20; // Number of samples to be saved per orientation
const int N_ALIGN_MAG_ACC = N_ORIENTATIONS*N_SAMPLES_PER_ORIENTATION; // Size of matrix required for static alignment data storeage
const int N_LASER_CAL = 8; // Number of orientations used in alser calibration
const float DEVICE_LENGTH = 0.2; // Length of device

#endif