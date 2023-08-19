#ifndef HEADER_FILESYSTEM_CSD
#define HEADER_FILESYSTEM_CSD

#define FILESYSTEM_ESP32
// #define SEEED

#ifdef FILESYSTEM_ESP32
#include <preferences.h>
#include <nvs_flash.h>
#include <ArduinoEigenDense.h>
#include <utility_csd.h>

static Preferences preferences;
void write_to_file(const char* fname, const char* name, const float data);
void read_from_file(const char* fname, const char* name, float& data);

void write_to_file(const char* fname, const char* name, const double data);
void read_from_file(const char* fname, const char* name, double& data);

void write_to_file(const char* fname, const char* name, const int data);
void read_from_file(const char* fname, const char* name, int& data);

void write_to_file(const char* fname, const char* name, const String data);
void read_from_file(const char* fname, const char* name, String& data);

void write_to_file(const char* fname, const char* name, const float* data, int size);
void read_from_file(const char* fname, const char* name, float* data, int size);

void write_to_file(const char* fname, const char* name, const MatrixXf& mat);
// void write_to_file(const char* fname, const char* name, const VectorXf& vec);
void read_from_file(const char* fname, const char* name, Ref<MatrixXf> mat);
// void read_from_file(const char* fname, const char* name, Ref<VectorXf> vec);

void erase_flash();
#else 
#ifdef FILESYSTEM_SEEED


#endif
#endif


#endif