#ifndef HEADER_FILESYSTEM_CSD
#define HEADER_FILESYSTEM_CSD

#define ESP32
// #define SEEED

#ifdef ESP32
#include <preferences.h>
#include <nvs_flash.h>
#include <ArduinoEigenDense.h>

static Preferences preferences;
void write_to_file(const char* fname, const char* name, const float data);
void write_to_file(const char* fname, const char* name, const double data);
void write_to_file(const char* fname, const char* name, const int data);
void write_to_file(const char* fname, const char* name, const String data);
void read_from_file(const char* fname, const char* name, float* data);
void read_from_file(const char* fname, const char* name, int* data);
void erase_storage();

// template<typename Derived> 
// void write_to_file(const char* fname, const char* name, const Eigen::MatrixBase<Derived>& Data);
// template<typename Derived> 
// void write_to_file(const char* fname, const char* name, const Eigen::MapBase<Derived>& Data);
// template<typename Derived> 
// void read_from_file(const char* fname, const char* name, Eigen::MatrixBase<Derived>* Data);
// template<typename Derived> 
// void read_from_file(const char* fname, const char* name, Eigen::MapBase<Derived>* Data);

void write_to_file(const char* fname, const char* name, const float* data, int size);
void read_from_file(const char* fname, const char* name, float* data, int size);

#else 
#ifdef SEEED


#endif
#endif


#endif