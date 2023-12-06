#ifndef ESP_FILESYSTEM_EIGEN_FILEFUNCS_H
#define ESP_FILESYSTEM_EIGEN_FILEFUNCS_H

#include <preferences.h>
#include <nvs_flash.h>
#include <ArduinoEigen.h>
#include "FileFuncs.h"

namespace EigenFileFuncs
{
using namespace FileFuncs;

template<typename Derived>
void writeToFile(const char* fname, const char* name, const MatrixBase<Derived> &mat)
{
  preferences.begin(fname, false);
  preferences.putBytes(name,mat.data(),mat.size()*sizeof(Derived::Scalar));
  preferences.end();
}

template<typename Derived>
void read_from_file(const char* fname, const char* name, MatrixBase<Derived> &mat)
{
  preferences.begin(fname, false);
  preferences.getBytes(name,mat.data(),mat.size()*sizeof(Derived::Scalar));
  preferences.end();
}


}

#endif