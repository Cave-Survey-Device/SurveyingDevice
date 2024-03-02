#ifndef ESP_FILESYSTEM_EIGEN_FILEFUNCS_H
#define ESP_FILESYSTEM_EIGEN_FILEFUNCS_H

#include <preferences.h>
#include <nvs_flash.h>
#include <ArduinoEigen.h>
#include "FileFuncs.h"
using namespace Eigen;

namespace EigenFileFuncs
{
using namespace FileFuncs;

inline void writeToFile(const char* fname, const char* name, const Ref<const MatrixXf> &mat)
{
  preferences.begin(fname, false);
  preferences.putBytes(name,mat.data(),mat.size()*sizeof(float));
  preferences.end();
}

inline void readFromFile(const char* fname, const char* name, Ref<MatrixXf> mat)
{
  preferences.begin(fname, false);
  preferences.getBytes(name,mat.data(),mat.size()*sizeof(float));
  preferences.end();
}


}

#endif