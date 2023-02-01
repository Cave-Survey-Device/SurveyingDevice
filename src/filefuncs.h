#ifndef HEADER_filefuncs
#define HEADER_filefuncs

#include <preferences.h>
#include <nvs_flash.h>

#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigenSparse.h>

#include "config.h"
#include "utility.h"


static Preferences preferences;
void write_to_file(const char* fname, const char* name, const float data);
void write_to_file(const char* fname, const char* name, const int data);
void write_to_file(const char* fname, const char* name, const String data);
void write_to_file(const char* fname, const char* name, const node* n);
void erase_storage();

void read_from_file(const char* fname, const char* name, node* n);

#endif