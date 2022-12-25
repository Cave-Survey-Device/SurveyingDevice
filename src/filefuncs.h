#ifndef HEADER_filefuncs
#define HEADER_filefuncs

#include <preferences.h>
#include <nvs_flash.h>
#include "config.h"

static Preferences preferences;

void write_to_file(const char* fname, const char* name, const float data);
void write_to_file(const char* fname, const char* name, const int data);
void write_to_file(const char* fname, const char* name, const String data);
void erase_storage();

#endif