#ifndef ESP_FILESYSTEM_FILEFUNCS_H
#define ESP_FILESYSTEM_FILEFUNCS_H

#include <preferences.h>
#include <nvs_flash.h>


namespace FileFuncs
{

static Preferences preferences;

bool locationExists(const char* fname, const char* vname);

void writeToFile(const char* fname, const char* vname, const float data);
bool readFromFile(const char* fname, const char* vname, float& data);

void writeToFile(const char* fname, const char* vname, const double data);
bool readFromFile(const char* fname, const char* vname, double& data);

void writeToFile(const char* fname, const char* vname, const int data);
bool readFromFile(const char* fname, const char* vname, int& data);

void writeToFile(const char* fname, const char* vname, const unsigned int data);
bool readFromFile(const char* fname, const char* vname, unsigned int& data);

void writeToFile(const char* fname, const char* vname, const String data);
bool readFromFile(const char* fname, const char* vname, String& data);

void writeToFile(const char* fname, const char* vname, const float* data, int size);
bool readFromFile(const char* fname, const char* vname, float* data, int size);

void writeToFile(const char* fname, const char* vname, const void* data, size_t size);
bool readFromFile(const char* fname, const char* vname, void* data, size_t size);

bool isKey(const char* fname, const char* key);

void erase_flash();
void getStatus();
}




#endif