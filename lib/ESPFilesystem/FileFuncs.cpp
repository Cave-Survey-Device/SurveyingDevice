#include "FileFuncs.h"

namespace FileFuncs
{
void writeToFile(const char* fname, const char* name, const float data){
  preferences.begin(fname, false);
  preferences.putFloat(name,data);
  preferences.end();
}
void readFromFile(const char* fname, const char* name, float& data){
  preferences.begin(fname, false);
  preferences.getFloat(name,data);
  preferences.end();
}

void writeToFile(const char* fname, const char* name, const double data){
  preferences.begin(fname, false);
  preferences.putDouble(name,data);
  preferences.end();
}
void readFromFile(const char* fname, const char* name, double& data){
  preferences.begin(fname, false);
  preferences.getDouble(name,data);
  preferences.end();
}

void writeToFile(const char* fname, const char* name, const int data){
  preferences.begin(fname, false);
  preferences.putInt(name,data);
  preferences.end();
}
void readFromFile(const char* fname, const char* name, int& data){
  preferences.begin(fname, false);
  preferences.getInt(name,data);
  preferences.end();
}

void writeToFile(const char* fname, const char* name, const unsigned int data){
  preferences.begin(fname, false);
  preferences.putUInt(name,data);
  preferences.end();
}
void readFromFile(const char* fname, const char* name, unsigned int& data){
  preferences.begin(fname, false);
  preferences.getUInt(name,data);
  preferences.end();
}

void writeToFile(const char* fname, const char* name, const String data){
  preferences.begin(fname, false);
  preferences.putString(name,data);
  preferences.end();
}
void readFromFile(const char* fname, const char* name, String& data){
  preferences.begin(fname, false);
  preferences.getString(name,data);
  preferences.end();
}

void writeToFile(const char* fname, const char* name, const float* data, int size)
{
    preferences.begin(fname, false);
    preferences.putBytes(name,data,size*sizeof(float));
    preferences.end();
}
void readFromFile(const char* fname, const char* name, float* data, int size)
{
    preferences.begin(fname, true);
    preferences.getBytes(name,data,size*sizeof(float));
    preferences.end();
}

void writeToFile(const char* fname, const char* name, const void* data, size_t size)
{
    preferences.begin(fname, false);
    preferences.putBytes(name,data,size);
    preferences.end();
}
void readFromFile(const char* fname, const char* name, void* data, size_t size)
{
    preferences.begin(fname, true);
    preferences.getBytes(name,data,size);
    preferences.end();
}


void erase_flash()
{
  Serial.println("ERASING FLASH...");
  nvs_flash_erase(); // erase the NVS partition and...
  nvs_flash_init(); // initialize the NVS partition.
}

void getStatus()
{
  nvs_stats_t nvs_stats;
  nvs_get_stats(NULL, &nvs_stats);
  Serial.printf("Count: UsedEntries = (%lu), FreeEntries = (%lu), NamespaceCount = (%lu), AllEntries = (%lu)\n",
        nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.namespace_count, nvs_stats.total_entries);
}

}
