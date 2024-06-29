#include "FileFuncs.h"

namespace FileFuncs
{
bool locationExists(const char* fname, const char* vname)
{
  if (preferences.begin(fname, true))
  {
    if (preferences.isKey(vname))
    {
        preferences.end();
        return true;
    }
    Serial.print("ERROR: Key does not exist in file\n");
  }
  Serial.print("ERROR: file does not exist\n");
  preferences.end();
  return false;
}

void writeToFile(const char* fname, const char* vname, const float data){
  preferences.begin(fname, false);
  preferences.putFloat(vname,data);
  preferences.end();
}
bool readFromFile(const char* fname, const char* vname, float& data){
  if (!locationExists(fname,vname)) { return false; }
  preferences.begin(fname, true);
  data = preferences.getFloat(vname);
  preferences.end();
  return true;
}

void writeToFile(const char* fname, const char* vname, const double data){
  preferences.begin(fname, false);
  preferences.putDouble(vname,data);
  preferences.end();
}
bool readFromFile(const char* fname, const char* vname, double& data){
  if (!locationExists(fname,vname)) { return false; }
  preferences.begin(fname, true);
  data = preferences.getDouble(vname);
  preferences.end();
  return true;
}

void writeToFile(const char* fname, const char* vname, const int data){
  preferences.begin(fname, false);
  preferences.putInt(vname,data);
  preferences.end();
}
bool readFromFile(const char* fname, const char* vname, int& data){
  if (!locationExists(fname,vname)) { return false; }
  preferences.begin(fname, true);
  data = preferences.getInt(vname);
  preferences.end();
  return true;
}

void writeToFile(const char* fname, const char* vname, const unsigned int data){
  preferences.begin(fname, false);
  preferences.putUInt(vname,data);
  preferences.end();
}
bool readFromFile(const char* fname, const char* vname, unsigned int& data){
  if (!locationExists(fname,vname)) {
    return false;
  }
  preferences.begin(fname, true);
  data = preferences.getUInt(vname);
  preferences.end();
  return true;
}

void writeToFile(const char* fname, const char* vname, const String data){
  preferences.begin(fname, false);
  preferences.putString(vname,data);
  preferences.end();
}
bool readFromFile(const char* fname, const char* vname, String& data){
  if (!locationExists(fname,vname)) { return false; }
  preferences.begin(fname, true);
  data = preferences.getString(vname);
  preferences.end();
  return true;
}

void writeToFile(const char* fname, const char* vname, const float* data, int size)
{
    preferences.begin(fname, false);
    preferences.putBytes(vname,data,size*sizeof(float));
    preferences.end();
}
bool readFromFile(const char* fname, const char* vname, float* data, int size)
{
  if (!locationExists(fname,vname)) { return false; }
  preferences.begin(fname, true);
  preferences.getBytes(vname,data,size*sizeof(float));
  preferences.end();
  return true;
}

void writeToFile(const char* fname, const char* vname, const void* data, size_t size)
{
    preferences.begin(fname, false);
    preferences.putBytes(vname,data,size);
    preferences.end();
}
bool readFromFile(const char* fname, const char* vname, void* data, size_t size)
{
  if (!locationExists(fname,vname)) { return false; }
  preferences.begin(fname, true);
  preferences.getBytes(vname,data,size);
  preferences.end();
  return true;
}

bool isKey(const char* fname, const char* key)
{
    preferences.begin(fname, true);
    return preferences.isKey(key);
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
