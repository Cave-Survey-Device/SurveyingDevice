#include "filefuncs.h"

void write_to_file(const char* fname, const char* name, const float data){
  preferences.begin(fname, false);
  preferences.putFloat(name,data);
  preferences.end();
}

void write_to_file(const char* fname, const char* name, const double data){
  preferences.begin(fname, false);
  preferences.putDouble(name,data);
  preferences.end();
}
void write_to_file(const char* fname, const char* name, const int data){
  preferences.begin(fname, false);
  preferences.putInt(name,data);
  preferences.end();
}
void write_to_file(const char* fname, const char* name, const String data){
  preferences.begin(fname, false);
  preferences.putString(name,data);
  preferences.end();
}

void write_to_file(const char* fname,  const char* name, const node* n){
  preferences.begin(fname, false);
  preferences.putBytes(name,n,sizeof(node));
  preferences.end();
}


void erase_storage(){
  nvs_flash_erase();
  nvs_flash_init();
}