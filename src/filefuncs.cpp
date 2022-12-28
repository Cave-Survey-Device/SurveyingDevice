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
  Serial.printf("Saving at name: %s\n", name);
  preferences.putBytes(name,n,sizeof(node));
  preferences.end();
}

void read_from_file(const char* fname, const char* name, node* n)
{
  // Open file
  preferences.begin(fname, true);

  // Read data into node pointer location
  preferences.getBytes(name,n,sizeof(struct node));

  debug(DEBUG_FILE,"Read line: %s ID: %d Previous ID: %d Prev vec: %f %f %f",name,n->id,n->previous,n->vector_to_prev(0),n->vector_to_prev(1),n->vector_to_prev(2));
  
  // Close file
  preferences.end();
}

void erase_storage(){
  nvs_flash_erase();
  nvs_flash_init();
}