#include "filefuncs.h"

void write_to_file(const char* fname, const char* name, const float data){
  preferences.begin(fname, false);
  char str_buf[60];
  sprintf(str_buf,"Saving at name: %s\n", name);
  debug(DEBUG_FILE,str_buf);
  preferences.putFloat(name,data);
  preferences.end();
}

void write_to_file(const char* fname, const char* name, const double data){
  preferences.begin(fname, false);
  char str_buf[60];
  sprintf(str_buf,"Saving at name: %s\n", name);
  debug(DEBUG_FILE,str_buf);
  preferences.putDouble(name,data);
  preferences.end();
}

void write_to_file(const char* fname, const char* name, const int data){
  preferences.begin(fname, false);
  char str_buf[60];
  sprintf(str_buf,"Saving at name: %s\n", name);
  debug(DEBUG_FILE,str_buf);
  preferences.putInt(name,data);
  preferences.end();
}

void write_to_file(const char* fname, const char* name, const String data){
  preferences.begin(fname, false);
  char str_buf[60];
  sprintf(str_buf,"Saving at name: %s\n", name);
  debug(DEBUG_FILE,str_buf);
  preferences.putString(name,data);
  preferences.end();
}

void write_to_file(const char* fname,  const char* name, const node* n){
  preferences.begin(fname, false);
  char str_buf[60];
  sprintf(str_buf,"Saving at name: %s ID: %d Heading: %f Inclination %f\n",name,n->id,n->heading,n->inclination);
  debug(DEBUG_FILE,str_buf);
  preferences.putBytes(name,n,sizeof(node));
  preferences.end();
}

void read_from_file(const char* fname, const char* name, node* n)
{
  // Open file
  debug(DEBUG_FILE,"Opening file");
  preferences.begin(fname, true);

  // Read data into node pointer location
  debug(DEBUG_FILE,"Reading bytes");
  preferences.getBytes(name,&n,sizeof(struct node));

  debug(DEBUG_FILE,"Constructing message to debug");
  char str_buf[60];
  sprintf(str_buf,"Read line: %s ID: %d Heading: %f Inclination %f",name,n->id,n->heading,n->inclination);
  debug(DEBUG_FILE,str_buf);
  
  // Close file
  debug(DEBUG_FILE,"Closing file");
  preferences.end();
  debug(DEBUG_FILE,"Closed file");
}

void erase_storage(){
  nvs_flash_erase();
  nvs_flash_init();
}