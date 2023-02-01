/*  BLE.h
*   Handles all BLE related control, including a ble data object
*   for sahred memory and a BLE handler object to keep the BLE running
*   Additionally, contains all BLE UUIDs and characteristics
*/

#ifndef HEADER_BLE
#define HEADER_BLE

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <mutex>
#include "utility.h"

#define bleServerName "bean_boi"
#define BEAN_BOI_SERVICE "8040889e-1df9-4279-ad09-7cc5ed83761a"

static bool device_connected = false;

static char DEVICE_INFORMATION_SERVICE_UUID[]           = "0000180A-0000-1000-8000-00805f9b34fb";
static char BATTERY_SERVICE_UUID[]                      = "0000180F-0000-1000-8000-00805f9b34fb";
static char MEASUREMENT_SYNC_SERVICE_UUID[]             = "000058d0-0000-1000-8000-00805f9b34fb";
static char MEASUREMENT_PRIMARY_CHARACTERISTIC_UUID[]   = "000058d1-0000-1000-8000-00805f9b34fb";
static char MEASUREMENT_METADATA_CHARACTERISTIC_UUID[]  = "000058d2-0000-1000-8000-00805f9b34fb";
static char MEASUREMENT_ERRORS_CHARACTERISTIC_UUID[]    = "000058d3-0000-1000-8000-00805f9b34fb";
static char LAST_TIME_CHARACTERISTIC_UUID[]             = "000058d3-0000-1000-8000-00805f9b34fb";
static char DEVICE_CONTROL_SERVICE_UUID[]               = "000058e0-0000-1000-8000-00805f9b34fb";
static char DEVICE_CONTROL_CHARACTERISTIC_UUID[]        = "000058e1-0000-1000-8000-00805f9b34fb";

static char AZIMUTH_CHARACTERISTIC_UUID[]               = "00e72748-8854-4107-857c-001140e0f1fc";
static char INCLINATION_CHARACTERISTIC_UUID[]           = "c501a8ac-5334-4e1d-8d45-5befdc98f923";
static char ID_CHARACTERISTIC_UUID[]                    = "c5a74ffa-c011-480c-83ac-b4601d0455a4";
static char COMMAND_CHARACTERISTIC_UUID[]               = "e1c99549-386d-4b25-be0f-fc113317d794";

static const int CMD_SIZE = 20;

// For some reason these can't be accessed if in the BLEData class?!
static std::mutex ble_data_mtx;
static std::mutex ble_command_mtx;
static char command[CMD_SIZE] = "No command";

class BLEData
{
  public:
    void write_data(const node* new_data);
    void read_data(node* node_obj);

    void write_command(const char* command);
    void read_command(char* command);

  private:
    // std::mutex ble_data_mtx;
    // std::mutex ble_command_mtx;
    node data;
    
};

class MyServerCallbacks:public BLEServerCallbacks {
  void onConnect(BLEServer* pServer);
  void onDisconnect(BLEServer* pServer);
};

class MyCommandCharacteristicCallbacks:public BLECharacteristicCallbacks {
  public:
    MyCommandCharacteristicCallbacks(BLEData* ble_data_ptr);
    
  private:
    void onWrite(BLECharacteristic* pCharacteristic);
    BLEData* pBLEData;
};

class BLEHandler
{
  public:
    void start();
    void update();
    BLEData shared_bledata;

  private:

    void message_handler();

    BLEServer *pServer;
    BLEService *bean_boi_service;
    BLECharacteristic *model_number_characteristic;
    

    BLECharacteristic *azimuth_characteristic;
    BLEDescriptor *heading_descriptor;

    BLECharacteristic *inclination_characteristic;
    BLEDescriptor *inclination_descriptor;

    BLECharacteristic *id_characteristic;
    BLEDescriptor *id_descriptor;

    BLECharacteristic *command_charateristic;
    BLEDescriptor *command_descriptor;

    BLEAdvertising *pAdvertising;
};

#endif