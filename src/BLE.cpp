#include "BLE.h"

BLEData shared_bledata;

void MyServerCallbacks::onConnect(BLEServer* pServer) {
    device_connected = true;
}
void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
    device_connected = false;
}

void BLEHandler::start()
{
    // Create BLE Server
    BLEDevice::init(bleServerName);
    pServer = BLEDevice::createServer();
    // pServer->setCallbacks(new MyServerCallbacks());

    // Create BLE Services
    bean_boi_service = pServer->createService(BEAN_BOI_SERVICE);

    // Create BLE Characteristic
    model_number_characteristic = new BLECharacteristic(BLEUUID((uint16_t)ESP_GATT_UUID_MODEL_NUMBER_STR), BLECharacteristic::PROPERTY_READ);
    bean_boi_service->addCharacteristic(model_number_characteristic);
    model_number_characteristic->setValue("Bean boi alpha");

    azimuth_characteristic = new BLECharacteristic(BLEUUID(AZIMUTH_CHARACTERISTIC_UUID),BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    heading_descriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    azimuth_characteristic->addDescriptor(heading_descriptor);

    bean_boi_service->addCharacteristic(azimuth_characteristic);
    azimuth_characteristic->setValue("azimuth");
    heading_descriptor->setValue("Azimuth data");


    inclination_characteristic = new BLECharacteristic(BLEUUID(INCLINATION_CHARACTERISTIC_UUID),BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    inclination_descriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    inclination_characteristic->addDescriptor(inclination_descriptor);

    bean_boi_service->addCharacteristic(inclination_characteristic);
    inclination_characteristic->setValue("inclination");
    inclination_descriptor->setValue("Inclination data");


    id_characteristic = new BLECharacteristic(BLEUUID(ID_CHARACTERISTIC_UUID),BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    id_descriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    id_characteristic->addDescriptor(id_descriptor);

    bean_boi_service->addCharacteristic(id_characteristic);
    id_characteristic->setValue("id");
    id_descriptor->setValue("ID Data");

    // begin bluetooth...
    bean_boi_service->start();
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BEAN_BOI_SERVICE);
    pServer->getAdvertising()->start();
    Serial.println("Waiting a client connection to notify...");
}

void BLEHandler::update()
{
    Serial.printf("Updating BLE. Data: %f %f %i \n",shared_bledata.data.heading,shared_bledata.data.inclination,shared_bledata.data.id);
    azimuth_characteristic->setValue(shared_bledata.data.heading);
    inclination_characteristic->setValue(shared_bledata.data.inclination);
    id_characteristic->setValue(shared_bledata.data.id);

    azimuth_characteristic->notify();
    inclination_characteristic->notify();

    id_characteristic->indicate();
    Serial.println("Received response!");
}

void BLEData::read(node* node_obj)
{
    lock();
    *node_obj = data;
    unlock();
}

void BLEData::write(const node* new_data)
{
    lock();
    data = *new_data;
    // memcpy(&data,new_data,sizeof(node));
    unlock();
}

void BLEData::lock()
{
    ble_data_mtx.lock();
}

void BLEData::unlock()
{
    ble_data_mtx.unlock();
}
