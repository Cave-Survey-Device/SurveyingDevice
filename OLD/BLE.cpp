#include "BLE.h"

void MyServerCallbacks::onConnect(BLEServer* pServer) {
    device_connected = true;
}
void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
    device_connected = false;
}

MyCommandCharacteristicCallbacks::MyCommandCharacteristicCallbacks(BLEData* ble_data_ptr)
{
    pBLEData = pBLEData;
}

void MyCommandCharacteristicCallbacks::onWrite(BLECharacteristic *pCharacteristic)
{
    int i;
    char str[CMD_SIZE];
    char str2[CMD_SIZE];

    Serial.printf("Received data: ");
    std::string s = pCharacteristic->getValue();
    strlcpy(str,s.c_str(),CMD_SIZE);

    Serial.println(str);

    pBLEData->write_command(str);
    pBLEData->read_command(str2);

    Serial.print("Saved data: ");
    Serial.println(str2);
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


    command_charateristic = new BLECharacteristic(BLEUUID(COMMAND_CHARACTERISTIC_UUID),BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
    command_descriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    command_charateristic->addDescriptor(command_descriptor);

    bean_boi_service->addCharacteristic(command_charateristic);
    command_charateristic->setValue("none");
    command_descriptor->setValue("Command characteristic");
    command_charateristic->setCallbacks(new MyCommandCharacteristicCallbacks(&shared_bledata));

    // begin bluetooth...
    bean_boi_service->start();
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BEAN_BOI_SERVICE);
    pServer->getAdvertising()->start();
    Serial.println("Waiting a client connection to notify...");
}

void BLEHandler::update()
{
    node tempNode;
    shared_bledata.read_data(&tempNode);
    Serial.printf("Updating BLE. Data: h: %f i: %f d: %f id: %i \n",tempNode.heading, tempNode.inclination, tempNode.distance, tempNode.id);
    azimuth_characteristic->setValue(tempNode.heading);
    inclination_characteristic->setValue(tempNode.inclination);
    id_characteristic->setValue(tempNode.id);
    // set distance characteristic!

    azimuth_characteristic->notify();
    inclination_characteristic->notify();

    id_characteristic->indicate();
    // Serial.println("Received response!");
}

void BLEData::read_data(node* node_obj)
{
    std::lock_guard<std::mutex> guard(ble_data_mtx);
    *node_obj = data;
}
void BLEData::write_data(const node* new_data)
{
    std::lock_guard<std::mutex> guard(ble_data_mtx);
    data = *new_data;
    // memcpy(&data,new_data,sizeof(node));
}

void BLEData::write_command(const char* cmd_to_write)
{
    std::lock_guard<std::mutex> guard(ble_command_mtx);
    strlcpy(command,cmd_to_write,CMD_SIZE);
}
void BLEData::read_command(char* cmd_to_read)
{
    std::lock_guard<std::mutex> guard(ble_command_mtx);
    strlcpy(cmd_to_read,command,CMD_SIZE);
}