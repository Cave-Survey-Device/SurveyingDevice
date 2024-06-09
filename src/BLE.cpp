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
    char str[CMD_SIZE];
    std::string s = pCharacteristic->getValue();
    debugf(DEBUG_BLE,"Received data: %s\n",s);

    strlcpy(str,s.c_str(),CMD_SIZE);
    Serial.println(str);

    pBLEData->write_command(str);
    pBLEData->read_command(str);

   debugf(DEBUG_BLE,"Saved data: %s\n", str);
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

    // --------------------------------------------------------------------------------------------------------------------------------------------------------
    azimuth_characteristic = new BLECharacteristic(BLEUUID(AZIMUTH_CHARACTERISTIC_UUID),BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    azimuth_descriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    azimuth_characteristic->addDescriptor(azimuth_descriptor);

    bean_boi_service->addCharacteristic(azimuth_characteristic);
    azimuth_characteristic->setValue("azimuth");
    azimuth_descriptor->setValue("Azimuth data");

    // --------------------------------------------------------------------------------------------------------------------------------------------------------
    inclination_characteristic = new BLECharacteristic(BLEUUID(INCLINATION_CHARACTERISTIC_UUID),BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    inclination_descriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    inclination_characteristic->addDescriptor(inclination_descriptor);

    bean_boi_service->addCharacteristic(inclination_characteristic);
    inclination_characteristic->setValue("inclination");
    inclination_descriptor->setValue("Inclination data");

    // --------------------------------------------------------------------------------------------------------------------------------------------------------
    roll_characteristic = new BLECharacteristic(BLEUUID(ROLL_CHARACTERISTIC_UUID),BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    roll_descriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    roll_characteristic->addDescriptor(roll_descriptor);

    bean_boi_service->addCharacteristic(roll_characteristic);
    roll_characteristic->setValue("roll");
    roll_descriptor->setValue("Roll Data");

    // --------------------------------------------------------------------------------------------------------------------------------------------------------
    distance_characteristic = new BLECharacteristic(BLEUUID(DISTANCE_CHARACTERISTIC_UUID),BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    distance_descriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    distance_characteristic->addDescriptor(distance_descriptor);

    bean_boi_service->addCharacteristic(distance_characteristic);
    distance_characteristic->setValue("distance");
    distance_descriptor->setValue("Distance Data");

    
    // --------------------------------------------------------------------------------------------------------------------------------------------------------
    id_characteristic = new BLECharacteristic(BLEUUID(ID_CHARACTERISTIC_UUID),BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    id_descriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    id_characteristic->addDescriptor(id_descriptor);

    bean_boi_service->addCharacteristic(id_characteristic);
    id_characteristic->setValue("id");
    id_descriptor->setValue("ID Data");


    // --------------------------------------------------------------------------------------------------------------------------------------------------------
    command_charateristic = new BLECharacteristic(BLEUUID(COMMAND_CHARACTERISTIC_UUID),BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
    command_descriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    command_charateristic->addDescriptor(command_descriptor);

    bean_boi_service->addCharacteristic(command_charateristic);
    command_charateristic->setValue("none");
    command_descriptor->setValue("Command characteristic");
    command_charateristic->setCallbacks(new MyCommandCharacteristicCallbacks(&shared_bledata));

    // Begin bluetooth...
    bean_boi_service->start();
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BEAN_BOI_SERVICE);
    pServer->getAdvertising()->start();
    Serial.println("Waiting a client connection to notify...");
}

void BLEHandler::update()
{
    Node temp_node;
    shared_bledata.read_data(&temp_node);
    debugf(DEBUG_BLE,"Updating BLE. Data: h: %f i: %f d: %f id: %i \n",temp_node.heading, temp_node.inclination, temp_node.distance, temp_node.id);
    azimuth_characteristic->setValue(temp_node.heading);
    inclination_characteristic->setValue(temp_node.inclination);
    id_characteristic->setValue(temp_node.id);
    distance_characteristic->setValue(temp_node.distance);

    // Send notifications to client
    azimuth_characteristic->notify();
    inclination_characteristic->notify();
    roll_characteristic->notify();
    distance_characteristic->notify();

    // Send notification to client and wait for response
    id_characteristic->indicate();
    // Serial.println("Received response!");
}

void BLEData::read_data(Node* node_obj)
{
    std::lock_guard<std::mutex> guard(ble_data_mtx);
    *node_obj = data;
}
void BLEData::write_data(const Node* new_data)
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