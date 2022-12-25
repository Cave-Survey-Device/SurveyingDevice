#ifndef HEADER_LIDAR
#define HEADER_LIDAR

#include <Arduino.h>
#include <HardwareSerial.h>

const static int LIDAR_READ_SOFTWARE_VERSION = 0x01;
const static int LIDAR_READ_DEVICE_TYPE = 0x02;
const static int LIDAR_READ_SLAVE_ADDR = 0x04;
const static int LIDAR_SET_SLAVE_ADDR = 0x41;
const static int LIDAR_READ_DEVICE_ERROR_CODE = 0x08;
const static int LIDAR_LASER_ON = 0x42;
const static int LIDAR_LASER_OFF = 0x43;
const static int LIDAR_SINGLE_MEAS = 0x44;
const static int LIDAR_CONT_MEAS = 0x45;
const static int LIDAR_STOP_CONT_MEAS = 0x46;



const static int lidar_READ_SOFTWARE_VERSION = 0x01;

// using namespace uart_types;

struct lidar_msg {
    byte initiate;
    byte address;
    byte command;
    byte data[100];
    byte checksum;
    byte end;
};

class lidar{
    public:
        lidar();
        void send_command(int type, byte data[100]);

    private:
        HardwareSerial SerialPort;
};

#endif