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

// using namespace uart_types;

struct lidar_msg {
    char initiate;
    char address;
    char command;
    char data[100];
    char checksum;
    char end;
    char *serialised;

    void serialise();
    void reset();
};

class Lidar {
    public:
        Lidar();
        void generate_command(int type, char data[100] = {});
        void receive_response(char data[]);
        double get_measurement();

    private:
        char single_char_buffer;
        char buffer[100];
        HardwareSerial SerialPort = HardwareSerial(1);
        void enable();
        void disable();
        lidar_msg packet;
};

#endif