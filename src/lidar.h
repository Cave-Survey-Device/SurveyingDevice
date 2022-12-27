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
const static int LIDAR_DISABLE_BEEPER = 0x47;
const static int LIDAR_SEND_COMMAND_SIZE = 6;
const static int LIDAR_RECEIVE_DATA_MAX_SIZE = 12;
const static char LIDAR_START_BYTE = 0xAA;
const static char LIDAR_END_BYTE = 0xA8;
const static int LIDAR_BUFFER_SIZE = 100;
// using namespace uart_types;

struct lidar_received_msg {
    char address;
    char command;
    char data[LIDAR_RECEIVE_DATA_MAX_SIZE];
};

class Lidar {
    public:
        Lidar();
        void init();
        void generate_command(int type, char command_arr[LIDAR_SEND_COMMAND_SIZE]);
        void receive_response(char raw_data[], lidar_received_msg* receivec_msg);
        double get_measurement();

    private:
        void enable();
        void disable();
        char single_char_buffer;
        char buffer[LIDAR_BUFFER_SIZE];
        int msg_len;
        void erase_buffer();
        void read_msg_from_uart(char* buffer);
};

#endif