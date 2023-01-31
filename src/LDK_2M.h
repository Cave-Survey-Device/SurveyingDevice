#ifndef HEADER_LDK_2M
#define HEADER_LDK_2M

#include <Arduino.h>
#include <HardwareSerial.h>
#include "unified.h"
#include "lidar.h"

// Const vars for lidar

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
const static int LIDAR_ENABLE_BEEPER = 0xF0;
const static int LIDAR_SEND_COMMAND_SIZE = 6;
const static int LIDAR_RECEIVE_DATA_MAX_SIZE = 12;
const static char LIDAR_START_BYTE = 0xAA;
const static char LIDAR_END_BYTE = 0xA8;
const static int LIDAR_BUFFER_SIZE = 100;
const static int LIDAR_MEAS_LEN = 6;
static int RX_BUFFER_SIZE = 256;

// Current lidar laser status
extern bool laser_on;

// LIDAR message struct
struct lidar_received_msg {
    char address;
    char command;
    char data[LIDAR_RECEIVE_DATA_MAX_SIZE];
};

extern bool interrupt_uart_timeout;
static hw_timer_t* uart_read_timer = NULL;
void IRAM_ATTR ISR_UART_TIMEOUT();
void init_uart_read_timer();
void start_uart_read_timer();
void stop_uart_read_timer();

// Class to deal with all things lidar
class LDK_2M: public Lidar {
    public:
        // Default constructor
        LDK_2M();

        // Initialise lidar module
        void init();

        // Get lidar mesaurement
        double get_measurement();

        // Toggle laser
        void toggle_laser();

    private:
        // Enable lidar via GPIO pin
        void enable();

        // Disable lidar via gpio pin
        void disable();

        // Holds a single character - used for reading single char from UART buffer until start bit received
        char single_char_buffer;

        // Larger char buffer to hold message received on UART buffer
        char buffer[LIDAR_BUFFER_SIZE];

        // Length of message received from uart buffer
        int msg_len;

        // Erase message buffer (variable not actual UART buffer)
        void erase_buffer();

        // Reads a message from the UART into buffer
        bool read_msg_from_uart(char* buffer);

        // Converts a string containing the distance to a double
        double to_distance(char* data);

        // Flush rx
        void flush_serial1();

        // Generate lidar command 
        void generate_command(int type, char command_arr[LIDAR_SEND_COMMAND_SIZE]);

        // Pack received lidar message into lidar msg struct
        void receive_response(char raw_data[], lidar_received_msg* receivec_msg);

};

#endif