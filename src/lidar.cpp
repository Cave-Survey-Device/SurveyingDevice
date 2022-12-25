#include "lidar.h"

void lidar::send_command(int type, byte data[100])
{
    struct lidar_msg packet;
    packet.initiate = 0xAA;
    packet.address = 0x01;
    packet.command = 0x7A;
    packet.end = 0xA8;

    switch (type){
        case LIDAR_READ_SOFTWARE_VERSION:
            packet.command = LIDAR_READ_SOFTWARE_VERSION;
            packet.checksum = 0x01;
            break;
        case LIDAR_READ_DEVICE_TYPE:
            packet.command = LIDAR_READ_DEVICE_TYPE;
            packet.checksum = 0x03;
            break;
        case LIDAR_READ_SLAVE_ADDR:
            packet.command = LIDAR_READ_SLAVE_ADDR;
            packet.address = 0x00;
            packet.checksum = 0x04;
            break;
        case LIDAR_SET_SLAVE_ADDR:
            packet.command = LIDAR_SET_SLAVE_ADDR;
            packet.address = 0x00;
            packet.data[100] = {0x01};
            packet.checksum = 0x43;
            break;
        case LIDAR_READ_DEVICE_ERROR_CODE:
            packet.command = LIDAR_READ_DEVICE_ERROR_CODE;
            packet.checksum = 0x09;
            break;
        case LIDAR_LASER_ON:
            packet.command = LIDAR_LASER_ON;
            packet.checksum = 0x43;
            break;
        case LIDAR_LASER_OFF:
            packet.command = LIDAR_LASER_OFF;
            packet.checksum = 0x44;
            break;
        case LIDAR_SINGLE_MEAS:
            packet.command = LIDAR_SINGLE_MEAS;
            packet.checksum = 0x45;
            break;
        case LIDAR_CONT_MEAS:
            packet.command = LIDAR_CONT_MEAS;
            packet.checksum = 0x46;
            break;
        case LIDAR_STOP_CONT_MEAS:
            packet.command = LIDAR_STOP_CONT_MEAS;
            packet.checksum = 0x49;
            break;
    }
};


byte* receive_command(byte data[])
{
    struct lidar_msg packet;
    packet.address = data[0];
    packet.command = data[1];

    int i;
    int data_size = sizeof(data)/sizeof(byte)-1;;
    int start = 2;
    int end = data_size-1;
    for (i=start;i<end;++i)
    {
        packet.data[i-start] = data[i];
    }

    packet.checksum = data[-1];

    byte calculated_checksum = 0x00;
    calculated_checksum += packet.address;
    calculated_checksum += packet.command;
    int i;
    for (i=0;i<data_size;++i)
    {
        calculated_checksum += data[i];
    }
    calculated_checksum = calculated_checksum & 0x7F;

    if (calculated_checksum != packet.checksum)
    {
        throw std::invalid_argument("Checksum Invaid!");
    }

    return data;
}


lidar::lidar()
{   
    // Using UART1
    SerialPort = HardwareSerial(1);
    SerialPort.begin(15200, SERIAL_8N1, 4, 2); 
};
