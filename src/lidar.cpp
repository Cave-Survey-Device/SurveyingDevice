#include "lidar.h"

char* lidar_msg::serialise()
{
    char char_array[sizeof(lidar_msg)] = {initiate, address, command, *data, end};
    return char_array;
}


char* lidar::generate_command(int type, char data[100])
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
    return packet.serialise();
};


char* lidar::receive_response(char data[])
{
    struct lidar_msg packet;
    packet.address = data[0];
    packet.command = data[1];

    int i;
    int data_size = sizeof(*data)/sizeof(byte)-1;;
    int start = 2;
    int end = data_size-1;
    for (i=start;i<end;++i)
    {
        packet.data[i-start] = data[i];
    }

    packet.checksum = data[-1];

    int calculated_checksum = 0x00;
    calculated_checksum += (int)packet.address;
    calculated_checksum += (int)packet.command;
    
    for (i=0;i<data_size;++i)
    {
        calculated_checksum += (int)packet.data[i];
    }
    calculated_checksum = (char)calculated_checksum & 0x7F;

    if ((char)calculated_checksum != packet.checksum)
    {
        throw ("Checksum Invaid!");
    }

    return packet.data;
}


lidar::lidar()
{   
    // Using UART1
    single_char_buffer = { NULL };
    *buffer = { NULL };
    SerialPort.begin(15200, SERIAL_8N1, 4, 2); 
};

void lidar::disable()
{
    digitalWrite(GPIO_NUM_14,LOW);
}

void lidar::enable()
{
    digitalWrite(GPIO_NUM_14,HIGH);
}

double lidar::get_measurement()
{
    enable();
    SerialPort.write(generate_command(LIDAR_SINGLE_MEAS));
    while (single_char_buffer != 0xAA)
    {
        SerialPort.read(&single_char_buffer,1);
    }
    
    SerialPort.readBytesUntil(0xA8,buffer,99);

    try {
        char *data = receive_response(buffer);
        Serial.print("LIDAR SINGLE MEASURE: ");
        Serial1.println(data);
        return 0;
    }
    catch(char* e ) {
        Serial.print("ERROR: ");
        Serial.println(e);
    }
    disable();
    return 0;
}