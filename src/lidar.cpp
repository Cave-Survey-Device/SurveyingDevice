#include "lidar.h"

void Lidar::generate_command(int type, char command_packet[6])
{
    char initiate = LIDAR_START_BYTE;
    char address = 0x01;
    char command = 0;
    char data = 0;
    char checksum;
    char end = LIDAR_END_BYTE;
    
    switch (type){
        case LIDAR_READ_SOFTWARE_VERSION:
            command = LIDAR_READ_SOFTWARE_VERSION;
            checksum = 0x01;
            break;
        case LIDAR_READ_DEVICE_TYPE:
            command = LIDAR_READ_DEVICE_TYPE;
            checksum = 0x03;
            break;
        case LIDAR_READ_SLAVE_ADDR:
            command = LIDAR_READ_SLAVE_ADDR;
            address = 0x00;
            checksum = 0x04;
            break;
        case LIDAR_SET_SLAVE_ADDR:
            command = LIDAR_SET_SLAVE_ADDR;
            address = 0x00;
            data = 0x01;
            checksum = 0x43;
            break;
        case LIDAR_READ_DEVICE_ERROR_CODE:
            command = LIDAR_READ_DEVICE_ERROR_CODE;
            checksum = 0x09;
            break;
        case LIDAR_LASER_ON:
            command = LIDAR_LASER_ON;
            checksum = 0x43;
            break;
        case LIDAR_LASER_OFF:
            command = LIDAR_LASER_OFF;
            checksum = 0x44;
            break;
        case LIDAR_SINGLE_MEAS:
            command = LIDAR_SINGLE_MEAS;
            checksum = 0x45;
            break;
        case LIDAR_CONT_MEAS:
            command = LIDAR_CONT_MEAS;
            checksum = 0x46;
            break;
        case LIDAR_STOP_CONT_MEAS:
            command = LIDAR_STOP_CONT_MEAS;
            checksum = 0x49;
            break;
    }
    
    command_packet[0] = initiate;
    command_packet[1] = address;
    command_packet[2] = command;

    if (data == 0)
    {
    command_packet[3] = data;
    command_packet[4] = checksum;
    command_packet[5] = end;
    }
    else
    {
    command_packet[3] = checksum;
    command_packet[4] = end;
    }

};

void Lidar::receive_response(char raw_message[], lidar_received_msg* msg)
{

    msg->address = raw_message[0];
    msg->command = raw_message[1];

    int i;
    int message_size;
    int data_size;
    int start;

    char checksum;
    int calculated_checksum;


    // Size of received message => address, command, data[MAX 12], checksum
    message_size = sizeof(*raw_message)/sizeof(char)-1;

    // Message size => address, command, checksum
    data_size = message_size - 3;

    // Start is 2 instead of 0 due to address, command
    start = 2;

    for (i=start;i<start+LIDAR_RECEIVE_DATA_MAX_SIZE;++i)
    {
        if (i<start+data_size)
        {
            msg->data[i-start] = raw_message[i];
        } else {
            msg->data[i-start] = 0;
        }
    }

    checksum = raw_message[-1];

    calculated_checksum = 0x00;
    calculated_checksum += (int)msg->address;
    calculated_checksum += (int)msg->command;
    
    for (i=0;i<data_size;++i)
    {
        calculated_checksum += (int)raw_message[i];
    }
    calculated_checksum = (char)calculated_checksum & 0x7F;

    if ((char)calculated_checksum != checksum)
    {
        throw ("Checksum Invaid!");
    }
}

Lidar::Lidar()
{   
    // Using UART1
    single_char_buffer = { NULL };
    *buffer = { NULL };
    SerialPort.begin(9600); 
};

void Lidar::disable()
{
    digitalWrite(GPIO_NUM_14,LOW);
}

void Lidar::enable()
{
    digitalWrite(GPIO_NUM_14,HIGH);
}

void Lidar::erase_buffer()
{
    int i;
    for (i=0; i<LIDAR_BUFFER_SIZE;++i)
    {
        buffer[i] = 0;
    }
}

void Lidar::read_msg_from_uart(char* buffer)
{
    while (single_char_buffer != LIDAR_START_BYTE)
    {
        SerialPort.read(&single_char_buffer,1);
    }
    
    // Reads bytes until terminator into buffer (not including terminator)
    SerialPort.readBytesUntil(LIDAR_END_BYTE,buffer,99);
}

double Lidar::get_measurement()
{
    char generated_command[LIDAR_SEND_COMMAND_SIZE];
    lidar_received_msg received_msg;
    enable();

    generate_command(LIDAR_LASER_ON,generated_command);
    SerialPort.write(generated_command);
    read_msg_from_uart(buffer);

    delay(500);
    

    generate_command(LIDAR_SINGLE_MEAS,generated_command);
    SerialPort.write(generated_command);
    read_msg_from_uart(buffer);

    try {
        receive_response(buffer,&received_msg);
        Serial.print("LIDAR SINGLE MEASURE: ");
        Serial1.println(received_msg.data);
        return 0;
    }
    catch(char* e ) {
        Serial.print("ERROR: ");
        Serial.println(e);
    }

    generate_command(LIDAR_LASER_OFF,generated_command);
    SerialPort.write(generated_command);
    read_msg_from_uart(buffer);
    disable();
    return 0;
}