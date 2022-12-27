#include "lidar.h"

void Lidar::generate_command(int type, char command_packet[LIDAR_SEND_COMMAND_SIZE])
{
    char address = 0x01;
    char command = 0;
    char data = 0xFF;
    char checksum;
    
    switch (type){
        case LIDAR_READ_SOFTWARE_VERSION:
            command = LIDAR_READ_SOFTWARE_VERSION;
            address = 0x00;
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
        case LIDAR_DISABLE_BEEPER:
            command = LIDAR_DISABLE_BEEPER;
            data = 0x00;
            checksum = 0x48;
    }
    
    command_packet[0] = LIDAR_START_BYTE;
    command_packet[1] = address;
    command_packet[2] = command;

    if ((int)data != (int)0xFF)
    {
        command_packet[3] = data;
        command_packet[4] = checksum;
        command_packet[5] = LIDAR_END_BYTE;
    }
    else
    {
        command_packet[3] = checksum;
        command_packet[4] = LIDAR_END_BYTE;
        command_packet[5] = 0x00;
    }
    Serial.printf("Generated command (in func): %X %X %X %X %X %X\n", command_packet[0],command_packet[1],command_packet[2],command_packet[3],command_packet[4],command_packet[5]);
};

void Lidar::receive_response(char raw_message[], lidar_received_msg* msg)
{
    int i;
    int data_size;
    int start;

    char checksum;
    unsigned int calculated_checksum;

    Serial.println("CONSTRUCTING RESPONSE - Assigning address and command...");
    msg->address = raw_message[0];
    msg->command = raw_message[1];

    Serial.println("CONSTRUCTING RESPONSE - Assigning data...");
    // Size of received message => address, command, data[MAX 12], checksum

    // Message size => address, command, checksum
    data_size = msg_len - 3;

    // Start is 2 instead of 0 due to address, command
    start = 2;

    
    Serial.printf("Address: %X/%c\n",raw_message[0],raw_message[0]);
    Serial.printf("Command: %X/%c\n",raw_message[1],raw_message[1]);
    Serial.print("Received data: ");
    for (i=start;i<start+LIDAR_RECEIVE_DATA_MAX_SIZE;++i)
    {
        Serial.printf("%X/%c ",raw_message[i],raw_message[i]);

        if (i<start+data_size)
        {
            msg->data[i-start] = raw_message[i];
        } else {
            msg->data[i-start] = 0;
        }
    }
    Serial.println("");
    

    Serial.println("CONSTRUCTING RESPONSE - Assigning checksum...");
    checksum = raw_message[start+data_size];

    calculated_checksum = 0x00;
    calculated_checksum += (unsigned int)msg->address;
    calculated_checksum += (unsigned int)msg->command;
    
    Serial.println("CONSTRUCTING RESPONSE - Comparing checksum...");
    for (i=0;i<data_size;++i)
    {
        calculated_checksum += (unsigned int)msg->data[i];
    }
    Serial.println("CONSTRUCTING RESPONSE - AND checksum with 0x7F");
    calculated_checksum = calculated_checksum & (unsigned int)0x7F;

    Serial.println("CONSTRUCTING RESPONSE - Comparing checksums");
    if ((unsigned int)calculated_checksum != (unsigned int)checksum)
    {
        Serial.printf("Checksum Invaid! %X != %X\n",calculated_checksum,(unsigned int)checksum);
    }
}

Lidar::Lidar()
{   
    // Using UART1
    Serial1.begin(9600);
    single_char_buffer = { 0x00 };
    *buffer = { 0x00 };
}

void Lidar::init()
{
    char generated_command[LIDAR_SEND_COMMAND_SIZE];
    lidar_received_msg received_msg;


    // Serial.println("LIDAR - Laser Off");
    // generate_command(LIDAR_LASER_OFF,generated_command);
    // Serial1.write(generated_command,LIDAR_SEND_COMMAND_SIZE);
    // read_msg_from_uart(buffer);
    // Serial.println(buffer);
    // erase_buffer();

    // delay(1000);

    // Serial.println("LIDAR - Laser On");
    // generate_command(LIDAR_LASER_ON,generated_command);
    // Serial1.write(generated_command,LIDAR_SEND_COMMAND_SIZE);
    // read_msg_from_uart(buffer);
    // Serial.println(buffer);
    // erase_buffer();

    Serial.println("LIDAR - get software version");
    generate_command(LIDAR_READ_SOFTWARE_VERSION,generated_command);
    Serial1.write(generated_command,LIDAR_SEND_COMMAND_SIZE);
    read_msg_from_uart(buffer);
    Serial.println(buffer);
    erase_buffer();

    Serial.println("LIDAR - disable beeper");
    generate_command(LIDAR_DISABLE_BEEPER,generated_command);
    Serial1.write(generated_command,LIDAR_SEND_COMMAND_SIZE);
    erase_buffer();

    // Serial.println("LIDAR - set slave address");
    // generate_command(LIDAR_SET_SLAVE_ADDR,generated_command);
    // Serial1.write(generated_command,LIDAR_SEND_COMMAND_SIZE);
    // read_msg_from_uart(buffer);
    // Serial.println(buffer);
    // erase_buffer();

    // Serial.println("LIDAR - get slave address");
    // generate_command(LIDAR_READ_SLAVE_ADDR,generated_command);
    // Serial1.write(generated_command,LIDAR_SEND_COMMAND_SIZE);
    // read_msg_from_uart(buffer);
    // Serial.println(buffer);
    // erase_buffer();


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
        buffer[i] = 0x00;
    }
}

void Lidar::read_msg_from_uart(char* buffer)
{
    char b[10];
    
    while ((int)single_char_buffer != (int)LIDAR_START_BYTE)
    {
        Serial1.read(&single_char_buffer,1);
        Serial.printf("Received byte: %c, %X\n", single_char_buffer, single_char_buffer);
    }
    //Serial1.readBytes(&single_char_buffer,1);
    erase_buffer();
    single_char_buffer = 0;
    // Reads bytes until terminator into buffer (not including terminator)
    msg_len = Serial1.readBytesUntil(LIDAR_END_BYTE,buffer,99);
}

double Lidar::to_distance(char* data)
{
    int i;
    double distance = 0;
    for (i=0; i<LIDAR_MEAS_LEN;i++)
    {
        distance += pow(10.0,-3-i) * (double)(int)data[i];
    }
    return distance;
}

double Lidar::get_measurement()
{
    double distance = 0.0;
    Serial.println("LIDAR - Initialising variables");
    char generated_command[LIDAR_SEND_COMMAND_SIZE];
    lidar_received_msg received_msg;
    enable();

    Serial.println("LIDAR - Turning laser on");
    generate_command(LIDAR_LASER_ON,generated_command);
    Serial1.write(generated_command,LIDAR_SEND_COMMAND_SIZE);
    read_msg_from_uart(buffer);
    erase_buffer();

    
    Serial.println("LIDAR - Requesting single measurement");
    generate_command(LIDAR_SINGLE_MEAS,generated_command);
    Serial1.write(generated_command);
    read_msg_from_uart(buffer);

    try {
        Serial.println("LIDAR - Constructing response...");
        receive_response(buffer,&received_msg);
        Serial.print("LIDAR SINGLE MEASURE: ");
        Serial.println(received_msg.data);
        erase_buffer();
        distance = to_distance(received_msg.data);
    }
    catch(char* e ) {
        Serial.print("ERROR: ");
        Serial.println(e);
        return 0;
    }

    Serial.println("LIDAR - Turning laser off");
    generate_command(LIDAR_LASER_OFF,generated_command);
    Serial1.write(generated_command);
    read_msg_from_uart(buffer);
    erase_buffer();

    disable();
    return distance;
}