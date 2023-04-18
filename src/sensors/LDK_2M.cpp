#include "LDK_2M.h"
#include "utils/config.h"

// Global laser status flag
bool laser_on = true;
bool uart_timeout = false;
unsigned long SERIAL1_TIMEOUT_MS = 1000;

// Utility functions
void LDK_2M::flush_serial1()
{
    while (Serial1.read() > 0) {delay(10);}
}

void LDK_2M::disable()
{
    digitalWrite(PIN_LASER_ENA,LOW);
}

void LDK_2M::enable()
{
    digitalWrite(PIN_LASER_ENA,HIGH);
}

void LDK_2M::erase_buffer()
{
    int i;
    for (i=0; i<LIDAR_BUFFER_SIZE;++i)
    {
        buffer[i] = 0x00;
    }
}

float LDK_2M::to_distance(char* data)
{
    float d;
    sscanf(data, "%lf", &d);
    d = d/1000.0;
    return d;
}

// Main functions
LDK_2M::LDK_2M()
{   
    // Using UART1
    Serial1.setTimeout(SERIAL1_TIMEOUT_MS);
    Serial1.begin(9600);
    single_char_buffer = { 0x00 };
    *buffer = { 0x00 };
    laser_on = false;
}

void LDK_2M::init()
{
    char generated_command[LIDAR_SEND_COMMAND_SIZE];
    lidar_received_msg received_msg;
    
    Serial1.flush();
    debug(DEBUG_LIDAR,"(Init 1/3) Get software version");
    generate_command(LIDAR_READ_SOFTWARE_VERSION,generated_command);
    Serial1.write(generated_command,LIDAR_SEND_COMMAND_SIZE);
    read_msg_from_uart(buffer);
    // Serial.println(buffer);
    debug(DEBUG_LIDAR,"(Init 2/3) Disable beeper");
    generate_command(LIDAR_DISABLE_BEEPER,generated_command);
    Serial1.write(generated_command,LIDAR_SEND_COMMAND_SIZE);

    erase_buffer();

    debug(DEBUG_LIDAR,"(Init 3/3) Finished INIT");
};

bool LDK_2M::read_msg_from_uart(char* buffer)
{
    debug(DEBUG_LIDAR,"(Read from UART 1/3) Starting timer");

    if ( Serial1.readBytesUntil(LIDAR_START_BYTE,&single_char_buffer,1) == 0)
    {
        debug(DEBUG_LIDAR,"(Read from UART 2/3) timer expired, read failed");
        return 0;
    }
    // Erase buffer
    erase_buffer();

    // Reads bytes until terminator into buffer (not including terminator)
    debug(DEBUG_LIDAR,"(Read from UART 2/3) Reading data");
    // TODO: check 99 length, it this necessary?
    msg_len = Serial1.readBytesUntil(LIDAR_END_BYTE,buffer,99);
    debug(DEBUG_LIDAR,"(Read from UART 3/3) Succesfully read data");
    return 1;
}

void LDK_2M::generate_command(int type, char command_packet[LIDAR_SEND_COMMAND_SIZE])
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
            break;
        case LIDAR_ENABLE_BEEPER:
            command = LIDAR_DISABLE_BEEPER;
            data = 0x01;
            checksum = 0x49;
            break;
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
    
    char str_buf[60];
    sprintf(str_buf,"(Generate command 1/1) Generated command: %X %X %X %X %X %X\n", command_packet[0],command_packet[1],command_packet[2],command_packet[3],command_packet[4],command_packet[5]);
    debug(DEBUG_LIDAR,str_buf);
};

void LDK_2M::parse_response(char raw_message[], lidar_received_msg* msg)
{
    int i;

    int data_size; // Size of data in message (i.e. not address, command, or checksum)
    int start; // Loop start position
    char checksum; // Received checksum
    unsigned int calculated_checksum; // Calculated checksum

    // Assign address and command form raw message
    msg->address = raw_message[0];
    msg->command = raw_message[1];
    
    // Size of received message => address, command, data[MAX 12], checksum
    data_size = msg_len - 3;

    // Start is 2 instead of 0 due to address, command
    start = 2;

    // Loop to populate the data array
    for (i=start;i<start+LIDAR_RECEIVE_DATA_MAX_SIZE;++i)
    {
        if (i<start+data_size)
        {
            msg->data[i-start] = raw_message[i];
        } else {
            msg->data[i-start] = 0;
        }
    }
    
    // Populate checksum from raw_message
    checksum = raw_message[start+data_size];

    // Calculate checksum
    calculated_checksum = 0x00;
    calculated_checksum += (unsigned int)msg->address;
    calculated_checksum += (unsigned int)msg->command;
    
    for (i=0;i<data_size;++i)
    {
        calculated_checksum += (unsigned int)msg->data[i];
    }
    calculated_checksum = calculated_checksum & (unsigned int)0x7F;

    // validate checksum
    if ((unsigned int)calculated_checksum != (unsigned int)checksum)
    {
        debugf(DEBUG_LIDAR, "Checksum Invaid! %X != %X\n",calculated_checksum,(unsigned int)checksum);
        throw ("Checksum invalid!");
    }
}

float LDK_2M::GetMeasurement()
{
    
    float distance = 0.0; // Distance returned by LIDAR
    char generated_command[LIDAR_SEND_COMMAND_SIZE]; // Command to send to LIDAR
    lidar_received_msg received_msg;

    // Generate lidar single measurement command and send
    debug(DEBUG_LIDAR,"(Get measurement 1/4) Request single measure");
    generate_command(LIDAR_SINGLE_MEAS,generated_command);

    // Flush the hardware UART buffer to remove old messages
    flush_serial1();
    // Flush the software message buffer to prevent erroneous reading
    erase_buffer();

    // Send command to LIDAR
    Serial1.write(generated_command);

    // Parse resonse
    debug(DEBUG_LIDAR,"(Get measurement 2/4) Read response");
    if (!read_msg_from_uart(buffer))
    {
        return 0;
    }

    // Attempt to parse the message received over UART
    debug(DEBUG_LIDAR,"(Get measurement 3/4) Parse read response");
    try {
        parse_response(buffer,&received_msg);
        distance = to_distance(received_msg.data);
        laser_on = false;
    }
    catch(const char* e ) {
        Serial.print("ERROR: ");
        Serial.println(e);
        return 0;
    }
    catch (...)
    {
        return 0;
    }

    // Return result
    debug(DEBUG_LIDAR,"(Get measurement 4/4) Return response");
    return distance;
}

void LDK_2M::ToggleLaser()
{
    char generated_command[LIDAR_SEND_COMMAND_SIZE];

    // Generate lidar LASER ON command and send
    // TODO: check whether receive response before changing laser status
    debug(DEBUG_LIDAR,"(Toggle laser 1/1) Toggle laser");
    if (laser_on)
    {
        generate_command(LIDAR_LASER_OFF,generated_command);
        laser_on = false;
    } else {
        generate_command(LIDAR_LASER_ON,generated_command);
        laser_on = true;
    }
    Serial1.write(generated_command);
}