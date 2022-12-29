#include "lidar.h"

void IRAM_ATTR ISR_UART_TIMEOUT()
{
    uart_read_timedout = true;
}

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
        case LIDAR_ENABLE_BEEPER:
            command = LIDAR_DISABLE_BEEPER;
            data = 0x01;
            checksum = 0x49;
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
    sprintf(str_buf,"Generated command (in func): %X %X %X %X %X %X\n", command_packet[0],command_packet[1],command_packet[2],command_packet[3],command_packet[4],command_packet[5]);
    debug(DEBUG_LIDAR,str_buf);
};

void Lidar::receive_response(char raw_message[], lidar_received_msg* msg)
{
    int i;

    // Size of data in message (i.e. not address, command, or checksum)
    int data_size;

    // Loop start position
    int start;

    // Received checksum
    char checksum;

    // Calculated checksum
    unsigned int calculated_checksum;


    // Assign address and command form raw message
    msg->address = raw_message[0];
    msg->command = raw_message[1];
    
    // Size of received message => address, command, data[MAX 12], checksum
    data_size = msg_len - 3;

    // Start is 2 instead of 0 due to address, command
    start = 2;

    Serial.printf("%X ",msg->address);
    Serial.printf("%X ",msg->command);

    // Loop to populate the data array
    for (i=start;i<start+LIDAR_RECEIVE_DATA_MAX_SIZE;++i)
    {
        Serial.printf("%X ",raw_message[i]);

        if (i<start+data_size)
        {
            msg->data[i-start] = raw_message[i];
        } else {
            msg->data[i-start] = 0;
        }
    }
    

    // Populate checksum from raw_message
    checksum = raw_message[start+data_size];

    Serial.println(checksum);

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
        Serial.printf("Checksum Invaid! %X != %X\n",calculated_checksum,(unsigned int)checksum);
        throw ("Checksum invalid!");
    }
}

void Lidar::flush_serial1()
{
    
    char a[RX_BUFFER_SIZE];
    Serial1.readBytes(a,RX_BUFFER_SIZE);
}

Lidar::Lidar()
{   
    // Using UART1
    Serial1.setRxBufferSize(RX_BUFFER_SIZE);
    Serial1.begin(9600);
    single_char_buffer = { 0x00 };
    *buffer = { 0x00 };
}

void Lidar::init()
{
    char generated_command[LIDAR_SEND_COMMAND_SIZE];
    lidar_received_msg received_msg;

    uart_read_timer = timerBegin(1, 80, true);
    timerAttachInterrupt(uart_read_timer, &ISR_UART_TIMEOUT, true);
    timerAlarmWrite(uart_read_timer, 3000000, true);
    
    Serial1.flush();
    debug(DEBUG_LIDAR,"Get software version");
    generate_command(LIDAR_READ_SOFTWARE_VERSION,generated_command);
    Serial1.write(generated_command,LIDAR_SEND_COMMAND_SIZE);
    read_msg_from_uart(buffer);
    Serial.println(buffer);
    debug(DEBUG_LIDAR,"Disable beeper");
    generate_command(LIDAR_DISABLE_BEEPER,generated_command);
    Serial1.write(generated_command,LIDAR_SEND_COMMAND_SIZE);

    erase_buffer();

    debug(DEBUG_LIDAR,"Finished INIT");
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
    int count = 0;
    uart_read_timedout = false;
    debug(DEBUG_LIDAR,"Starting timer");
    timerStart(uart_read_timer);
    timerRestart(uart_read_timer);
    timerAlarmEnable(uart_read_timer);

    // Read buffer until either 20 characters have been read with no avail or start byte found
    // Reset single_char_buffer to prevent erroneous message success
    single_char_buffer = 0;
    while ((int)single_char_buffer != (int)LIDAR_START_BYTE)
    {
        Serial1.read(&single_char_buffer,1);
        if (uart_read_timedout)
        {
            timerStop(uart_read_timer);
            timerAlarmDisable(uart_read_timer);
            uart_read_timedout = false;
            throw ("No message received!");
        }
        count++;
    }
    // Erase buffer
    erase_buffer();

    // Reads bytes until terminator into buffer (not including terminator)
    msg_len = Serial1.readBytesUntil(LIDAR_END_BYTE,buffer,99);
}

double Lidar::to_distance(char* data)
{
    double d;
    sscanf(data, "%lf", &d);
    d = d/1000.0;
    return d;
}

double Lidar::get_measurement()
{
    // Distance returned by lidar
    double distance = 0.0;
    char generated_command[LIDAR_SEND_COMMAND_SIZE];

    lidar_received_msg received_msg;
    enable();

    // Generate lidar LASER ON command and send
    debug(DEBUG_LIDAR,"Laser on");
    generate_command(LIDAR_LASER_ON,generated_command);
    Serial1.write(generated_command);

    // Generate lidar single measurement command and send
    debug(DEBUG_LIDAR,"Single measure");
    flush_serial1();
    erase_buffer();
    generate_command(LIDAR_SINGLE_MEAS,generated_command);
    Serial1.write(generated_command);
    read_msg_from_uart(buffer);

    // Read lidar distance measurement
    try {
        receive_response(buffer,&received_msg);
        Serial.println(received_msg.data);
        distance = to_distance(received_msg.data);
    }
    catch(char* e ) {
        Serial.print("ERROR: ");
        Serial.println(e);
        return 0;
    }

    // Generate lidar off command and send
    debug(DEBUG_LIDAR,"Laser off");
    generate_command(LIDAR_LASER_OFF,generated_command);
    Serial1.write(generated_command);

    // Flush uart     
    Serial1.flush();

    // Disable laser
    disable();

    return distance;
}