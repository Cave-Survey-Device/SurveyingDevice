#ifndef HEADER_RMC3100
#define HEADER_RMC3100

#include <Wire.h>
#include <Arduino.h>
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigenSparse.h>
#include "magnetometer.h"

using namespace Eigen;

//internal register values without the R/W bit
#define RM3100_REVID_REG 0x36 // Hexadecimal address for the Revid internal register
#define RM3100_POLL_REG 0x00 // Hexadecimal address for the Poll internal register
#define RM3100_CMM_REG 0x01 // Hexadecimal address for the CMM internal register
#define RM3100_STATUS_REG 0x34 // Hexadecimal address for the Status internal register
#define RM3100_CCX1_REG 0x04 // Hexadecimal address for Cycle Count X1 internal register
#define RM3100_CCX0_REG 0x05 // Hexadecimal address for the Cycle Count X0 internal register

// I'd rather this be protected but cant get it to work

class RMC3100: public Magnetometer
{
public:
    RMC3100();

protected:
    void get_raw_data();

private:
    // options
    const int pin_drdy = 9; // CHANGE THIS
    const int RM3100Address = 0x20;
    const int initialCC = 200;
    const bool singleMode = true;
    const bool useDRDYPin = true;

    uint8_t revid;
    uint16_t cycleCount;
    float gain;

    void init();

    // Writes to a register
    void writeReg(uint8_t addr, uint8_t data);

    // Changes the cycle count
    void changeCycleCount(uint16_t newCC);
    
    // Reads from a register
    uint8_t readReg(uint8_t addr);
};



#endif