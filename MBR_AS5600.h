#ifndef MBR_AS5600_h
#define MBR_AS5600_h

#include <Arduino.h>
#include <Wire.h>

class MBR_AS5600{
  public:
    MBR_AS5600();

    // Magnet Status
    int magnetStatus = 0; // value of register (MD, ML, MH)
    int magnetStatusLast = 0;

    // Degrees
    float degAbsolute; // absolute degrees relative to sensor
    float degStart = 0; // setting angle to 0
    float degRelative = 0; // tared angle - start up angle, relative to 0
    float degAbsoluteLast = 0;
    int quadrant = 0;
    int previousQuadrant = 0; 
    float turns = 0;
    float degAccumulative = 0; // total angular displacement relative to 0
    float zeroOffset = 169.6-3.27; // for hard coding a 0 position for calibration, unused
    
    void checkMagnetStatus();
    void printMagnetStatus();
    void readDegAbsolute();
    void printDegAbsolute();
    void resetStartDeg();
    void setDegRelative();
    void checkQuadrant();
    void checkTurns();
    void setDegAccumulative();
    void checkMagnetAGC();
    void printMagnetAGC();
    void checkMagnetPresence();
    void serialOut();


  private:
    int sensorAddress = 0x36;
    // magnet
    int magnetStatusRegister = 0x0B;
    int magnetAGCRegister = 0x1A;
    // Magnet Automatic Gain Control
    int magnetAGC = 0;
    int magnetAGCLast = 0;

    // i2C Data
    int lowByteRegister = 0x0D;
    int highByteRegister = 0x0C;
    int lowbyte; // raw angle 7:0
    word highbyte; // raw angel 7:0 and 11:8
    int bitDeg; // bit degree before scaling raw angle in degrees (360/4096 * (value betweeen 0-4095)) (12 bit number max)

    // calculate angle:
    // 12-bit = 4096 max value, 365/4096 equal parts = .087890625
    float bit12Max = 4069.0;
    float totalDegrees = 360.0; // total degrees per circle
    float scalar = totalDegrees / bit12Max; //converts bit max to degrees
};

#endif