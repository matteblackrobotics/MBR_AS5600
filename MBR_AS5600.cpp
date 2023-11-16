#include <Wire.h> //This is for i22
#include <Arduino.h>
#include "MBR_AS5600.h"
#include <MBR_movingAverage.h>

// using multiple sensors need i2C multiplexor since their addresses are the same
// Nano i2c pins:
// A4 SDA, yellow
// A5 SCL, orange

// Constructor
MBR_AS5600::MBR_AS5600(){}


// --------------------- Magnet Status ------------------------ //
// @brief: check magnet state and zero degree if magnet installed
// Status register output: 0 0 MD ML MH 0 0 0 
// MH: Too strong magnet - 100111 - DEC: 39
// ML: Too weak magnet - 10111 - DEC: 23
// MD: OK magnet - 110111 - DEC: 55
void MBR_AS5600::checkMagnetStatus(){
  magnetStatusLast = magnetStatus;
  Wire.beginTransmission(sensorAddress);
  Wire.write(magnetStatusRegister); // figure 21 - register map: Status: MD ML MH
  Wire.endTransmission();
  Wire.requestFrom(sensorAddress, 1);

  // wait till data available or timeout
  Serial.println("Wire.avilable()");
  int timeEnd = millis() + 5000;
  int timeRemaining = timeEnd - millis();
  while(Wire.available() == 0 && timeRemaining > 0){
    timeRemaining = timeEnd - millis();
    Serial.println(timeRemaining);
  }
  if(timeRemaining < 0){
    Serial.print("no Wire data. end");
    while(1);
  }

  magnetStatus = Wire.read();
}


void MBR_AS5600::printMagnetStatus(){
  Serial.print("Magnet Status: ");
  Serial.print(magnetStatus);
  Serial.print(", BIN: ");
  Serial.print(magnetStatus, BIN);
  Serial.print(" ");

  if((magnetStatus) == 103){
    Serial.println(" MH: Magnet too strong 1100111 ");
  }
  else if((magnetStatus) == 19){
    Serial.println(" ML: Magnet too weak 1010111 "); // 87
  }
  else if((magnetStatus) == 51){
    Serial.println("MD: Magent detected 1110111"); //
  }
  else{
    Serial.println(" data error ");
  }
}


// --------------------- Read Degree ------------------------ //
// @brief: read 2 registers on i2c and assemble data to single 12-bit value from 2 8-bits
// @intput: sensor address, data registers
// @output: degAbsolute
void MBR_AS5600::readDegAbsolute(){
  // 7:0 bits (lowbyte)
  Wire.beginTransmission(sensorAddress); // Select sensor
  Wire.write(lowByteRegister); // figure 21 - register map: Raw angle (7:0) point to register
  Wire.endTransmission(); 
  Wire.requestFrom(sensorAddress, 1); // request from sensor (1) byte
  while(Wire.available() == 0); // waiting for wire
  lowbyte = Wire.read();

  // 11:8 - 4 bits (highbyte)
  Wire.beginTransmission(sensorAddress);
  Wire.write(highByteRegister); // figure 21 - register map: Raw angle (11:8) read register
  Wire.endTransmission();
  Wire.requestFrom(sensorAddress, 1);
  while(Wire.available()== 0); // waiting for wire 
  highbyte = Wire.read();

  // bit shift:
  // building 12-bit number from 16-bit number
  // variable is shifted to the left 8 bits
  // Initial value: 00000000|00001111 (word = 16 bits or two bytes)
  // Left shift: 00001111|00000000 (filling high byte) 
  highbyte = highbyte << 8;

  // bit combination:
  // highbyte: 00001111|00000000
  // lowbyte: 00000000|00001111
  // h|l: 00001111|00001111
  bitDeg = highbyte | lowbyte; // bitwise (or) function 16-bits word bit degree

  // convert bits to degrees
  degAbsoluteLast = degAbsolute;
  degAbsolute = bitDeg * scalar;

  // include mechancial offset
  degAbsolute = degAbsolute - zeroOffset;
}

void MBR_AS5600::printDegAbsolute(){
  Serial.print("degAbsolute: ");
  Serial.println(degAbsolute);
}


// @brief: When the magent is reset, reset the 0 angle
// @input: magnetStatus, magnetStatusLast
// @output: degStart
void MBR_AS5600::resetStartDeg(){
  if(magnetStatus != magnetStatusLast){
    if(magnetStatus == 51)
    {
      readDegAbsolute();
      degStart = degAbsolute;
    }
  }
}

// more dependent on being placed in right part of loop
void MBR_AS5600::resetStartDeg1(){
  if(magnetStatus != magnetStatusLast){
    degStart = degAbsolute;
  }
}



// @brief: Tare angle from degStart
// @input: degAbsolute, degStart
// @output: degRelative
void MBR_AS5600::setDegRelative(){
  degRelative = degAbsolute - degStart;
  if(degRelative < 0){
    degRelative = degRelative + totalDegrees;
  }
}


// @brief: check relative quadrant of position
// @input: degRelative
// @output: quadrant, previousQuadrant
void MBR_AS5600::checkQuadrant(){
  previousQuadrant = quadrant;
  if(degRelative >=0 && degRelative <= 90){
    quadrant = 1;
  }
  if(degRelative > 90 && degRelative <= 180){
    quadrant = 2;
  }
  if(degRelative > 180 && degRelative <= 270){
    quadrant = 3;
  }
  if(degRelative >270 && degRelative < 360){
    quadrant = 4;
  }
}


// @brief: determine number of full relative rotations from degRelative
// @input: quadrant, previousQuadrant
// @output: turns
void MBR_AS5600::checkTurns(){
  if(quadrant != previousQuadrant){
    if(quadrant == 1 && previousQuadrant == 4){
      turns++; // clockwise
    }
    if(quadrant == 4 && previousQuadrant == 1){
      turns--; // counterclockwise
    }
  }
}


// @brief: track total angular displacement
// @input: degRelative, turns
// @output: degAccumulative
void MBR_AS5600::setDegAccumulative(){
  degAccumulative = degRelative + (turns * totalDegrees);
}


// --------------------- MagnetAGC ------------------------ //

//@brief: Automatic Gain Control, @5v [0:255] gain should be middle of range
// @3.3v [0:127]
// AGC works, magnet sits too high off device
void MBR_AS5600::checkMagnetAGC(){
  Wire.beginTransmission(sensorAddress); 
  Wire.write(magnetAGCRegister); 
  Wire.endTransmission(); 
  Wire.requestFrom(sensorAddress, 1); // request from sensor (1) byte
  while(Wire.available() == 0); // waiting for wire
  magnetAGC = Wire.read();
}

void MBR_AS5600::printMagnetAGC(){
  Serial.print("magnetAGC: ");
  Serial.println(magnetAGC);
}



// Old code
void MBR_AS5600::checkMagnetPresence()
{
  // magentStatus
  // magnet status register 0x0B: [0 0 MD ML MH 0 0 0]
  // magnetSatus & 32: 32 = 10000. if magnetStats has 1 at MD and is not 1 at that location, keep checking
  while((magnetStatus & 32) != 32){ // while magnet not at proper distance, binary 32 = 100000: MD = 1
    magnetStatus = 0;

    Wire.beginTransmission(sensorAddress);
    Wire.write(0x0B); // figure 21 - register map: Status: MD ML MH
    Wire.endTransmission();
    Wire.requestFrom(sensorAddress, 1);

    while(Wire.available() == 0);
    magnetStatus = Wire.read();

    //Serial.print("Magnet status: ");
    //Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
  }

  // Status register output: 0 0 MD ML MH 0 0 0 
  // MH: Too strong magnet - 100111 - DEC: 39
  // ML: Too weak magnet - 10111 - DEC: 23
  // MD: OK magnet - 110111 - DEC: 55
}

void MBR_AS5600::serialOut(){
  Serial.print(magnetStatus, BIN); Serial.print("\t");
  Serial.print(magnetAGC); Serial.print("\t");

  Serial.print(degAbsolute); Serial.print("\t");
  Serial.print(degRelative); Serial.print("\t");
  
  Serial.print(quadrant); Serial.print("\t");
  Serial.print(turns); Serial.print("\t");

  Serial.print(degAccumulative); Serial.print("\t");
}