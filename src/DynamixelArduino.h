/*
DynamixelArduino.h
Written by Akira

This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,  
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
 *****************************************************************************
 Decription:
 This library implement all the required operation to drive Dynamixel servo,
 using only one wire.
 It required the halfDuplexSerial lib (SoftHalfDuplexSerial.cpp & .h)
 Limitations:
  - The pin should support change interrupts (see halfDuplexSerial lib)
  - Baudrate is limited (57 600bps ~ 115 200bps for 16Mhz Arduino board)
  - 5V Arduino Board (I didn't try with 3.3V board as I didn't have one)
*/
#ifndef DynamixelArduino_h
#define DynamixelArduino_h

#include "DynamixelArduinoConfig.h"
#include <Arduino.h>

#include <SoftHalfDuplexSerial.h>


/******************************************************************************
* Definitions
******************************************************************************/

class dxl : public halfDuplexSerial
{
private:
  unsigned int    _delayForOneByte;
  unsigned short  _error;

  bool waitForStatus(uint8_t nByteToReceive);

public:
  
  // public methods
  dxl(const uint8_t dataPin);
  ~dxl();
  void begin(const unsigned long speed);

  unsigned short getError() const { return _error; }

  unsigned short ping(const byte  ID);    // ping the servo
  unsigned short action(const byte  ID);  // Excecute action written in REG_WRITE servo register
  unsigned short reset(const byte  ID);   // Reset the servo to factory default setting
  unsigned short reboot(const byte  ID);  // Reboot the servo

  unsigned short readModelNumber(const byte ID);
  byte readFirmware(const byte ID);

  unsigned short setId(const byte  ID, const byte newID);
  byte  readId(const byte ID);    // Is this really usefull ?
  
  unsigned short setBaudRate(const byte  ID, const byte baudRate);
  byte  readBaudRate(const byte ID); 
  
  unsigned short setReturnDelayTime(const byte  ID, const byte returnDelayTime);
  byte readReturnDelayTime(const byte ID);

  unsigned short setCWAngleLimit(const byte  ID, const unsigned short angle);
  unsigned short setCCWAngleLimit(const byte  ID, const unsigned short angle);

  unsigned short readCWAngleLimit(const byte ID);
  unsigned short readCCWAngleLimit(const byte ID);

  unsigned short setMaxTemperature(const byte  ID, const byte maxTemperature);
  byte readMaxTemperature(const byte ID);

  unsigned short setMinVoltage(const byte  ID, const byte minVoltage);
  byte readMinVoltage(const byte ID);

  unsigned short setMaxVoltage(const byte  ID, const byte maxVoltage);
  byte readMaxVoltage(const byte ID);

  unsigned short setMaxTorque(const byte  ID, const unsigned short maxTorque);
  unsigned short readMaxTorque(const byte ID);

  unsigned short setStatusReturnLevel(const byte  ID, const byte Status);
  byte readStatusReturnLevel(const byte ID);
  
  unsigned short setAlarmLed(const byte  ID, const byte Status);
  byte readAlarmLed(const byte ID);

  unsigned short setAlarmShutdown(const byte  ID, const byte Status);
  byte readAlarmShutdown(const byte ID);

  unsigned short setTorqueEnable(const byte  ID, const bool Status);
  bool readTorqueEnable(const byte ID);

  unsigned short setLedEnable(const byte  ID, const bool Status);
  bool readLedEnable(const byte  ID);

  unsigned short setCWComplianceMargin(const byte  ID, const byte margin);
  byte readCWComplianceMargin(const byte ID);

  unsigned short setCCWComplianceMargin(const byte  ID, const byte margin);
  byte readCCWComplianceMargin(const byte ID);

  unsigned short setCWComplianceSlope(const byte  ID, const byte slope);
  byte readCWComplianceSlope(const byte ID);

  unsigned short setCCWComplianceSlope(const byte  ID, const byte slope);
  byte readCCWComplianceSlope(const byte ID);
  
  unsigned short setGoalPosition(const byte ID, const short Position);
  unsigned short setGoalPositionAtSpeed(const byte ID, const short Position, const short Speed);
  unsigned short setMovingSpeed(const byte ID, const short Speed);
  unsigned short setTorqueLimit(const byte ID, const short torque);

  unsigned short readPresentPosition(const byte ID);
  unsigned short readPresentSpeed(const byte ID);
  unsigned short readPresentLoad(const byte ID);

  byte readVoltage(const byte ID);
  byte readTemperature(const byte ID);
  bool isRegistered(const byte ID);
  bool isMoving(const byte ID);

  unsigned short setEEPROMLock(const byte ID, const bool lock);
  bool isEEPROMLock(const byte ID);

  unsigned short setPunch(const byte  ID, const unsigned short current);
  unsigned short readPunch(const byte ID);


  // Low level public methods
  unsigned short readDxlWord(const byte ID, const byte dxlAddress);
  byte readDxlByte(const byte ID, const byte dxlAddress);

  unsigned short readDxl(const byte ID, const byte dxlAddress, byte *result, const byte nByteToBeRead);  // read operation will be stored in result
  unsigned short writeDxl(const byte ID, const byte dxlCommand, const byte *params, const byte nByteToBeWritten);
  unsigned short writeDxlData(const byte ID, const byte dxlAddress, const byte *params, const byte nByteToBeWritten);
  unsigned short writeDxlRegData(const byte ID, const byte dxlAddress, const byte *params, const byte nByteToBeWritten );
  
};

#endif // DynamixelArduino_h
