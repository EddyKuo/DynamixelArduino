/*
DynamixelArduino.cpp
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

//
// Includes
//
#include <DynamixelArduino.h>

//
// Constructor
//
dxl::dxl(const uint8_t dataPin) : 
  halfDuplexSerial(dataPin)
{
  
}

//
// Destructor
//
dxl::~dxl()
{
  end();
}

void dxl::begin(const unsigned long speed)
{
  _delayForOneByte = (1400000 / speed); // safety try to adjust this value. 1 000 000 µs * 10 (8 bit + 1 start + 1 stop) rounded up 
  halfDuplexSerial::begin(speed); 
}

//
// Public Methods
//
unsigned short dxl::ping(const byte  ID)
{
 byte noUse;
 return (dxl::writeDxl(ID, DXL_PING, &noUse, 0 ));
}

unsigned short dxl::action(const byte  ID)
{
 byte noUse;
 return (dxl::writeDxl(ID, DXL_ACTION, &noUse, 0 ));
}

unsigned short dxl::reset(const byte  ID)
{
 byte noUse;
 return (dxl::writeDxl(ID, DXL_RESET, &noUse, 0 ));
}

unsigned short dxl::reboot(const byte  ID)
{
 byte noUse;
 return (dxl::writeDxl(ID, DXL_REBOOT, &noUse, 0 ));
}



//
//  EEPROM commands
//

unsigned short  dxl::readModelNumber(const byte ID)
{ return readDxlWord(ID, DXL_ADD_MODEL_NUMBER); } 

byte dxl::readFirmware(const byte ID)
{ return readDxlByte(ID, DXL_ADD_FIRMWARE); } 

unsigned short dxl::setId(const byte  ID, const byte newID)
{ return writeDxlData(ID, DXL_ADD_ID, &newID, 1 );} 
byte  dxl::readId(const byte ID) 
{ return readDxlByte(ID, DXL_ADD_ID); }
 
unsigned short dxl::setBaudRate(const byte  ID, const byte baudRate)
{  return writeDxlData(ID, DXL_ADD_BAUDRATE, &baudRate, 1 );}
byte  dxl::readBaudRate(const byte ID) 
{ return readDxlByte(ID, DXL_ADD_BAUDRATE); }

unsigned short dxl::setReturnDelayTime(const byte  ID, const byte returnDelayTime)
{  return writeDxlData(ID, DXL_ADD_RETURN_DELAY_TIME , &returnDelayTime, 1 );}
byte  dxl::readReturnDelayTime(const byte ID) 
{ return readDxlByte(ID, DXL_ADD_RETURN_DELAY_TIME); }

unsigned short dxl::setCWAngleLimit(const byte  ID, const unsigned short angle)
{  return writeDxlData(ID, DXL_ADD_CW_ANGLE_LIMIT ,(const byte*) &angle, 2 );}
unsigned short dxl::setCCWAngleLimit(const byte  ID, const unsigned short angle)
{  return writeDxlData(ID, DXL_ADD_CCW_ANGLE_LIMIT , (const byte*) &angle, 2 );}

unsigned short  dxl::readCWAngleLimit(const byte ID)
{ return readDxlWord(ID, DXL_ADD_CW_ANGLE_LIMIT); } 
unsigned short  dxl::readCCWAngleLimit(const byte ID)
{ return readDxlWord(ID,DXL_ADD_CCW_ANGLE_LIMIT); } 

unsigned short dxl::setMaxTemperature(const byte  ID, const byte maxTemperature)
{ return writeDxlData(ID, DXL_ADD_MAX_TEMPERATURE , &maxTemperature, 1 );}
byte dxl::readMaxTemperature(const byte ID)
{ return readDxlByte(ID, DXL_ADD_MAX_TEMPERATURE);}

unsigned short dxl::setMinVoltage(const byte  ID, const byte minVoltage)
{ return writeDxlData(ID, DXL_ADD_MIN_VOLTAGE , &minVoltage, 1 );}
byte dxl::readMinVoltage(const byte ID)
{ return readDxlByte(ID, DXL_ADD_MIN_VOLTAGE);}

unsigned short dxl::setMaxVoltage(const byte  ID, const byte maxVoltage)
{ return writeDxlData(ID, DXL_ADD_MAX_VOLTAGE , &maxVoltage, 1 );}
byte dxl::readMaxVoltage(const byte ID)
{ return readDxlByte(ID,DXL_ADD_MAX_VOLTAGE);}

unsigned short dxl::setMaxTorque(const byte  ID, const unsigned short maxTorque)
{ return writeDxlData(ID, DXL_ADD_MAX_TORQUE ,(const byte*) &maxTorque, 2 );}
unsigned short dxl::readMaxTorque(const byte ID)
{ return readDxlWord(ID, DXL_ADD_MAX_TORQUE); } 

unsigned short dxl::setStatusReturnLevel(const byte  ID, const byte Status)
{ return writeDxlData(ID, DXL_ADD_STATUS_RETURN_LEVEL , &Status, 1 );}
byte dxl::readStatusReturnLevel(const byte ID)
{ return readDxlByte(ID, DXL_ADD_STATUS_RETURN_LEVEL);}
  
unsigned short dxl::setAlarmLed(const byte  ID, const byte Status)
{ return writeDxlData(ID, DXL_ADD_ALARM_LED , &Status, 1 );}
byte dxl::readAlarmLed(const byte ID)
{ return readDxlByte(ID, DXL_ADD_ALARM_LED);}

unsigned short dxl::setAlarmShutdown(const byte  ID, const byte Status)
{ return writeDxlData(ID, DXL_ADD_ALARM_SHUTDOWN , &Status, 1 );}
byte dxl::readAlarmShutdown(const byte ID)
{ return readDxlByte(ID, DXL_ADD_ALARM_SHUTDOWN);}

//
//  RAM commands
//
unsigned short dxl::setTorqueEnable(const byte  ID, const bool Status)
{ return writeDxlData(ID, DXL_ADD_TORQUE_ENABLE, (const byte*) &Status, 1 );} 
bool dxl::readTorqueEnable(const byte ID)
{ return readDxlByte(ID, DXL_ADD_TORQUE_ENABLE); }

unsigned short dxl::setLedEnable(const byte  ID, const bool Status)
{ return writeDxlData(ID, DXL_ADD_LED, (const byte*) &Status, 1 );} 
bool dxl::readLedEnable(const byte  ID )
{ return readDxlByte(ID, DXL_ADD_LED); }

unsigned short dxl::setCWComplianceMargin(const byte  ID, const byte margin)
{ return writeDxlData(ID, DXL_ADD_CW_COMPLIANCE_MARGIN , &margin, 1 );}
byte dxl::readCWComplianceMargin(const byte ID)
{ return readDxlByte(ID, DXL_ADD_CW_COMPLIANCE_MARGIN);}

unsigned short dxl::setCCWComplianceMargin(const byte  ID, const byte margin)
{ return writeDxlData(ID, DXL_ADD_CCW_COMPLIANCE_MARGIN , &margin, 1 );}
byte dxl::readCCWComplianceMargin(const byte ID)
{ return readDxlByte(ID, DXL_ADD_CCW_COMPLIANCE_MARGIN);}

unsigned short dxl::setCWComplianceSlope(const byte  ID, const byte slope)
{ return writeDxlData(ID, DXL_ADD_CW_COMPLIANCE_SLOPE , &slope, 1 );}
byte dxl::readCWComplianceSlope(const byte ID)
{ return readDxlByte(ID, DXL_ADD_CW_COMPLIANCE_SLOPE);}

unsigned short dxl::setCCWComplianceSlope(const byte  ID, const byte slope)
{ return writeDxlData(ID, DXL_ADD_CCW_COMPLIANCE_SLOPE , &slope, 1 );}
byte dxl::readCCWComplianceSlope(const byte ID)
{ return readDxlByte(ID, DXL_ADD_CCW_COMPLIANCE_SLOPE);}

unsigned short  dxl::setGoalPosition(const byte ID, const short Position)
{ return writeDxlData(ID, DXL_ADD_GOAL_POSITION, (const byte*) &Position, 2 );}   

unsigned short dxl::setGoalPositionAtSpeed(const byte ID, const short Position, const short Speed)
{ 
  const short params[] = { Position, Speed };
  return writeDxlData(ID, DXL_ADD_GOAL_POSITION, (const byte*) params, 4 );
}   

unsigned short dxl::setMovingSpeed(const byte ID, const short Speed)
{ return writeDxlData(ID, DXL_ADD_MOVING_SPEED, (const byte*) &Speed, 2 );}

unsigned short dxl::setTorqueLimit(const byte ID, const short torque)
{ return writeDxlData(ID, DXL_ADD_TORQUE_LIMIT, (const byte*) &torque, 2 );}

unsigned short dxl::readPresentPosition(const byte ID)
{ return readDxlWord(ID, DXL_ADD_PRESENT_POSITION); } 

unsigned short dxl::readPresentSpeed(const byte ID)
{ return readDxlWord(ID, DXL_ADD_PRESENT_SPEED); } 

unsigned short dxl::readPresentLoad(const byte ID)
{ return readDxlWord(ID, DXL_ADD_PRESENT_LOAD); } 

byte  dxl::readVoltage(const byte ID)
{ return readDxlByte(ID, DXL_ADD_PRESENT_VOLTAGE); } 

byte  dxl::readTemperature(const byte ID)
{ return readDxlByte(ID, DXL_ADD_PRESENT_TEMPERATURE); }  

bool dxl::isRegistered(const byte ID)
{ return (bool)readDxlByte(ID, DXL_ADD_REGISTERED ); }  

bool dxl::isMoving(const byte ID)
{ return (bool)readDxlByte(ID, DXL_ADD_MOVING ); }  

unsigned short dxl::setEEPROMLock(const byte ID, const bool lock)
{ return writeDxlData(ID, DXL_ADD_LOCK, (const byte*) &lock, 1 );}  
bool dxl::isEEPROMLock(const byte ID)
{ return (bool)readDxlByte(ID, DXL_ADD_LOCK ); }    

unsigned short dxl::setPunch(const byte  ID, const unsigned short current)
{ return writeDxlData(ID, DXL_ADD_PUNCH ,(const byte*) &current, 2 );}
unsigned short dxl::readPunch(const byte ID)
{ return readDxlWord(ID, DXL_ADD_PUNCH); } 

//
// Low Level Public Methods
//
byte dxl::readDxlByte(const byte ID, const byte dxlAddress)
{
  byte result = 0;
  readDxl(ID, dxlAddress, &result, 1);
  return result;
}

unsigned short dxl::readDxlWord(const byte ID, const byte dxlAddress)
{
  byte result[2] = {0};
  readDxl(ID, dxlAddress, result, 2);

  unsigned short value;
  value = (result[1] << 8) | result[0];   // (msb << 8) | lsb
  
  return value;
}

unsigned short dxl::readDxl(const byte ID, const byte dxlAddress, byte *result, const byte nByteToBeRead)
{  
  //const byte txDataLength =  2 + 2 ;    // length is "number of parameters N +2"
  byte i;
  byte check = 0;             // used to compute checksum
  byte header[5];             // 2 start byte + ID + length + Error
 // byte result[nByteToBeRead];  

  byte Checksum = (~(ID + /*txDataLength*/ 4  + DXL_READ_DATA + dxlAddress + nByteToBeRead)) & 0xFF;
  byte sentence[] ={ DXL_START, DXL_START, ID, /*txDataLength*/ 4, DXL_READ_DATA, dxlAddress, nByteToBeRead, Checksum};
  
  flushRx(); // clear the Rx buffer

  _error = 0;  // purge error
  // Checksum = 0; //
        

  write(sentence, /*txDataLength + 4*/ 8); // txDataLength + 2 bytes header + 1 byte ID + 1 byte Checksum

  if(!waitForStatus(nByteToBeRead + 6))   // wait for the uncoming byte
    _error |=  DXL_ERR_RX_TIMEOUT;        // 2 header byte + byte to be read + ID + length + Error + checksum
   

  // Read the incoming bytes
  header[0] = read();   // 2 Starts Bytes
  header[1] = read();   
    
  header[2] = read();   // Servo ID
  header[3] = read();   // Length
  header[4] = read();   // Error

  for (i = 0; i < nByteToBeRead; i++)
  {
    result[i] = read();
    check +=  result[i];
  }

  Checksum  = read();   // Checksum
  
  if (!( ( header[0] == 255) & ( header[1]  == 255) ))
    _error |=  DXL_ERR_RX_FAIL;     // No header found

  if (header[2]!=ID)
    _error |=DXL_ERR_ID;            // The response comes from another Servo !

  if (header[4]!=0)
    _error |=header[4];             // Servo send an error we push it. To catch it error mask should be used
  

  if (Checksum != ((~(header[2] + header[3]  + header[4] + check )) & 0xFF ))
    _error |=DXL_ERR_RX_CORRUPT;    // Checksum error

  return getError();               // Returns the read value
}


unsigned short dxl::writeDxlRegData(const byte ID, const byte dxlAddress, const byte *params, const byte nByteToBeWritten )
{
  byte sentence[1+ nByteToBeWritten];
  sentence[0] = dxlAddress;
  for (byte i = 0; i < nByteToBeWritten; i++)
    sentence[i+1] = params[i];
  return ( writeDxl(ID, DXL_REG_WRITE, sentence, nByteToBeWritten + 1 ));
}

unsigned short dxl::writeDxlData(const byte ID, const byte dxlAddress, const byte *params, const byte nByteToBeWritten )
{
  byte sentence[1+ nByteToBeWritten];
  sentence[0] = dxlAddress;
  for (byte i = 0; i < nByteToBeWritten; i++)
    sentence[i+1] = params[i];
  return ( writeDxl(ID, DXL_WRITE_DATA, sentence, nByteToBeWritten + 1 ));
}

unsigned short dxl::writeDxl(const byte ID, const byte dxlCommand, const byte *params, const byte nByteToBeWritten )
{  

  byte i;
  const byte txDataLength =  nByteToBeWritten + 2 ;    // length is "number of parameters N +2"
  byte result[5];  // 2 start byte + ID + length + Error
  byte sentence[6 + nByteToBeWritten];
  byte Checksum = 0;

  for (i = 0; i < nByteToBeWritten; i++)
    Checksum += params[i];

  Checksum = (~(ID + txDataLength  + dxlCommand + Checksum )) & 0xFF;
   
  sentence[0] = sentence[1] = DXL_START;  // 2 start bytes
  sentence[2] = ID;                       // Servo ID
  sentence[3] = txDataLength;             // length
  sentence[4] = dxlCommand;               // write instruction
             
  for (i = 0; i < nByteToBeWritten; i++)  // write params ( first one is address)
    sentence[i+5] = params[i];
  
  sentence[i+5] = Checksum;

  flushRx(); // clear the Rx buffer

  _error = 0;  // purge error
  // Checksum = 0; //
        
  write(sentence, txDataLength + 4); // txDataLength + 2 bytes header + 1 byte ID + 1 byte Checksum

  if(!waitForStatus(6))   // wait for the uncoming byte
    _error |=  DXL_ERR_RX_TIMEOUT;        // 2 header byte + byte to be read + ID + length + Error + checksum 

  // Read the incoming bytes
  result[0] = read();   // 2 Starts Bytes
  result[1] = read();   
    
  result[2] = read();   // Servo ID
  result[3] = read();   // Length
  result[4] = read();   // Error

  Checksum  = read();   // Checksum
  
  if (!( ( result[0] == 255) & ( result[1]  == 255) ))
    _error |=  DXL_ERR_TX_FAIL;     // No header found

  if (result[2]!=ID)
    _error |=DXL_ERR_ID;            // The response comes from another Servo !

  if (result[4]!=0)
    _error |=result[4];             // Servo send an error we push it. To catch it error mask should be used

  if (Checksum != ((~(result[2] + result[3]  + result[4] )) & 0xFF ))
    _error |=DXL_ERR_RX_CORRUPT;    // Checksum error

  return getError();               // Returns the read value
}

 
//
// Private Methods
//


bool dxl::waitForStatus(uint8_t nByteToReceive)
{
  delayMicroseconds( 2 * DXL_RETURN_DELAY_TIME); // wait for the return delay (transmit & receive)
  delayMicroseconds( ( nByteToReceive) * _delayForOneByte ); // wait for all the forecast byte (add one for safety) 
  
  if(available() >= nByteToReceive)
    return true;
  else
    delayMicroseconds( nByteToReceive * _delayForOneByte ); // retry
    
    // TO BE CHANGED FOR LET THE RETRY WORK !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  
    //return (available() >= nByteToReceive);

  return false;
}


