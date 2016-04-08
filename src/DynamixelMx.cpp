/*
DynamixelMx.cpp
written by Akira

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
 This library implement all the required operation to drive Dynamixel MX servo,
 
 Limitations:
  - This library is blocking, i.e. when a write or a read command occured, it will wait the answer of the servo
  - Baudrate tested up to 400 000bps on 16Mhz Arduino board (Mega 2560), with MAX485 component

 documentation for MX actuator is here:
 http://support.robotis.com/beta/en/product/dynamixel/mx_series/mx-64.htm
*/


//
// Includes
//
#include <DynamixelMx.h>

////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Constructor
//
dxlMx::dxlMx(halfDuplexSerial* port) :
 dxl(port)
{
}

//
// Destructor
//
dxlMx::~dxlMx()
{}

//
//  EEPROM commands
//

unsigned short dxlMx::setMultiturnOffset(const byte ID, const short Offset)
{ return writeDxlData(ID, DXL_ADD_MULTITURN_OFFSET, (const byte*) &Offset, 2 );}   
unsigned short dxlMx::readMultiturnOffset(const byte ID)
{ return readDxlWord(ID, DXL_ADD_MULTITURN_OFFSET); } 

unsigned short dxlMx::setResolutionDivider(const byte  ID, const byte Resolution)
{ return writeDxlData(ID, DXL_ADD_RESOLUTION_DIVIDER , &Resolution, 1 );}
byte dxlMx::readResolutionDivider(const byte ID)
{ return readDxlByte(ID, DXL_ADD_RESOLUTION_DIVIDER);}

//
//  RAM commands
//

unsigned short dxlMx::setDGain(const byte  ID, const byte gain)
{ return writeDxlData(ID, DXL_ADD_D_GAIN , &gain, 1 );}
byte dxlMx::readDGain(const byte ID)
{ return readDxlByte(ID, DXL_ADD_D_GAIN);}

unsigned short dxlMx::setIGain(const byte  ID, const byte gain)
{ return writeDxlData(ID, DXL_ADD_I_GAIN , &gain, 1 );}
byte dxlMx::readIGain(const byte ID)
{ return readDxlByte(ID, DXL_ADD_I_GAIN);}  

unsigned short dxlMx::setPGain(const byte  ID, const byte gain)
{ return writeDxlData(ID, DXL_ADD_P_GAIN , &gain, 1 );}
byte dxlMx::readPGain(const byte ID)
{ return readDxlByte(ID, DXL_ADD_P_GAIN);}

unsigned short dxlMx::setCurrent(const byte  ID, const unsigned short current)
{ return writeDxlData(ID, DXL_ADD_CURRENT ,(const byte*) &current, 2 );}
unsigned short dxlMx::readCurrent(const byte ID)
{ return readDxlWord(ID, DXL_ADD_CURRENT); }

unsigned short dxlMx::setTorqueControlEnable(const byte ID, const bool Enable)
{ return writeDxlData(ID, DXL_ADD_TORQUE_CONTROL_ENABLE, (const byte*) &Enable, 1 );} 
bool dxlMx::isTorqueControlEnable(const byte ID)
{ return (bool)readDxlByte(ID, DXL_ADD_TORQUE_CONTROL_ENABLE ); } 

unsigned short dxlMx::setGoalTorque(const byte  ID, const unsigned short Torque)
{ return writeDxlData(ID, DXL_ADD_GOAL_TORQUE ,(const byte*) &Torque, 2 );}
unsigned short dxlMx::readGoalTorque(const byte ID)
{ return readDxlWord(ID, DXL_ADD_GOAL_TORQUE); }

unsigned short dxlMx::setGoalAcceleration(const byte  ID, const byte Acceleration)
{ return writeDxlData(ID, DXL_ADD_GOAL_ACCELERATION , &Acceleration, 1 );}
byte dxlMx::readGoalAcceleration(const byte ID)
{ return readDxlByte(ID, DXL_ADD_GOAL_ACCELERATION);}



