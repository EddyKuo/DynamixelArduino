/*
DynamixelXl.cpp
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
#include <DynamixelXl.h>

////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Constructor
//
dxlXl::dxlXl(halfDuplexSerial* port) :
 dxl(port)
{
}

//
// Destructor
//
dxlXl::~dxlXl()
{}

//
//  RAM commands
//

bool dxlXl::setDGain(const byte  ID, const byte gain)
{ return sendDxlWrite(ID, DXL_ADD_D_GAIN , &gain, 1 );}
bool dxlXl::readDGain(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_D_GAIN , 1); }

bool dxlXl::setIGain(const byte  ID, const byte gain)
{ return sendDxlWrite(ID, DXL_ADD_I_GAIN , &gain, 1 );}
bool dxlXl::readIGain(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_I_GAIN , 1); }

bool dxlXl::setPGain(const byte  ID, const byte gain)
{ return sendDxlWrite(ID, DXL_ADD_P_GAIN , &gain, 1 );}
bool dxlXl::readPGain(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_P_GAIN , 1); }

bool dxlXl::setGoalTorque(const byte  ID, const unsigned short Torque)
{ return sendDxlWrite(ID, DXL_ADD_GOAL_TORQUE ,(const byte*) &Torque, 2 );}
bool dxlXl::readGoalTorque(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_GOAL_TORQUE , 2); }



bool dxlXl::readXlPresentPosition(const byte ID)
{ return sendDxlRead(ID, DXL_XL_ADD_PRESENT_POSITION , 2); }

bool dxlXl::readXlPresentSpeed(const byte ID)
{ return sendDxlRead(ID, DXL_XL_ADD_PRESENT_SPEED , 2); }

bool dxlXl::readXlPresentLoad(const byte ID)
{ return sendDxlRead(ID, DXL_XL_ADD_PRESENT_LOAD , 2); }

bool dxlXl::readXlVoltage(const byte ID)
{ return sendDxlRead(ID, DXL_XL_ADD_PRESENT_VOLTAGE , 1); }

bool dxlXl::readXlTemperature(const byte ID)
{ return sendDxlRead(ID, DXL_XL_ADD_PRESENT_TEMPERATURE, 1); }

bool dxlXl::isXlRegistered(const byte ID)
{ return sendDxlRead(ID, DXL_XL_ADD_REGISTERED , 1); }

bool dxlXl::isXlMoving(const byte ID)
{ return sendDxlRead(ID, DXL_XL_ADD_MOVING , 1); }

bool dxlXl::setXlEEPROMLock(const byte ID, const bool lock)
{ return sendDxlWrite(ID, DXL_XL_ADD_LOCK, (const byte*) &lock, 1 );}
bool dxlXl::isXlEEPROMLock(const byte ID)
{ return sendDxlRead(ID, DXL_XL_ADD_LOCK , 1); }

bool dxlXl::setXlPunch(const byte  ID, const unsigned short current)
{ return sendDxlWrite(ID, DXL_XL_ADD_PUNCH ,(const byte*) &current, 2 );}
bool dxlXl::readXlPunch(const byte ID)
{ return sendDxlRead(ID, DXL_XL_ADD_PUNCH , 2); }

