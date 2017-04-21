/*
DynamixelXl.h
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
 This library implement all the required operation to drive Dynamixel XL servo,

 Limitations:
  - This library is non blocking, i.e. when a write or a read command occured, it will NOT wait the answer of the servo
  - Baudrate tested up to 400 000bps on 16Mhz Arduino board (Mega 2560), with MAX485 component

 documentation for XL actuator is here:
    http://support.robotis.com/en/product/actuator/dynamixel_x/xl_series/xl-320.htm
*/
#ifndef DynamixelXl_h
#define DynamixelXl_h

#include <DynamixelXlConfig.h>
#include <Dynamixel.h>
#include <Arduino.h>

/******************************************************************************
* Definitions
******************************************************************************/

class dxlXl : public dxl
{
public:

  // public methods
  dxlXl(halfDuplexSerial* port);
  ~dxlXl();

  //
  //  RAM commands
  //

  bool setDGain(const byte  ID, const byte gain);
  bool readDGain(const byte ID);

  bool setIGain(const byte  ID, const byte gain);
  bool readIGain(const byte ID);

  bool setPGain(const byte  ID, const byte gain);
  bool readPGain(const byte ID);

  bool setGoalTorque(const byte  ID, const unsigned short Torque);
  bool readGoalTorque(const byte ID);

  bool readXlPresentPosition(const byte ID);
  bool readXlPresentSpeed(const byte ID);
  bool readXlPresentLoad(const byte ID);

  bool readXlVoltage(const byte ID);
  bool readXlTemperature(const byte ID);
  bool isXlRegistered(const byte ID);
  bool isXlMoving(const byte ID);

  bool setXlEEPROMLock(const byte ID, const bool lock);
  bool isXlEEPROMLock(const byte ID);

  bool setXlPunch(const byte  ID, const unsigned short current);
  bool readXlPunch(const byte ID);
};

#endif // DynamixelXl_h
