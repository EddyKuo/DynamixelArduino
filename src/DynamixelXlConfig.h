/*
DynamixelXlConfig.h
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
 This file contains all definition to drive Dynamixel servos (actually MX servos).
 You should modify the value according to your servo specification.
 Please visit http://support.robotis.com/en/product/dynamixel/mx_series/mx-64.htm for MX actuators description
 and http://support.robotis.com/en/product/dynamixel/dxl_communication.htm to understand Dynamixel communication protocol
*/

#ifndef DynamixelXlConfig_h
#define DynamixelXlConfig_h

/******************************************************************************
* Servo registers (ADDRESS)
******************************************************************************/

#define DXL_ADD_D_GAIN		            (0x1B)
#define DXL_ADD_I_GAIN 		            (0x1C)
#define DXL_ADD_P_GAIN   	            (0x1D)
#define DXL_ADD_GOAL_TORQUE           (0x23)
#define DXL_XL_ADD_PRESENT_POSITION      (0x25)
#define DXL_XL_ADD_PRESENT_SPEED         (0x27)
#define DXL_XL_ADD_PRESENT_LOAD          (0x29)
#define DXL_XL_ADD_PRESENT_VOLTAGE       (0x2D)
#define DXL_XL_ADD_PRESENT_TEMPERATURE   (0x2E)
#define DXL_XL_ADD_REGISTERED            (0x2F)
#define DXL_XL_ADD_MOVING                (0x31)
#define DXL_XL_ADD_LOCK                  (0x32)
#define DXL_XL_ADD_PUNCH                 (0x33)

/**************************************************************/

#endif // DynamixelXlConfig_h
