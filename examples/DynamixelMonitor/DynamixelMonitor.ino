/*
DynamixelMonitor.ino
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
 This is an example using DynamixelArduino lib.
 Hardware is quite simple : 
  - connect servo data pin to pin 8 (Arduino Leonardo)
  - servo GND to Arduino GND
  - servo VDD should be connected to 9~12V power supply
  - take care that power supply should be grounded with Arduino GND
  

To operate, simply construct a dxl object the only argument is the data pin used to communicate (see SoftHalfDuplexSerial lib limitations)
dxl dxlCom(8); // data pin 8

Then start communication with servo using dxl::begin(baudrate)
dxlCom.begin(57600);
 Baudrate is set to 57600bps for safety (please check that the servo is also set to this baudrate)
  
 To drive servo, open Serial Monitor in Arduino IDE and send command (upper part of the window):
  - "ID2" change PC operation to servo ID 2 for example
  - "ping" will ping servo (ID previously entered, by default PC will operate servo ID 1)
  - "action" will drive the reg command of the servo (see dynamixel doc)
  - "reboot" will reboot servo (ID previously entered, by default PC will operate servo ID 1)
  - "model" will return the servo model
  - "firmware" will return the servo firmware
  - "setID3" will set the servo ID (the one operated by the PC) to ID 3
  - "led1" will turn on the led ("led0" to turn off)
  - "move100" will move the servo to an angle of 100 (see dynamixel doc). It will also print the present position of servo during the movement.
  - "speed300" will set the speed to 300 (see dynamixel doc). Note that in DynamixelArduino lib provide a dxl::setGoalPositionAtSpeed(ID, position, speed) method
  - "torque20" will set up the servo torque to 20 (see dynamixel doc)
  - "voltage" will return the voltage of the servo (to be divided by 10 to get it in Volt)
  - "temperature" will return the temperature of the servo (in celsius)
  - "regmove100" will load an reg action. It has been provide to illustrate how to use advanced use of the lib (type "action" to trigger the reg action)

All the command send in the serial monitor should be follow by a hit on the enter key. Please not that there is no space between command and arguments
The result of the command (including catching error) is return in the serial monitor.

*/

#include <DynamixelArduino.h>

dxl dxlCom(8); // data pin 8
String readString;
int id = 1;

void printServoId(String msg);


void setup() {

  // Open serial communications and wait for port to open (PC communication)
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Starting COM!");

  dxlCom.begin(57600); // set the data rate for the SoftwareSerial port (servo communication)
  
}

/////////////////////////////////////////////////////////////////////////////////////

void loop() { // run over and over
  
 /*a if (dxlCom.isListening()) 
    Serial.println("dxl listening!");*/

 while (Serial.available()) {
    char c = Serial.read();  //gets one byte from serial buffer
    readString += c; //makes the string readString
    delay(2);  //slow looping to allow buffer to fill with next character
  }

  if (readString.length() >0) 
  {
    if (readString.startsWith("ID"))
    {
       readString.remove(0, 2);
       id = readString.toInt();  //convert readString into a number
       printServoId("Communicating with");
       Serial.println(id);
    }
    else if (readString.startsWith("ping"))
    {
      printServoId("Ping");
      printDxlError(dxlCom.ping(id));
    }
    else if (readString.startsWith("action"))
    {
      if (dxlCom.isRegistered(id))
      {
        printServoId("Execute reg command in");
        printDxlError(dxlCom.action(id));
      }
      else
      {
        printServoId("No reg command in");
        Serial.println();
      }
    }
    else if (readString.startsWith("reboot"))
    {
      printServoId("Reboot");
      printDxlError(dxlCom.reboot(id));
    }
    else if (readString.startsWith("model"))
    {
      printServoId("Model number of");
      Serial.println(dxlCom.readModelNumber(id));
    }
    else if (readString.startsWith("firmware"))
    {
      printServoId("Firmware number of");
      Serial.println(dxlCom.readFirmware(id));
    }
    else if (readString.startsWith("setID"))
    {
      readString.remove(0, 5);
      int newID = readString.toInt();  //convert readString into a number
      printServoId("Setting");
      Serial.print(id);
      Serial.print(" to ");
      Serial.print(newID);
      Serial.print(" : ");
      printDxlError(dxlCom.setId(id, newID));
    }
    else if (readString.startsWith("led"))
    {
      readString.remove(0, 3);
      bool Status = readString.toInt();  //convert readString into a number
      printServoId("Changing led status of ");
      printDxlError(dxlCom.setLedEnable(id,Status));
    }
    else if (readString.startsWith("move"))
    {
      readString.remove(0, 4);
      unsigned short Position = readString.toInt();  //convert readString into a number
      printServoId("Moving ");
      printDxlError(dxlCom.setGoalPosition(id,Position));
      while (dxlCom.isMoving(id))
      {
        Serial.println(dxlCom.readPresentPosition(id));
        delay(10);
      }
    }
    else if (readString.startsWith("speed"))
    {
      readString.remove(0, 5);
      unsigned short Speed = readString.toInt();  //convert readString into a number
      printServoId("Set speed of ");
      printDxlError(dxlCom.setMovingSpeed(id,Speed));
    }
    else if (readString.startsWith("torque"))
    {
      readString.remove(0, 6);
      unsigned short torque = readString.toInt();  //convert readString into a number
      printServoId("Set torque of ");
      printDxlError(dxlCom.setTorqueLimit(id,torque));
    }
    else if (readString.startsWith("voltage"))
    {
      printServoId("Voltage (to be divided by 10) of");
      Serial.println(dxlCom.readVoltage(id));
    }
    else if (readString.startsWith("temperature"))
    {
      printServoId("Temperature in celsius of");
      Serial.println(dxlCom.readTemperature(id));
    }
    else if (readString.startsWith("regmove"))
    {
      readString.remove(0, 7);
      unsigned short Position = readString.toInt();  //convert readString into a number
      printServoId("Write command (type action to execute) in REG register of ");
      printDxlError(dxlCom.writeDxlRegData(id, DXL_ADD_GOAL_POSITION, (const byte*) &Position,2 ));
    }
 
    readString=""; //empty for next input
  } 


}
void printServoId(String msg)
{
  Serial.print(msg);
  Serial.print(" servo ID ");
  Serial.print(id);
  Serial.print(" : ");
}

void printDxlError(unsigned short dxlError)
{
  // after any operation error can be retrieve using dx::getError() (i.e. after read or write operation)
  if(dxlError == DXL_ERR_SUCCESS)
    Serial.println("OK");
  else
  {
    if (dxlError && DXL_ERR_VOLTAGE)
      Serial.print("voltage out of range-");
    if (dxlError && DXL_ERR_ANGLE)
      Serial.print("angle out of range-");
    if (dxlError && DXL_ERR_OVERHEATING)
      Serial.print("overheating-");
    if (dxlError && DXL_ERR_RANGE)
      Serial.print("cmd out of range-");
    if (dxlError && DXL_ERR_TX_CHECKSUM)
      Serial.print("Tx CRC invalid-");
    if (dxlError && DXL_ERR_OVERLOAD )
      Serial.print("overload-");
    if (dxlError && DXL_ERR_INSTRUCTION )
      Serial.print("undefined instruction-");
    if (dxlError && DXL_ERR_TX_FAIL )
      Serial.print("Tx No header-");
    if (dxlError && DXL_ERR_RX_FAIL )
      Serial.print("Rx No header-");
    if (dxlError && DXL_ERR_TX_ERROR  )
      Serial.print("Tx error-");
    if (dxlError && DXL_ERR_RX_WAITING  )
      Serial.print("Rx waiting-");  // Not implemented yet
    if (dxlError && DXL_ERR_RX_TIMEOUT)
      Serial.print("timeout-");
    if (dxlError && DXL_ERR_RX_CORRUPT)
      Serial.print("Rx CRC invalid-");
    if (dxlError && DXL_ERR_ID )
      Serial.print("Wrong ID answered-"); // ?? Hardware issue
    Serial.println();
  }
}

