
#include <SoftHalfDuplexSerial.h>
#include <DynamixelAx.h>

softHalfDuplexSerial port(8); // data pin 8
dxlAx dxlCom(&port); 
String readString;         // Input string from serial monitor
int id = 1;                // Default Dynamixel servo ID  

void printServoId(String msg);


void setup() {

  // Open serial communications and wait for port to open (PC communication)
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Starting COM!");

   // Open serial communications with Dynamixel servos
  dxlCom.begin(57600); 
  
}

/////////////////////////////////////////////////////////////////////////////////////

void loop()
{ 

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
      printServoId("Temperature of");
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
    if (dxlError & DXL_ERR_VOLTAGE)
      Serial.print("voltage out of range-");
    if (dxlError & DXL_ERR_ANGLE)
      Serial.print("angle out of range-");
    if (dxlError & DXL_ERR_OVERHEATING)
      Serial.print("overheating-");
    if (dxlError & DXL_ERR_RANGE)
      Serial.print("cmd out of range-");
    if (dxlError & DXL_ERR_TX_CHECKSUM)
      Serial.print("Tx CRC invalid-");
    if (dxlError & DXL_ERR_OVERLOAD )
      Serial.print("overload-");
    if (dxlError & DXL_ERR_INSTRUCTION )
      Serial.print("undefined instruction-");
    if (dxlError & DXL_ERR_TX_FAIL )
      Serial.print("Tx No header-");
    if (dxlError & DXL_ERR_RX_FAIL )
      Serial.print("Rx No header-");
    if (dxlError & DXL_ERR_TX_ERROR  )
      Serial.print("Tx error-");
    if (dxlError & DXL_ERR_RX_WAITING  )
      Serial.print("Rx waiting-");  // Not implemented yet
    if (dxlError & DXL_ERR_RX_TIMEOUT)
      Serial.print("timeout-");
    if (dxlError & DXL_ERR_RX_CORRUPT)
      Serial.print("Rx CRC invalid-");
    if (dxlError & DXL_ERR_ID )
      Serial.print("Wrong ID answered-"); // ?? Hardware issue
    Serial.println();
    
  }
}



