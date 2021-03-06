 /** RF24Mesh_Example_Master.ino by TMRh20
  * 
  *
  * This example sketch shows how to manually configure a node via RF24Mesh as a master node, which
  * will receive all data from sensor nodes.
  *
  * The nodes can change physical or logical position in the network, and reconnect through different
  * routing nodes as required. The master node manages the address assignments for the individual nodes
  * in a manner similar to DHCP.
  *
  */
  
// =========== Mesh Network =========== //
#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <Wire.h>
#include <SPI.h>
//Include eeprom.h for AVR (Uno, Nano) etc. except ATTiny
#include <EEPROM.h>
// =========== Mesh Network =========== //



// =========== Graphics =========== //
#include <Adafruit_GFX.h>
#include <UTFTGLUE.h>
UTFTGLUE myGLCD(0x9488,A2,A1,A3,A4,A0);

// Declare which fonts we will be using
#if !defined(SmallFont)
extern uint8_t SmallFont[];    //.kbv GLUE defines as GFXFont ref
#endif
// =========== Graphics =========== //


// =========== Mesh Network =========== //
/***** Configure the chosen CE,CS pins *****/
RF24 radio(7,8);
RF24Network network(radio);
RF24Mesh mesh(radio,network);
// =========== Mesh Network =========== //


uint32_t displayTimer = 0;
//
//#define SDATA 2
//#define SCLOCK 3

void setup() 
{
  Serial.begin(115200);

  // =========== Display =========== //
  randomSeed(analogRead(5));   //.kbv Due does not like A0
  pinMode(A0, OUTPUT);       //.kbv mcufriend have RD on A0
  digitalWrite(A0, HIGH);
  // Setup the LCD
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);
  // =========== Display =========== //

  // Set the nodeID to 0 for the master node
  mesh.setNodeID(0);
  Serial.print("Node ID: ");
  Serial.println(mesh.getNodeID());

  // Write it to the oLED screen!
  Wire.beginTransmission(0x3C);
  Wire.write("Node ID: ");
  Wire.write(mesh.getNodeID());
  Wire.endTransmission();
  
  // Connect to the mesh
  mesh.begin();
}


void loop() 
{    

  // Call mesh.update to keep the network updated
  mesh.update();
  
  // In addition, keep the 'DHCP service' running on the master node so addresses will
  // be assigned to the sensor nodes
  mesh.DHCP();
  
  
  // Check for incoming data from the sensors
  if (network.available()) 
  {
    RF24NetworkHeader header;
    network.peek(header);
    
    uint32_t dat=0;
    Serial.print("Case: ");
    switch (header.type) 
    {
      // Display the incoming millis() values from the sensor nodes
      case 'M': 
        Serial.println("M");
        network.read(header,&dat,sizeof(dat));
        Serial.print("Data: "); 
        Serial.println(dat); 
        break;
      case 'S': // S is for Sensor! :D
        Serial.println("S");
        network.read(header, &dat, sizeof(dat));
        Serial.print("Data: ");
        Serial.println(dat);
      default: 
        Serial.println("Default");
        network.read(header,0,0); 
        Serial.print("Header and type: ");
        Serial.println(header.type);
        break;
    }
  }
  
  if (millis() - displayTimer > 5000)
  {
    displayTimer = millis();
    Serial.println(" ");
    Serial.println(F("********Assigned Addresses********"));
     for(int i = 0; i < mesh.addrListTop; i++)
     {
       Serial.print("NodeID: ");
       Serial.print(mesh.addrList[i].nodeID);
       Serial.print(" RF24Network Address: 0");
       Serial.println(mesh.addrList[i].address,OCT);
     }
    Serial.println(F("**********************************"));
  }
}
