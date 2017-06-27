#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <EEPROM.h>
//#include <printf.h>

/* Define algorithm to (weakly) authenticate nodes on the mesh. */
#define nodeID 6
const int MAX_ATTEMPTS = 30;

/*** PIR sensor ***/
const int ledPin = 5; // PWM
const int pirPin = 2;
const int calibrationTime = 20;

bool tripped = false;

//the time when the sensor outputs a low impulse
long unsigned int lowIn;

//the amount of milliseconds the sensor has to be low
//before we assume all motion has stopped
long unsigned int pause = 5000;

bool lockLow = true;
bool takeLowTime;


/**** Configure the nrf24l01 CE and CS pins ****/
RF24 radio(7, 8);
RF24Network network(radio);
RF24Mesh mesh(radio, network);
bool networkNotified = false;

uint32_t displayTimer = 0;

struct payload_t
{
  unsigned long ms;
  unsigned long counter;
  bool sensorTrip = false;
};


// Do something like this.....---------====================-------------...
bool isMotion()
{
  if (digitalRead(pirPin) == HIGH)
  {
    return true;  
  }
  else
  {
    return false;
  }
}

bool alertNetwork() 
{
  int attempts = 0;
  //if
  while (!mesh.write(&tripped, 'S', sizeof(tripped)))
  {
    if (attempts <= MAX_ATTEMPTS)
    {
      // If a write fails, check connectivity to the mesh network
      if ( ! mesh.checkConnection() )
      {
        //refresh the network address
        Serial.println("Renewing Address");
        mesh.renewAddress();
      }
      attempts++;  
    }
    else
    {
      Serial.print("Failed to connect to network after ");
      Serial.print(MAX_ATTEMPTS);
      Serial.println(" attempts!");
      return false;
    }
  }
    Serial.print("Network Notified!");
    networkNotified = true;
    return true;
}

void setup()
{
  Serial.begin(115200);

  lockLow = true;   
  pinMode(ledPin, OUTPUT);
  pinMode(pirPin, INPUT);
  digitalWrite(ledPin, LOW);
  digitalWrite(pirPin, LOW);

  //give the sensor some time to calibrate
  Serial.print("calibrating sensor ");
  for (int i = 0; i < calibrationTime; i++)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println(" done");
  Serial.println("SENSOR ACTIVE");
  //delay(50);

  // Set the nodeID manually
  mesh.setNodeID(nodeID);
  // Connect to the mesh
  Serial.println(F("Connecting to the mesh..."));
  mesh.begin();
  Serial.println("Connected");
}

////////////////////////////
//LOOP
int looped = 0;
void loop()
{
  Serial.println("Looped ");
  Serial.print(looped);
  Serial.println(" times");
  looped++;
  mesh.update();


  // Send to the master node every ten seconds.
  if (millis() - displayTimer >= 10000)
  {
    Serial.println("Checking in with mesh...");
    displayTimer = millis();

    // Send an 'M' type message containing the current millis()
    if (!mesh.write(&displayTimer, 'M', sizeof(displayTimer)))
    {

      // If a write fails, check connectivity to the mesh network
      if ( ! mesh.checkConnection() )
      {
        //refresh the network address
        Serial.println("Renewing Address");
        mesh.renewAddress();
      }
      else
      {
        Serial.println("Send fail, Test OK");
      }
    }
    else
    {
      Serial.print("Send OK: ");
      Serial.println(displayTimer);
    }
  }

  if (network.available())
  {
    Serial.println("Incoming message!");
    while (network.available())
    {
      RF24NetworkHeader header;
      payload_t payload;
      network.read(header, &payload, sizeof(payload));
      Serial.print("Received packet #");
      Serial.print(payload.counter);
      Serial.print(" at ");
      Serial.print(payload.ms);
      Serial.println(" ms");    
    }
  }



//  if (digitalRead(pirPin) == HIGH)
//  while (tripped)
Serial.println("Checking for motion");
  tripped = isMotion();
//  if (isMotion())
  if (tripped)
  {
    Serial.println("! MOTION DETECTED !");
//    digitalWrite(ledPin, HIGH);   //the led visualizes the sensors output pin state
    analogWrite(ledPin, 75); // For quieter times.
//    tripped = true;
    // Alert the network
    if (!networkNotified)
    {
      alertNetwork();
    }
    
    if (lockLow)
    {
      //makes sure we wait for a transition to LOW before any further output is made:
      lockLow = false;
      Serial.println("---");
      Serial.print(" at ");
      Serial.print(millis() / 1000);
      Serial.println(" sec");
      // Send it to master node! //
      /*** Send motion detected to notification to master node! ***/
      delay(50);
    }
    takeLowTime = true;
  } // Original end of if(tripped)
  else
  {
    Serial.println("No motion detected...");
  }

    if (digitalRead(pirPin) == LOW) 
    {
      digitalWrite(ledPin, LOW);  //the led visualizes the sensors output pin state
      tripped = false;
      networkNotified = false;
      if (takeLowTime) 
      {
        lowIn = millis();          //save the time of the transition from high to LOW
        takeLowTime = false;       //make sure this is only done at the start of a LOW phase
      }
      //if the sensor is low for more than the given pause,
      //we assume that no more motion is going to happen
      if (!lockLow && millis() - lowIn > pause) 
      {
        //makes sure this block of code is only executed again after
        //a new motion sequence has been detected
        lockLow = true;
        Serial.print("motion ended at ");      //output
        Serial.print((millis() - pause) / 1000);
        Serial.println(" sec");
        // Send motion ended notification to master node.
        /*** Send it! ***/
        delay(50);
      }
    }
//  } // New end of while(tripped)
}
