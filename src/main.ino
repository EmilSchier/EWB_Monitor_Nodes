#include "General_Purpose_Functions.h"
#include "RV-3028-C7.h"
#include <Wire.h>
#include <CayenneLPP.h>
#include <SPI.h>

//RH_RF95/RFM96 driver;
RH_RF95 driver(53,2);   // pins for Mega

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(driver, NODE1_ADDRESS);

// Cayenne Low Power Protocol
CayenneLPP lpp(PAYLOADMAXSIZE);
StaticJsonDocument<STATJSONBUFFERSIZE> jsonBuffer;   // Saved on stack. Recommended not to use if reserved memory>1kb. 
JsonArray root = jsonBuffer.to<JsonArray>();

RV3028 rtc;

// Defines
#define RTC_INT_PIN PD2
#define DEBUG_PIN1  PD6

double vcc;
//bool tenSecHasPassed = false;
const float radioFrequency = 868.0;

void rtcISR(){
  uint8_t flags = rtc.status();
  if (flags )
  {
/*    if ((flags & _BV(STATUS_PORF)))
    {
      // code to do if this flag is high
    }
    if ((flags & _BV(STATUS_EVF)))
    {
      // code to do if this flag is high
    } */
    if ((flags & _BV(STATUS_AF)))
    {
      // code to do if this flag is high
    }
    if ((flags & _BV(STATUS_TF)))
    {
      // code to do if this flag is high
    }
/*    if ((flags & _BV(STATUS_UF)))
    {
      // code to do if this flag is high
    }
    if ((flags & _BV(STATUS_BSF)))
    {
      // code to do if this flag is high
    }
    if ((flags & _BV(STATUS_CLKF)))
    {
      // code to do if this flag is high
    }
    if ((flags & _BV(STATUS_EEBUSY)))
    {
      // code to do if this flag is high
    }  */
  }
}

void setup() {
  pinMode(RTC_INT_PIN, INPUT);
  pinMode(DEBUG_PIN1, OUTPUT);
  
  // put your setup code here, to run once:
  Wire.begin();
  rtc.begin();

  //rtc.setCountdownTimer(10,UNIT_SECOND,true);
  //rtc.enableCountdownTimer();
  //attachInterrupt(0, rtcISR,FALLING);
  Serial.begin(9600);
  if (!manager.init())
    Serial.println("RFM96 init failed");
  // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
  driver.setFrequency(radioFrequency);
  //manager.setTxPower(20,false);

  Serial.println("Routing table before EEPROM read: ");
  manager.printRoutingTable();

  manager.clearRoutingTable();          // clear routing table
  getRoutingTable(routingTableFirstAddr,&manager); // get routing table from EEPROM mem

  Serial.println("Routing table after EEPROM read: ");
  manager.printRoutingTable();

  lpp.reset();
}

// Dont put this on the stack:
uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];

void loop() {
  // Do measurement
  float measVCC = (float) measureVCC(false);

  // Pack data into CayenneLPP packet
  lpp.addAnalogInput(SENSORCH0,measVCC);  

  Serial.println("Sending to NODE3_ADDRESS");
  
  // Send a message to node 3
  // A route to the destination will be automatically discovered.
  if (manager.sendtoWait(lpp.getBuffer(), lpp.getSize(), NODE3_ADDRESS) == RH_ROUTER_ERROR_NONE)
  {
    // It has been reliably delivered to the next node.
    // Now wait for a reply from the other node
    uint8_t len = sizeof(buf);
    uint8_t from;    
    if (manager.recvfromAckTimeout(buf, &len, 3000, &from))
    {
      Serial.print("got reply from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
    }
    else
    {
      Serial.println("No reply, are other nodes running?");
    }
  }
  else
     Serial.println("sendtoWait failed. Are the intermediate nodes running?");

  saveRoutingTable(routingTableFirstAddr,&manager);

  // Listen for new messages
  while(true){
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      Serial.print("got request from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);

      lpp.decode(buf,len,root);
      serializeJsonPretty(root,Serial);
      Serial.println();

      // Send a reply back to the originator client
        // if (manager.sendtoWait(data, sizeof(data), from) != RH_ROUTER_ERROR_NONE)
        //   Serial.println("sendtoWait failed");
    }
  }
}