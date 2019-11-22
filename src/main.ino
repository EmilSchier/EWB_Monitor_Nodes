#include <Arduino.h>

// Includes


#define RH_MESH_MAX_MESSAGE_LEN 50

#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include "General-porpoise-functions.h"
#include "RV-3028-C7.h"
#include "Wire.h"
// In this small artifical network of 4 nodes,
#define CLIENT_ADDRESS 1
#define SERVER1_ADDRESS 2
#define SERVER2_ADDRESS 3
#define SERVER3_ADDRESS 4

// Singleton instance of the radio driver
//RH_RF95 driver;
RH_RF95 driver(53,2);

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(driver, CLIENT_ADDRESS);



RV3028 rtc;
// Defines
#define RTC_INT_PIN PD2
#define DEBUG_PIN1  PD6

double vcc;
bool tenSecHasPassed = false;

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
    Serial.println("init failed");
  // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36

  Serial.println("Routing table before EEPROM read: ");
  manager.printRoutingTable();

  manager.clearRoutingTable();          // clear routing table
  getRoutingTable(routingTableFirstAddr,&manager); // get routing table from EEPROM mem

  Serial.println("Routing table after EEPROM read: ");
  manager.printRoutingTable();
  
}

uint8_t data[] = "Hello World!";
// Dont put this on the stack:
uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];

void loop() {
  Serial.println("Sending to manager_mesh_server3");
    
  // Send a message to a rf22_mesh_server
  // A route to the destination will be automatically discovered.
  if (manager.sendtoWait(data, sizeof(data), SERVER3_ADDRESS) == RH_ROUTER_ERROR_NONE)
  {
    // It has been reliably delivered to the next node.
    // Now wait for a reply from the ultimate server
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
      Serial.println("No reply, is rf22_mesh_server1, rf22_mesh_server2 and rf22_mesh_server3 running?");
    }
  }
  else
     Serial.println("sendtoWait failed. Are the intermediate mesh servers running?");   
  manager.printRoutingTable();
  saveRoutingTable(routingTableFirstAddr,&manager);
  while(1);
  
  
}