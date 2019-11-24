#include <Arduino.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include "General_Purpose_Functions.h"
#include "RV-3028-C7.h"
#include "Wire.h"
#include <CayenneLPP.h>

// Singleton instance of the radio driver
//RH_RF95 driver;
RH_RF95 driver(53,2);  // for Mega

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(driver, NODE1_ADDRESS);

RV3028 rtc;
// Defines
#define RTC_INT_PIN PD2
#define DEBUG_PIN1  PD6

CayenneLPP lpp(PAYLOADMAXSIZE);

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
    Serial.println("Serial init failed");
  // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36

  Serial.println("Routing table before EEPROM read: ");
  manager.printRoutingTable();

  manager.clearRoutingTable();                      // clear routing table
  getRoutingTable(routingTableFirstAddr,&manager);  // get routing table from EEPROM mem

  Serial.println("Routing table after EEPROM read: ");
  manager.printRoutingTable();
}

uint8_t data[] = "Faux sensor data";
// Dont put this on the stack:
uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];

uint8_t data[] = "e";

void loop() {
  uint8_t len = sizeof(buf);
  uint8_t from;
  if (manager.recvfromAck(buf, &len, &from))
  {
    Serial.print(from, HEX);

    // Send a reply back to the originator client
    if (manager.sendtoWait(data, sizeof(data), from) != RH_ROUTER_ERROR_NONE)
  }

  saveRoutingTable(routingTableFirstAddr,&manager);
  while(true);
}