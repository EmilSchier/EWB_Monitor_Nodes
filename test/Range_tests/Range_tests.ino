#include "General_Purpose_Functions.h"
#include <Wire.h>
#include <SPI.h>
#include "RV-3028-C7.h"

//RH_RF95/RFM96 driver;
RH_RF95 driver(4,2);   // pins for Mega
//RH_RF95 driver(53,2);   // pins for Mega

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(driver, NODE2_ADDRESS);

RV3028 rtc;

// Defines
#define RTC_INTERRUPT_PIN 10

// Pinheader P5
#define _PC4 20
#define _PC5 21
#define _PC6 22
#define _PC7 23

// defines the time and date to be set
#define SECONDS     0
#define MINUTES     0
#define HOURS       12
#define WEEKDAY     0
#define DATE        1
#define MONTH       1
#define YEAR        2019

#define TIMER_TIME        20 // the time, 0 = dissabled
#define TIMER_UNIT        UNIT_SECOND
/*****************
 Determines the unit used for the countdown time
 UNIT_MINUTE    =   Minutes
 UNIT_SECOND    =   Seconds
 UNIT_M_SECOND  =   milliseconds
 ****************/
#define TIMER_REPEAT      true // Repeat mode true or false

volatile bool rtcINT = false;
  
static uint16_t messageID = 0;
uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];

const bool sender = true;
const bool listener = true;
const float radioFrequency = 868.0;
const uint8_t txPower = 23;  // Comment for default

//RTC interrupt service routine
void rtcISR() {
  rtcINT = true; // set a flag to show we had an interrupt.
}

void setup() {
  pinMode(RTC_INTERRUPT_PIN, INPUT);
  pinMode(_PC4,INPUT);
  pinMode(_PC5,INPUT);
  pinMode(_PC6,INPUT);  
  pinMode(_PC7,INPUT);
  
  Serial.begin(9600);
  Wire.begin();
  
  while (!manager.init())
    Serial.println("RFM96 init failed");
  // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
  
  driver.setFrequency(radioFrequency);
  driver.setTxPower(txPower,false);

  manager.clearRoutingTable();          // clear routing table

  if (digitalRead(_PC4)) {
    driver.setModemConfig(RH_RF95::Bw500Cr45Sf128);  // Fast and short range
    Serial.println("Config set: Bw500Cr45Sf128");
  }
  else if (digitalRead(_PC5)) {
    driver.setModemConfig(RH_RF95::Bw125Cr45Sf128);  // Medium range
    Serial.println("Config set: Bw125Cr45Sf128");
  }
  else if (digitalRead(_PC6)) {
    driver.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);  // Slow and long range
    Serial.println("Config set: Bw31_25Cr48Sf512");
  }
  else if (digitalRead(_PC7)) {
    driver.setModemConfig(RH_RF95::Bw125Cr48Sf4096);  // Slow and long range
    Serial.println("Config set: Bw125Cr48Sf4096");
  }
  else
    Serial.println("Error - no input on PC4 to PC7. Default conf used.");

  // setup RTC 
  if (!rtc.begin())
    Serial.println("RTC setup failed");
  if (rtc.setTime(SECONDS, MINUTES, HOURS, WEEKDAY, DATE, MONTH, YEAR)) {
    Serial.println("RTC initialised. Time set");
  }
  rtc.setCountdownTimer(TIMER_TIME, TIMER_UNIT, TIMER_REPEAT);
  rtc.enableCountdownTimer(); // uncomment to enable the countdown timer
  //rtc.disableCountdownTimer(); // Uncomment to disable the countdown timer

  //rtc.enableTimerInterrupt;
  
  rtc.clearInterrupts();
  attachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN), rtcISR, FALLING);  
}

uint8_t data[] = "Test";

void loop() {

/*********************
 * the following has the same effect as it is was written before, 
 * it is just easyer to read and turn on or off.
 * *********/

  listenForMessages(listener);
  rtcIntHandler();
  
}

// Listen for new messages
void listenForMessages(bool run){
  if(!run){ // only run if supposed to
    return;
  }
  
  uint8_t len = sizeof(buf);
  uint8_t from;
  if (manager.recvfromAck(buf, &len, &from))
  {
    // Send a reply back to the originator client
    //if (manager.sendtoWait(data, sizeof(data), from) != RH_ROUTER_ERROR_NONE)
      //Serial.println("Reply failed");
      
    Serial.print("got request from : 0x");
    Serial.println(from, HEX);
    Serial.print("Received data: ");
    Serial.println((char*)buf);
    int16_t Rssi = driver.lastRssi();
    Serial.print("Rssi: ");
    Serial.println(Rssi);
  }
}

// Send a message
void sendMessage(bool run){
  if(!run){ // only run if supposed to
    return;
  }
  
  Serial.println("Sending to NODE1_ADDRESS");
  uint8_t payload[] = "8 bytes";  // messageID i stedet? 
  uint32_t recTime = 0;
  uint32_t sendTime = millis();
  int error;
  // Send start timestamp, which gives an eight byte payload. 
  if ( (error = manager.sendtoWait(payload, sizeof(payload), NODE1_ADDRESS) ) == RH_ROUTER_ERROR_NONE)
  {
    recTime = millis();  // is run as soon as the data has been send and acknowledged by the next node. 
    
    // It has been reliably delivered to the next node.
    uint16_t resTime = recTime - sendTime;
    Serial.print("Message send and ACK received. Message ID: ");
    Serial.println(messageID);
    Serial.print("resTime = ");
    Serial.println(resTime);
    messageID++;
  }
  else
  {
    Serial.println("No ack, are other nodes running or too far away?");
    Serial.print("Error code: ");
    Serial.println(error);
  }   
}

void rtcIntHandler() {
  if (!rtcINT) { // only run if the rtcINT flag is true
    return;
  }
  rtcINT = false; // clear the flag
  uint8_t flags = rtc.status();
  rtc.updateTime();

  /*    if ((flags & _BV(STATUS_PORF)))
    {
      // code to do if this flag is high
    }
    if ((flags & _BV(STATUS_EVF)))
    {
      // code to do if this flag is high
    } */
  if ((flags & _BV(STATUS_AF))) // Alarm interrupt
  {
    // code to do if this flag is high
    //Serial.print("ALARM!! at: ");
    //Serial.print(rtc.stringTime());
    
  }
  if ((flags & _BV(STATUS_TF))) // Timer interrupt
  {
    // code to do if this flag is high
    //Serial.print("Timer at: ");
    //Serial.println(rtc.stringTime());

    sendMessage(sender);  // Kun ved RTC interrupt
    
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
