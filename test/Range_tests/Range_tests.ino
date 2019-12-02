#include "General_Purpose_Functions.h"
#include <Wire.h>
#include <SPI.h>
#include "RV-3028-C7.h"

//RH_RF95/RFM96 driver;
RH_RF95 driver(4,2);   // pins for 1284P
//RH_RF95 driver(53,2);   // pins for Mega

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(driver, NODE1_ADDRESS);

RV3028 rtc;

// Defines
#define RTC_INTERRUPT_PIN 10

// defines the time and date to be set
#define SECONDS     0
#define MINUTES     0
#define HOURS       12
#define WEEKDAY     0
#define DATE        1
#define MONTH       1
#define YEAR        2019

#define TIMER_TIME        10 // the time, 0 = dissabled
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
const uint16_t waitTime = 10000;

//RTC interrupt service routine
void rtcISR() {
  rtcINT = true; // set a flag to show we had an interrupt.
}

void setup() {
  pinMode(RTC_INTERRUPT_PIN, INPUT);
  pinMode(PC4,INPUT);
  pinMode(PC5,INPUT);
  pinMode(PC6,INPUT);  
  pinMode(PC7,INPUT);
  
  Serial.begin(9600);
  Wire.begin();
  
  while (!manager.init())
    Serial.println("RFM96 init failed");
  // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
  
  driver.setFrequency(radioFrequency);
  driver.setTxPower(txPower,false);

  if (PC4) {
    driver.setModemConfig(RH_RF95::Bw500Cr45Sf128);  // Fast and short range
    Serial.println("Config set: Bw500Cr45Sf128");
  }
  else if (PC5) {
    driver.setModemConfig(RH_RF95::Bw125Cr45Sf128);  // Medium range
    Serial.println("Config set: Bw500Cr45Sf128");
  }
  else if (PC6) {
    driver.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);  // Slow and long range
    Serial.println("Config set: Bw31_25Cr48Sf512");
  }
  else if (PC7) {
    driver.setModemConfig(RH_RF95::Bw125Cr48Sf4096);  // Slow and long range
    Serial.println("Config set: Bw125Cr48Sf4096");
  }
  else {
    Serial.println("Error - no input on PC4 to PC7.");
    while(true);
  }

  manager.clearRoutingTable();          // clear routing table

  // setup RTC 
  rtc.begin();
  if (rtc.setTime(SECONDS, MINUTES, HOURS, WEEKDAY, DATE, MONTH, YEAR)) {
    Serial.println("time set");
  }
  rtc.setCountdownTimer(TIMER_TIME, TIMER_UNIT, TIMER_REPEAT);
  rtc.enableCountdownTimer(); // uncomment to enable the countdown timer
  //rtc.disableCountdownTimer(); // Uncomment to disable the countdown timer
  
  rtc.clearInterrupts();
  attachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN), rtcISR, FALLING);  
}

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
    Serial.print("got request from : 0x");
    Serial.println(from, HEX);
    Serial.print("Received data: ");
    Serial.println((char*)buf);
    uint16_t Rssi = driver.lastRssi();
    Serial.print("Rssi: ");
    Serial.println(Rssi);
  }
}

// Send a message
void sendMessage(bool run){
  if(!run){ // only run if supposed to
    return;
  }
  
  Serial.println("Sending to NODE2_ADDRESS");
  uint8_t payload[] = "8 bytes!";  // messageID i stedet? 
  long int recTime = 0;
  long int sendTime = millis();
  // Send start timestamp, which gives an eight byte payload. 
  if (manager.sendtoWait(payload, sizeof(payload), NODE2_ADDRESS) == RH_ROUTER_ERROR_NONE)
  {
    int recTime = millis();  // is run as soon as the data has been send and acknowledged by the next node. 
    
    // It has been reliably delivered to the next node.
    long int resTime = recTime - sendTime;
    Serial.print("Message send and ACK received. Message ID: ");
    Serial.println(messageID);
    Serial.print("resTime = ");
    Serial.println(resTime);
  }
  else
  {
    Serial.println("No ack, are other nodes running or too far away?");
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

    sendMessage(sender);
    
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
