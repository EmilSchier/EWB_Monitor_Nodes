#include <Wire.h>
#include "RV-3028-C7.h"

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

#define TIMER_TIME        5 // the time, 0 = dissabled
#define TIMER_UNIT        UNIT_SECOND
/*****************
 Determines the unit used for the countdown time
 UNIT_MINUTE    =   Minutes
 UNIT_SECOND    =   Seconds
 UNIT_M_SECOND  =   milliseconds
 ****************/
#define TIMER_REPEAT      true // Repeat mode true or false

volatile bool rtcINT = false;

const bool sender = true;
const bool listener = true;

//RTC interrupt service routine
void rtcISR() {
  rtcINT = true; // set a flag to show we had an interrupt.
}

void setup() {
  pinMode(RTC_INTERRUPT_PIN, INPUT);
  
  Serial.begin(9600);
  Wire.begin();

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
  Serial.print("Status reg: ");
  Serial.println(rtc.readRegister(RV3028_STATUS)); 
  Serial.print("Status1 reg: ");
  Serial.println(rtc.readRegister(RV3028_CTRL1));
  Serial.print("Status2 reg: ");
  Serial.println(rtc.readRegister(RV3028_CTRL2));
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

}

// Send a message
void sendMessage(bool run){
  if(!run){ // only run if supposed to
    return;
  }
  
  Serial.println("Sending to NODE1_ADDRESS");
   
}

void rtcIntHandler() {
  uint8_t statReg;
  statReg = rtc.readRegister(RV3028_STATUS);
  if (statReg == 8) {
    Serial.print("Status reg: ");
    Serial.println(statReg);
  }
  else if (statReg == 24) {
    Serial.print("Status reg: ");
    Serial.println(statReg);
  }
  
  if (!rtcINT) { // only run if the rtcINT flag is true
    return;
  }
  Serial.print("Interrupt occured");
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
