/*****************************
 * microcontroller will sleep for 5 sekonds and then wake and pul pin 23/PC7 Low.
 * this is to test the time from an interupt is generatet to the processer is awake
 * **************************/
#include "General_Purpose_Functions.h"
#include "RV-3028-C7.h"
#include <Wire.h>
#include <CayenneLPP.h>
#include <SPI.h>
#include <LowPower.h>

//RH_RF95/RFM96 driver;
RH_RF95 driver(4,2);   // pins for ATmega1284p

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(driver, NODE1_ADDRESS);

// Cayenne Low Power Protocol
CayenneLPP lpp(PAYLOADMAXSIZE);
StaticJsonDocument<STATJSONBUFFERSIZE> jsonBuffer;   // Saved on stack. Recommended not to use if reserved memory>1kb.
JsonArray root = jsonBuffer.to<JsonArray>();

RV3028 rtc;

// Defines
#define RTC_INTERRUPT_PIN 10
#define EN_LORA_PIN       3
#define TESTPIN           23

// defines the time and date to be set
#define SECONDS     0
#define MINUTES     0
#define HOURS       12
#define WEEKDAY     0
#define DATE        1
#define MONTH       1
#define YEAR        2019

/*********************************
Set the alarm mode in the following way:
0: When minutes, hours and weekday/date match (once per weekday/date)
1: When hours and weekday/date match (once per weekday/date)
2: When minutes and weekday/date match (once per hour per weekday/date)
3: When weekday/date match (once per weekday/date)
4: When hours and minutes match (once per day)
5: When hours match (once per day)
6: When minutes match (once per hour)
7: All disabled � Default value
If you want to set a weekday alarm (setWeekdayAlarm_not_Date = true), set 'date_or_weekday' from 0 (Sunday) to 6 (Saturday)
********************************/
#define ALARM_MINUTES     0
#define ALARM_HOURS       12
#define ALARM_WEEKDAY     0
#define ALARM_DATE        1
#define ALARM_NOT_DATES   false
#define ALARM_MODE        7 //disabled 

#define TIMER_TIME        5 // the time, 0 = dissabled
#define TIMER_UNIT        UNIT_SECOND
/*****************
 Determines the unit used for the countdown time
 UNIT_MINUTE    =   Minutes
 UNIT_SECOND    =   Seconds
 UNIT_M_SECOND  =   milliseconds
 ****************/
#define TIMER_REPEAT      false // Repeat mode true or false


double vcc;
volatile bool rtcINT = false;
//bool tenSecHasPassed = false;
const float radioFrequency = 868.0;

//RTC interrupt service routine
void rtcISR() {
  rtcINT = true; // set a flag to show we had an interrupt.
}

void setup() {
  pinMode(RTC_INTERRUPT_PIN, INPUT);
  pinMode(EN_LORA_PIN,OUTPUT);
  pinMode(TESTPIN,OUTPUT);

  digitalWrite(EN_LORA_PIN,LOW);
  digitalWrite(TESTPIN,HIGH);

  Serial.begin(9600);
  Wire.begin();
  
  // setup RTC 
  rtc.begin();
  if (rtc.setTime(SECONDS, MINUTES, HOURS, WEEKDAY, DATE, MONTH, YEAR)) {
    Serial.println("time set");
  }
  rtc.enableAlarmInterrupt(ALARM_MINUTES, ALARM_HOURS, ALARM_HOURS, ALARM_NOT_DATES, ALARM_MODE);
  rtc.setCountdownTimer(TIMER_TIME, TIMER_UNIT, TIMER_REPEAT);
  rtc.enableCountdownTimer(); // uncomment to enable the countdown timer
  //rtc.disableCountdownTimer(); // Uncomment to disable the countdown timer
  rtc.clearInterrupts();
  attachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN), rtcISR, FALLING);
  
  /*
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
*/
  lpp.reset(); 

}

 //Dont put this on the stack:
uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];
bool gotosleep = true;  

void loop() {
  if(gotosleep){
    LowPower.powerDown(SLEEP_FOREVER,ADC_OFF,BOD_OFF);
    digitalWrite(TESTPIN,LOW);
    delay(1);
    gotosleep = false;
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
    //Serial.println(rtc.stringTime());
    
    
  }
  if ((flags & _BV(STATUS_TF))) // Timer interrupt
  {
    // code to do if this flag is high
    //Serial.print("Timer at: ");
    //Serial.println(rtc.stringTime());

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