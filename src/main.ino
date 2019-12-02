#include "General_Purpose_Functions.h"
#include "RV-3028-C7.h"
#include <Wire.h>
#include <CayenneLPP.h>
#include <SPI.h>
#include <LowPower.h>

//RH_RF95/RFM96 driver;
RH_RF95 driver(4,2);   // pins for Mega

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(driver, NODE1_ADDRESS);

// Cayenne Low Power Protocol
CayenneLPP lpp(PAYLOADMAXSIZE);
StaticJsonDocument<STATJSONBUFFERSIZE> jsonBuffer;   // Saved on stack. Recommended not to use if reserved memory>1kb.
JsonArray root = jsonBuffer.to<JsonArray>();

RV3028 rtc;

// Defines
// PIN definitions
#define EN_LORA           3
#define RTC_INTERRUPT_PIN 10

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

#define TIMER_TIME        20 // the time, 0 = dissabled
#define TIMER_UNIT        UNIT_SECOND
/*****************
 Determines the unit used for the countdown time
 UNIT_MINUTE    =   Minutes
 UNIT_SECOND    =   Seconds
 UNIT_M_SECOND  =   milliseconds
 ****************/
#define TIMER_REPEAT      true // Repeat mode true or false


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
  pinMode(EN_LORA,OUTPUT);

  digitalWrite(EN_LORA,HIGH); // enable power to the LoRa module

  //
Serial.begin(9600);
  Wire.begin();
  
  // setup RTC 
  rtc.begin();
  if (rtc.setTime(SECONDS, MINUTES, HOURS, WEEKDAY, DATE, MONTH, YEAR)) {
    Serial.println("time set");
  }
  rtc.enableAlarmInterrupt(ALARM_MINUTES, ALARM_HOURS, ALARM_HOURS, ALARM_NOT_DATES, ALARM_MODE);
  rtc.setCountdownTimer(TIMER_TIME, TIMER_UNIT, TIMER_REPEAT);
  //rtc.enableCountdownTimer(); // uncomment to enable the countdown timer
  //rtc.disableCountdownTimer(); // Uncomment to disable the countdown timer
  rtc.clearInterrupts();
  attachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN), rtcISR, FALLING);
  
  
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

  digitalWrite(EN_LORA,LOW); // Cut power for the Lora module
  Serial.println("starting test, awake doing nothing");
   rtc.enableCountdownTimer();
}

 //Dont put this on the stack:
uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];
bool gotosleep = false;  
bool test = false;

void loop() {

/************************
 * This is code for testing power draw of the system in different states. 
 * 1. The microcontroller wil start of with 10 seconds of doing nothing after setup
 * 2. then 10 seconds of sleep in deep sleep mode.
 * 3. a series of messages wil be sent approximatley 5 seconds appart 
 *    with increasing power levels
 * 4. The microcontroller will listen for messages with the LoRa module
 * 5. the microcontroller will go to an infinete loop
*************************/ 

  if(gotosleep){
    Serial.println("going to sleep");
    delay(200);
    LowPower.powerDown(SLEEP_FOREVER,ADC_OFF,BOD_OFF);
    gotosleep = false; // only go to sleep again when asked to.
  }
  rtcIntHandler();

  if(test){
     rtc.disableCountdownTimer();
     Serial.println("sending messages");
    digitalWrite(EN_LORA,HIGH);
    Serial.println("tx power: +5"); // er ikke komplet sikker på værdigerne, men i RH_RF95.h står der  mellem +5 og +20
                                    // databladet siger +2 til +20 men at man skal være opmærksom ved +20
    driver.setTxPower(5,false);
    sendMessageTest(true);
    delay(5000);
    Serial.println("tx power: +10");
    driver.setTxPower(10,false);
    sendMessageTest(true);
    delay(5000);
    Serial.println("tx power: +13 (standard)");
    driver.setTxPower(13,false);
    sendMessageTest(true);
    delay(5000);
    Serial.println("tx power: +15");
    driver.setTxPower(15,false);
    sendMessageTest(true);
    delay(5000);
    Serial.println("tx power: +20");
    driver.setTxPower(20,false);
    sendMessageTest(true);
    delay(5000);
    
    Serial.println("starting to listen");
    while (test)
    {
      listenForMessages(true);
    }

    digitalWrite(EN_LORA,LOW);
    Serial.println("test done going to infinete loop");
  while (1)
    {
      // loop forever
    }
  }
  
  
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
    Serial.print(from, HEX);
    Serial.print(": ");
    Serial.println((char*)buf);

    lpp.decode(buf,len,root);
    serializeJsonPretty(root,Serial);
    Serial.println();

    test = false;
    // Send a reply back to the originator client
    // if (manager.sendtoWait(data, sizeof(data), from) != RH_ROUTER_ERROR_NONE)
    //   Serial.println("sendtoWait failed");
  }
}

void sendMessageTest(bool run){
  if(!run){ // only run if supposed to
    return;
  }
  
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
}
uint8_t timesAwake = 0;
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
    Serial.print("ALARM!! at: ");
    Serial.println(rtc.stringTime());
  }
  if ((flags & _BV(STATUS_TF))) // Timer interrupt
  {
    // code to do if this flag is high
    Serial.print("Timer at: ");
    Serial.println(rtc.stringTime());
    
    if (0 == timesAwake)
    {
      timesAwake ++;
      gotosleep = true;
    }else if (1 == timesAwake)
    {
      timesAwake ++;
      test = true;
    }
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