
#include "General_Purpose_Functions.h"
#include "RV-3028-C7.h"
#include <Wire.h>
#include <CayenneLPP.h>
#include <SPI.h>
#include <LowPower.h>

//RH_RF95/RFM96 driver;
RH_RF95 driver(4, 2); // pins for ATmega1284

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(driver, NODE3_ADDRESS);

// Cayenne Low Power Protocol
CayenneLPP lpp(PAYLOADMAXSIZE);
//StaticJsonDocument<STATJSONBUFFERSIZE> jsonBuffer;   // Saved on stack. Recommended not to use if reserved memory>1kb.
//JsonArray root = jsonBuffer.to<JsonArray>();

RV3028 rtc;

// Defines
// defines the time and date to be set
#define SECONDS 0
#define MINUTES 15
#define HOURS 15
#define WEEKDAY 4
#define DATE 12
#define MONTH 12
#define YEAR 2019


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
If you want to set a weekday alarm (ALARM_NOT_DATES = true), set 'ALARM_DATE' from 0 (Sunday) to 6 (Saturday)
********************************/
#define ALARM_MINUTES 0
#define ALARM_HOURS 12
#define ALARM_WEEKDAY 0
#define ALARM_DATE 1
#define ALARM_NOT_DATES false
#define ALARM_MODE 4 // 7 disabled

#define TIMER_TIME 60 // the time, 0 = dissabled
#define TIMER_UNIT UNIT_SECOND
/*****************
 Determines the unit used for the countdown time
 UNIT_MINUTE    =   Minutes
 UNIT_SECOND    =   Seconds
 UNIT_M_SECOND  =   milliseconds
 ****************/
#define TIMER_REPEAT true // Repeat mode true or false

countdownTimerType timerSettings = {TIMER_TIME, TIMER_UNIT, TIMER_REPEAT};
statusflagsType statusflags;
double vcc;
volatile bool rtcINT = false;

//RTC interrupt service routine
void rtcISR()
{
  rtcINT = true; // set a flag to show we had an interrupt.
}

void setup()
{
  pinMode(RTC_INTERRUPT_PIN, INPUT);
  pinMode(EN_LORA_PIN, OUTPUT);

  digitalWrite(EN_LORA_PIN, LOW); // turn off the LoRa module

  Serial.begin(9600);
  Wire.begin();

  // setup RTC
  rtc.begin();

  rtc.setTime(SECONDS,MINUTES,HOURS,WEEKDAY,DATE,MONTH,YEAR);


  rtc.enableAlarmInterrupt(ALARM_MINUTES, ALARM_HOURS, ALARM_HOURS, ALARM_NOT_DATES, ALARM_MODE);
  rtc.setCountdownTimer(timerSettings.time, timerSettings.unit, timerSettings.repatMode);
  rtc.enableCountdownTimer(); // uncomment to enable the countdown timer
  //rtc.disableCountdownTimer(); // Uncomment to disable the countdown timer
  rtc.clearInterrupts();
  attachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN), rtcISR, FALLING);

  lpp.reset();
  //lpp.addAnalogInput(0,100.0);
  //lpp.addAnalogInput(1,1.5);
  updateSupplyStatus(&statusflags, &rtc);
  statusflags.justRestartet = true; // Indikate that we just restartet
  statusflags.gsmNode = NODE1_ADDRESS; // The node to send data to
  statusflags.hasGSM = false; // Does this node have GSM module?
  statusflags.ownAdress = NODE3_ADDRESS;
  //statusflags.connectet = true; // testing
  
}

void loop()
{
  if (statusflags.gotosleep)
  {
#ifdef DEBUGMODE
    Serial.println("Going to sleep");
    Serial.flush();
#endif
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    statusflags.gotosleep = false;
  }
  rtcIntHandler();
  runOnTimerInterrupt(&lpp, &rtc, &manager, &driver, &statusflags, &timerSettings);
  runOnAlarmInterrupt(&lpp, &rtc, &manager, &driver, &statusflags, &timerSettings);
  if (!statusflags.alarmINT && !statusflags.timerINT)
  {

    statusflags.gotosleep = true;
  }
}

void rtcIntHandler()
{
  if (!rtcINT)
  { // only run if the rtcINT flag is true
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
  if ((flags & _BV(STATUS_TF))) // Timer interrupt
  {
#ifdef DEBUGMODE
    Serial.println("Timer Int");
#endif
    // code to do if this flag is high
    if (statusflags.alarmINT && !statusflags.windowEnd)
    {
      statusflags.windowEnd = true;
#ifdef DEBUGMODE
      Serial.println("End of communications window In interrupt");
#endif
    }
    else
    { // dont do this when the timer signalts the end ofthe alarm
      if (!statusflags.timerINT)
        statusflags.timerINT = true; // Indicade that this interrupt has happened
      /****************************************
     * Tasks where timing is important
     * For example making a measurement at a specific time ore with a specific time differance
     ***************************************/
     //float V_cap = (float)measureUnregulatetVCC()/1000.0;
     //lpp.addAnalogInput(0,V_cap);
    }
  }
  if ((flags & _BV(STATUS_AF))) // Alarm interrupt
  {
#ifdef DEBUGMODE
    Serial.println("Alarm Int");
#endif
    // code to do if this flag is high
    if (!statusflags.alarmINT)
      statusflags.alarmINT = true; // Indicade that this interrupt has happened
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
