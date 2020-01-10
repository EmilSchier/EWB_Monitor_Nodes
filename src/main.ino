
#include "General_Purpose_Functions.h"
#include "RV-3028-C7.h"
#include <Wire.h>
#include <CayenneLPP.h>
#include <SPI.h>
#include <LowPower.h>

//RH_RF95/RFM96 driver;
RH_RF95 driver(4, 2); // pins for ATmega1284

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(driver, NODE1_ADDRESS);

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
#define ALARM_MINUTES 30
#define ALARM_HOURS 12
#define ALARM_WEEKDAY 0
#define ALARM_DATE 1
#define ALARM_NOT_DATES false
#define ALARM_MODE 7 // 7 disabled

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
enum states nextState;
enum states lastState;
volatile bool rtcINT = false;
statusflagsType statusflags;
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
  statusflags.connectet = true;
  statusflags.currentState = CollectData;
  lastState = Sleep;
  nextState = CollectData;
  statusflags.gsmNode = 1;
  statusflags.hasGSM = true;
  statusflags.supplyStatusFlag = SupplyIsExcellent;
}
void loop()
{
stateMashine();
  
}

void gotoSleep()
{
#ifdef DEBUGMODE
    Serial.println("Going to sleep");
    Serial.flush();
#endif
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
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
    statusflags.timerINT = true; // Indicade that this interrupt has happened
  }
  if ((flags & _BV(STATUS_AF))) // Alarm interrupt
  {
#ifdef DEBUGMODE
    Serial.println("Alarm Int");
#endif
    // code to do if this flag is high
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

void stateMashine()
{
  
  switch (statusflags.currentState)
  {
  case Sleep:
    gotoSleep();
    rtcIntHandler();
    if(statusflags.alarmINT)
    {
      nextState = DataExchange;
      statusflags.alarmINT = false;
    }else if(statusflags.timerINT){
      nextState = CollectData;
      statusflags.timerINT = false;
    }
    break;
  case CollectData:
    if(lastState != statusflags.currentState){
#ifdef DEBUGMODE
      Serial.println("Entered mode: CollectData");
#endif
      statusflags.timesAwake ++;
      
    }
    // collect data from sensors
    //lpp.addAnalogInput(1,(float) measureVCC(false));

    if(WAKE_TIMES_BEFORE_STATUS_CHECK == statusflags.timesAwake){
      updateSupplyStatus(&statusflags,&rtc);
      statusflags.timesAwake = 0;
    }
    //should i broadcast time?
    switch (statusflags.supplyStatusFlag)
    {
    case SupplyIsExcellent:
      digitalWrite(EN_LORA_PIN, HIGH); // turn on the LoRa module
      delay(3); // ved ikke hvor stort et delay der skal være
      if(!manager.init()){
#ifdef DEBUGMODE
        Serial.println("RFM96 init failed");
#endif
      }
      // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
      driver.setFrequency(RADIO_FREQUENCY);
      driver.setTxPower(10);
      broardcastTime(&rtc,&manager);
      digitalWrite(EN_LORA_PIN,LOW);
      break;
    case SupplyIsGood:
      digitalWrite(EN_LORA_PIN, HIGH); // turn on the LoRa module
      delay(3); // ved ikke hvor stort et delay der skal være
      if(!manager.init()){
#ifdef DEBUGMODE
        Serial.println("RFM96 init failed");
#endif
      }
      // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
      driver.setFrequency(RADIO_FREQUENCY);
      driver.setTxPower(10);
      broardcastTime(&rtc,&manager);
      digitalWrite(EN_LORA_PIN,LOW);
      break;
    case SupplyIsModerate:
      /* code */
      break;
    case SupplyIsBad:
      /* code */
      break;
    case SupplyIsTerrible:
      // save data to EEPROM
      break;
    default:
      break;
    }

    nextState = Sleep;
    if(nextState != statusflags.currentState){

    }
    break;
  case DataExchange:
    if(lastState != statusflags.currentState){
#ifdef DEBUGMODE
      Serial.println("Entered mode: DataExchange");
#endif
      digitalWrite(EN_LORA_PIN, HIGH); // turn on the LoRa module
      delay(3); // ved ikke hvor stort et delay der skal være
      if(!manager.init()){
#ifdef DEBUGMODE
        Serial.println("RFM96 init failed");
#endif
      }
      // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
      driver.setFrequency(RADIO_FREQUENCY);
      driver.setTxPower(10);
#ifdef DEBUGMODE
        Serial.println("Com. window start");
#endif
      rtc.disableCountdownTimer();
      rtc.setCountdownTimer(WINDOW_DURATION,UNIT_SECOND,true);
      rtc.enableCountdownTimer();
    }
    if(statusflags.hasGSM)
    {
      listenForMessages(&manager,&statusflags);
    }else{
      sendMessage(&lpp, statusflags.gsmNode, &manager, &statusflags); // only sends a message if the LPP buffer is not empty, When message is sent the LPP buffer is reset
      listenForMessages(&manager, &statusflags);
    }
#ifdef DEBUGMODE
      Serial.println("I'm just before the interrupthandler");
#endif
    rtcIntHandler();
    if(statusflags.timerINT){
      statusflags.timerINT = false;
#ifdef DEBUGMODE
      Serial.println("Com. window end");
#endif
      if(statusflags.recievedAck || statusflags.recievedmsg){
        statusflags.connectet = true;
        nextState = Sleep;
        if(statusflags.hasGSM){
          GMSSend();
        }
      } else {
        nextState = ListenForTime;
        statusflags.connectet = false;
      }
    }
    if(nextState != statusflags.currentState){
      digitalWrite(EN_LORA_PIN, LOW); // turn off the LoRa module
      // genstart countdown timeren med periodevis timer interrupt
      rtc.setCountdownTimer(timerSettings.time,timerSettings.unit,timerSettings.repatMode);
      rtc.enableCountdownTimer();
    }
    break;
  case ListenForTime:
    if(lastState != statusflags.currentState){
#ifdef DEBUGMODE
      Serial.println("Entered mode: ListenForTime");
#endif
      digitalWrite(EN_LORA_PIN, HIGH); // turn on the LoRa module
      delay(3); // ved ikke hvor stort et delay der skal være
      if(!manager.init()){
#ifdef DEBUGMODE
        Serial.println("RFM96 init failed");
#endif
      }
      // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
      driver.setFrequency(RADIO_FREQUENCY);
      driver.setTxPower(10);
      rtc.disableCountdownTimer();
      rtc.disableAlarmInterrupt();
    }
    listenForTime(&rtc, &manager, &statusflags);
    if(statusflags.connectet){
      nextState = Sleep;
    }
    if(nextState != statusflags.currentState){
      digitalWrite(EN_LORA_PIN, LOW); // turn off the LoRa module
      // genstart countdown timeren med brugerens indstillinger
      rtc.setCountdownTimer(timerSettings.time,timerSettings.unit,timerSettings.repatMode);
      rtc.enableCountdownTimer();
      rtc.enableAlarmInterrupt();
    }
    break;
  default:
#ifdef DEBUGMODE
        Serial.println("Default Case");
#endif
    break;
  }
  lastState = statusflags.currentState; 
  statusflags.currentState = nextState; 
}