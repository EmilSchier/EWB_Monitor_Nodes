#include "General_Purpose_Functions.h"
#include "RV-3028-C7.h"
#include <Wire.h>
#include <CayenneLPP.h>
#include <SPI.h>
#include <LowPower.h>

//RH_RF95/RFM96 driver;
RH_RF95 driver(53, 2); // pins for Mega

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(driver, NODE1_ADDRESS);

// Cayenne Low Power Protocol
CayenneLPP lpp(PAYLOADMAXSIZE);
StaticJsonDocument<STATJSONBUFFERSIZE> jsonBuffer; // Saved on stack. Recommended not to use if reserved memory>1kb.
JsonArray root = jsonBuffer.to<JsonArray>();

RV3028 rtc;

// Defines
#define RTC_INTERRUPT_PIN 10
#define EN_LORA_PIN 3

// The number of times the the processer has to have been woken before
// checking the status af VCC and Unregulatet VCC
#define WAKE_TIMES_BEFORE_STATUS_CHECK 5 //Might need a new name
// defines the time and date to be set
#define SECONDS 0
#define MINUTES 0
#define HOURS 12
#define WEEKDAY 0
#define DATE 1
#define MONTH 1
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
If you want to set a weekday alarm (setWeekdayAlarm_not_Date = true), set 'date_or_weekday' from 0 (Sunday) to 6 (Saturday)
********************************/
#define ALARM_MINUTES 0
#define ALARM_HOURS 12
#define ALARM_WEEKDAY 0
#define ALARM_DATE 1
#define ALARM_NOT_DATES false
#define ALARM_MODE 7 //disabled

#define TIMER_TIME 0 // the time, 0 = dissabled
#define TIMER_UNIT UNIT_SECOND
/*****************
 Determines the unit used for the countdown time
 UNIT_MINUTE    =   Minutes
 UNIT_SECOND    =   Seconds
 UNIT_M_SECOND  =   milliseconds
 ****************/
#define TIMER_REPEAT true // Repeat mode true or false

#define HAS_GSM false // set false if the specific node to be set up does not have a GSM module

countdownTimerType timerSettings = {TIMER_TIME, TIMER_UNIT, TIMER_REPEAT};
countdownTimerType windowTimerSettings = {WINDOW_DURATION, UNIT_SECOND, false};
statusflagsType statusflags;
double vcc;
volatile bool rtcINT = false;
const float radioFrequency = 868.0;

#if HAS_GSM == true
struct bufStruct
{
  uint8_t buf[RH_MESH_MAX_MESSAGE_LEN * 20];
  uint16_t curser;
} dataBuf;
#endif

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
  if (rtc.setTime(SECONDS, MINUTES, HOURS, WEEKDAY, DATE, MONTH, YEAR))
  {
    Serial.println("time set");
  }
  rtc.enableAlarmInterrupt(ALARM_MINUTES, ALARM_HOURS, ALARM_HOURS, ALARM_NOT_DATES, ALARM_MODE);
  rtc.setCountdownTimer(timerSettings.time, timerSettings.unit, timerSettings.repatMode);
  //rtc.enableCountdownTimer(); // uncomment to enable the countdown timer
  rtc.disableCountdownTimer(); // Uncomment to disable the countdown timer
  rtc.clearInterrupts();
  attachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN), rtcISR, FALLING);

  Serial.println("Routing table before EEPROM read: ");
  manager.printRoutingTable();

  manager.clearRoutingTable();                      // clear routing table
  getRoutingTable(routingTableFirstAddr, &manager); // get routing table from EEPROM mem

  Serial.println("Routing table after EEPROM read: ");
  manager.printRoutingTable();

  lpp.reset();
  updateSupplyStatus(&statusflags,&rtc);
  statusflags.justRestartet = true; // Indikate that we just restartet
}

void loop()
{
  if (statusflags.gotosleep)
  {
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
  rtcIntHandler();
  runOnTimerInterrupt();
  runOnAlarmInterrupt();
  if(!statusflags.alarmINT && !statusflags.timerINT)
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
  if ((flags & _BV(STATUS_AF))) // Alarm interrupt
  {
    // code to do if this flag is high
    if (!statusflags.alarmINT)
      ;
    statusflags.alarmINT = true; // Indicade that this interrupt has happened
  }
  if ((flags & _BV(STATUS_TF))) // Timer interrupt
  {
    // code to do if this flag is high
    if (!statusflags.timerINT)
      statusflags.timerINT = true; // Indicade that this interrupt has happened
    if (statusflags.alarmINT)
    {
    }
    /****************************************
     * Tasks where timing is important
     * For example making a measurement at a specific time ore with a specific time differance
     ***************************************/
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

void runOnTimerInterrupt()
{
  if (!statusflags.timerINT)
    return;
  //only run once per timer interrupt
  statusflags.timerINT = false;
  // see if the timer time is very low, if it is, then do not count them as waking
  if ((20 <= timerSettings.time && UNIT_SECOND == timerSettings.unit) || (10000 <= timerSettings.time && UNIT_M_SECOND == timerSettings.unit))
  {
    /****************************************
     * Check status of VCC and UnregulatetVCC
     * if it is time for this
     ***************************************/
    if (WAKE_TIMES_BEFORE_STATUS_CHECK <= statusflags.timesAwake)
    {

      updateSupplyStatus(&statusflags,&rtc);
      statusflags.timesAwake = 0;
    }
    else
    {
      statusflags.timesAwake++;
    }

    switch (statusflags.statusFlag)
    {
    case SupplyIsExcellent:

      digitalWrite(EN_LORA_PIN, HIGH);
      delay(200); // let the module turne on
      if (!manager.init())
        Serial.println("RFM96 init failed");
      // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
      driver.setFrequency(radioFrequency);
      //manager.setTxPower(20,false);
      if (statusflags.connectet)
      {
        broardcastTime();
      }
      else
      {
        rtc.disableAlarmInterrupt();
        rtc.disableCountdownTimer();
        while (!statusflags.connectet)
        {
          listenForTime();
        }
        rtc.enableAlarmInterrupt();
        rtc.enableCountdownTimer();
      }
      digitalWrite(EN_LORA_PIN, LOW);

      if (statusflags.gsmNotSent && HAS_GSM == true)
      {
        // Read saved data from EEPROM and send using GSM module
        statusflags.gsmNotSent = false;
      }
      break;
    case SupplyIsGood:
      digitalWrite(EN_LORA_PIN, HIGH);
      delay(200); // let the module turne on
      if (!manager.init())
        Serial.println("RFM96 init failed");
      // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
      driver.setFrequency(radioFrequency);
      //manager.setTxPower(20,false);
      if (statusflags.connectet)
      {
        broardcastTime();
      }
      else
      {
        rtc.disableAlarmInterrupt();
        rtc.disableCountdownTimer();
        while (!statusflags.connectet)
        {
          listenForTime();
        }
        rtc.enableAlarmInterrupt();
        rtc.enableCountdownTimer();
      }
      digitalWrite(EN_LORA_PIN, LOW);
      break;
    case SupplyIsModerate:
      if (!statusflags.connectet)
      {
        digitalWrite(EN_LORA_PIN, HIGH);
        delay(200); // let the module turne on
        if (!manager.init())
          Serial.println("RFM96 init failed");
        // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
        driver.setFrequency(radioFrequency);
        //manager.setTxPower(20,false);

        // Listen for time pings from other nodes
        rtc.disableAlarmInterrupt();
        rtc.disableCountdownTimer();
        while (!statusflags.connectet)
        {
          listenForTime();
        }
        rtc.enableAlarmInterrupt();
        rtc.enableCountdownTimer();
        digitalWrite(EN_LORA_PIN, LOW);
      }

      break;
    case SupplyIsBad:
      // maybe either make shure to check status more often to cach when things go bad,
      // or make shure to wake less often to save on power
      break;
    case SupplyIsTerrible:
      // save unsent data to EEPROM
      break;
    default:
      break;
    }
  }
  return;
}
bool runOnce = false;
void runOnAlarmInterrupt()
{
  if (!statusflags.alarmINT)
    return;

  if (!runOnce)
  {
    /**** initialize the LoRa module *****************/
    digitalWrite(EN_LORA_PIN, HIGH); // Enable the module
    delay(200);                      // let the module turne on
    if (!manager.init())
      Serial.println("RFM96 init failed");
    // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
    driver.setFrequency(radioFrequency);
    //manager.setTxPower(20,false);

    manager.clearRoutingTable();                      // clear routing table
    getRoutingTable(routingTableFirstAddr, &manager); // get routing table from EEPROM mem
    // start the countdown timer to set the syncronisation window
    runOnce = true;
    rtc.setCountdownTimer(windowTimerSettings.time, windowTimerSettings.unit, windowTimerSettings.repatMode);
    rtc.enableCountdownTimer();
  }
  // Check if this is first time after startup we are transmitting
  if (statusflags.justRestartet)
  {
    /*** check if data is saved in EEPROM
     * {
     *    // if it is read it to the transmit buffer
     *    // delete it from EEPROM
     * }else{
     *    // Nothing
     * }
     * 
     * */
    statusflags.justRestartet = false; // Now we dont need to know
  }
//check to see if this node has a GSM module, this changes how the node behaves
#if HAS_GSM == false
  sendMessage(&lpp, NODE3_ADDRESS); // NEEDS ANOTHER WAY TO DETERMINE end reserver
#endif
  listenForMessages();
  if (statusflags.windowEnd)
  {
    digitalWrite(EN_LORA_PIN, LOW);
    statusflags.alarmINT = false; // end this process
    statusflags.windowEnd = false;
    runOnce = false; // reset runonce
    #if HAS_GSM == true
    if(SupplyIsExcellent == statusflags.statusFlag)
    {
      // send data via GSM
      uint16_t len = dataBuf.curser;
      for (int i = len - 1; i >= 0; i--)
      {
        dataBuf.buf[dataBuf.curser] = 0;
        dataBuf.curser--;
      }
    }
    else
    {
      //save data in EEPROM
    }
    #endif
  }
}

//Dont put this on the stack:
uint8_t messageBuf[RH_MESH_MAX_MESSAGE_LEN];
// Listen for new messages
void listenForMessages()
{
  uint8_t len = sizeof(messageBuf);
  uint8_t from;
  if (manager.recvfromAck(messageBuf, &len, &from))
  {
    statusflags.connectet = true;
    /*  for debugging
    Serial.print("got request from : 0x");
    Serial.print(from, HEX);
    Serial.print(": ");
    Serial.println((char *)messageBuf);
    */
#if HAS_GSM == true
    for (int i = len - 1; i <= len; i++)
    { // coppy the data from message buffer to the data buffer

      dataBuf.buf[dataBuf.curser] = messageBuf[i];
      dataBuf.curser++;
    }
#endif
  }
}

void sendMessage(CayenneLPP *_lpp, uint8_t adr)
{
  //Serial.println("Sending to NODE3_ADDRESS"); // debugging
  if (0 < _lpp->getSize()) // is the buffer empty?
  {                      // if not send data
    // Send a message
    // A route to the destination will be automatically discovered.
    if (manager.sendtoWait(_lpp->getBuffer(), _lpp->getSize(), adr) == RH_ROUTER_ERROR_NONE)
    {
      statusflags.connectet = true; // indicate that the node connectet to the network
      _lpp->reset();
      /************************ debugging *****************************
    // It has been reliably delivered to the next node.
    // Now wait for a reply from the other node
    uint8_t len = sizeof(messageBuf);
    uint8_t from;
    if (manager.recvfromAckTimeout(messageBuf, &len, 3000, &from))
    {
      Serial.print("got reply from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char *)messageBuf);
    }
    else
    {
      Serial.println("No reply, are other nodes running?");
    }
    ************************************************************/
    }
    else
    {
      statusflags.connectet = false; // indicate that the node did not connect to the network
      //Serial.println("sendtoWait failed. Are the intermediate nodes running?"); // debugging
    }
  }
}

void broardcastTime()
{
  
  rtc.updateTime();
  messageBuf[0] = rtc.getSeconds();
  messageBuf[1] = rtc.getMinutes();
  messageBuf[2] = rtc.getHours();
  uint8_t len = sizeof(messageBuf);
  manager.sendtoWait(messageBuf, len, RH_BROADCAST_ADDRESS);
  return;
}

void listenForTime()
{
  uint8_t len = sizeof(messageBuf);
  uint8_t from;
  if (manager.recvfromAck(messageBuf, &len, &from))
  { 
    if(RH_BROADCAST_ADDRESS == from)
    {
      rtc.setSeconds(messageBuf[0]);
      rtc.setMinutes(messageBuf[1]);
      rtc.setHours(messageBuf[2]);
      statusflags.connectet = true;
    }
  }
  return;
}