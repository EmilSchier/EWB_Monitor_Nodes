#if !defined(GENERALP_FUNCTION)
#define GENERALP_FUNCTION

// Includes
#include <Arduino.h>
#include <EEPROM.h>
#include "RHMesh.h"
#include "RH_RF95.h"
#include <CayenneLPP.h>
#include "RV-3028-C7.h"
#include <SoftwareSerial.h>

#define DEBUGMODE // defien this to enter debug mode with seriel readouts

// Defines
#define routingTableFirstAddr 0

// pin definitions
#define RTC_INTERRUPT_PIN 10
#define EN_LORA_PIN 3

// Define each node address
#define NODE1_ADDRESS 1
#define NODE2_ADDRESS 2
#define NODE3_ADDRESS 3
#define NODE4_ADDRESS 4

// Phone number to send data to. Add country code without '+' character. For example '45' for Denmark.
#define PHONENUMBERCOUNTRYCODE 45
#define PHONENUMBER 60242640
#define TXSOFTSERIAL 19
#define RXSOFTSERIAL 20
#define DTRPIN 18

//#define RH_MESH_MAX_MESSAGE_LEN 50
#define SENSORCH0 0
#define SENSORCH1 1
#define PAYLOADMAXSIZE 51
#define DYNJSONBUFFERSIZE 4096
#define STATJSONBUFFERSIZE 512 // use https://arduinojson.org/v5/assistant/ to calculate the proper size
#define RADIO_FREQUENCY 868.0

//Pin definitions
#define CAP_MEAS_PIN A0
#define CAP_MEAS_ON_OFF_PIN 25

// Definitions for Supply status function
#define CAP_MEAS_R1 9802               // R1 of the voltage devider for measuring Capacitor voltage
#define CAP_MEAS_R2 2390               // R2 of the voltage devider for measuring Capacitor voltage
#define VCAP_THRESHHOLD_EXCELLENT 5000 //
#define VCAP_THRESHHOLD_GOOD 4700      //
#define VCAP_THRESHHOLD_MODERATE 4400  //
#define VCAP_THRESHHOLD_BAD 4000       //
#define VCAP_THRESHHOLD_TERRiBLE 3500  //
#define VCC_THRESHHOLD 2000            //

#define WINDOW_DURATION 10 //
#define SEND_TRIES 3       // the numpber of times it is atemptet tosendt the message if at first it does not succed
// The number of times the the processer has to have been woken before
// checking the status af VCC and Unregulatet VCC
#define WAKE_TIMES_BEFORE_STATUS_CHECK 5 //Might need a new name

#define SWITCH_GSM 15
#define _SS_MAX_RX_BUFF 256 // Expand Software Serial buffer to 256 characters. Default is 64.
enum supplyStatusFlag
{
  SupplyIsExcellent,
  SupplyIsGood,
  SupplyIsModerate,
  SupplyIsBad,
  SupplyIsTerrible,
};

struct statusflagsType
{
  bool connectet;
  bool recievedmsg;
  bool recievedAck;
  bool gsmNotSent;
  bool alarmINT, timerINT, windowEnd;
  bool gotosleep;
  bool justRestartet;
  bool hasGSM;
  double vcc;
  double vSupercap;
  enum supplyStatusFlag statusFlag;
  uint8_t timesAwake;
  uint8_t tsSeconds, tsMinutes, tsHours;
  uint8_t gsmNode;
  uint8_t ownAdress;
};
struct bufStruct
{
  uint8_t buf[RH_MESH_MAX_MESSAGE_LEN * 20];
  uint16_t curser;
};
// Load switch pins

//Functions

// Measures VCC and returns measured value
// arguments:
//      bool in_mV      : determines if retuned value is in mV or V.
// returns the measured VCC value.
double measureVCC(bool in_mV);
void updateSupplyStatus(statusflagsType *p, RV3028 *_rtc);
uint16_t measureUnregulatetVCC();
// This functions gets the most recent routing table saved in EEPROM memory
// and saves it in the manager pointet to by ptrManager.
// Arguments:
//          int row_addr        : Starting adress of the routing table in EEPROM
//          RHMesh *ptrManager  : Pointer to the manager to where the
//                                routing table should be loaded
void getRoutingTable(int row_addr, RHMesh *ptrManager);

// Saves the current routing table to EEPROM
// Arguments:
//          int row_addr        : Starting adress of the routing table in EEPROM
//          RHMesh *ptrManager  : Pointer to the manager from witch to copy the
//                                routing table
void saveRoutingTable(int row_addr, RHMesh *ptrManager);

// deletes the saved routing table from EEPROM
void deleteRoutingTableEEPROM(int row_addr);

void listenForTime(RV3028 *_rtc, RHMesh *man, statusflagsType *status);
void broardcastTime(RV3028 *_rtc, RHMesh *man);
void sendMessage(CayenneLPP *_lpp, uint8_t adr, RHMesh *man, statusflagsType *status);

void listenForMessages(RV3028 *_rtc, RHMesh *man, statusflagsType *status);
void runOnAlarmInterrupt(CayenneLPP *_lpp, RV3028 *_rtc, RHMesh *man, RH_RF95 *driv, statusflagsType *status, countdownTimerType *timSettings);
void runOnTimerInterrupt(CayenneLPP *_lpp, RV3028 *_rtc, RHMesh *man, RH_RF95 *driv, statusflagsType *status, countdownTimerType *timSetting);
// Sends data using SIM800L module
void sendGSMData(const uint8_t *payload, uint8_t payloadSize);

// Forwards received Serial data to Software Serial Port and vice versa.
// Reference: https://lastminuteengineers.com/sim800l-gsm-module-arduino-tutorial/
void updateSerial(SoftwareSerial mySerial);

void sendRoutingTable(int row_addr, RHMesh *man, uint8_t adr);
#endif // GENERALP_FUNCTION
