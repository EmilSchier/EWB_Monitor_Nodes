#if !defined(GENERALP_FUNCTION)
#define GENERALP_FUNCTION

// Includes
#include <Arduino.h>
#include <EEPROM.h>
#include "RHMesh.h"
#include "RH_RF95.h"
#include <SoftwareSerial.h>

// Defines
#define routingTableFirstAddr  0

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
#define STATJSONBUFFERSIZE 512  // use https://arduinojson.org/v5/assistant/ to calculate the proper size

// Load switch pins
#define SWITCH_GSM 15
#define _SS_MAX_RX_BUFF 256  // Expand Software Serial buffer to 256 characters. Default is 64. 

//Functions 

// Measures VCC and returns measured value
// arguments:
//      bool in_mV      : determines if retuned value is in mV or V.
// returns the measured VCC value.
double measureVCC(bool in_mV);

// This functions gets the most recent routing table saved in EEPROM memory 
// and saves it in the manager pointet to by ptrManager.
// Arguments: 
//          int row_addr        : Starting adress of the routing table in EEPROM
//          RHMesh *ptrManager  : Pointer to the manager to where the 
//                                routing table should be loaded
void getRoutingTable(int row_addr,RHMesh *ptrManager);

// Saves the current routing table to EEPROM
// Arguments: 
//          int row_addr        : Starting adress of the routing table in EEPROM
//          RHMesh *ptrManager  : Pointer to the manager from witch to copy the 
//                                routing table
void saveRoutingTable(int row_addr, RHMesh *ptrManager);

// deletes the saved routing table from EEPROM
void deleteRoutingTableEEPROM(int row_addr);

// Sends data using SIM800L module
void sendGSMData(const uint8_t *payload, uint8_t payloadSize);

// Forwards received Serial data to Software Serial Port and vice versa. 
// Reference: https://lastminuteengineers.com/sim800l-gsm-module-arduino-tutorial/ 
void updateSerial(SoftwareSerial mySerial);

#endif // GENERALP_FUNCTION
