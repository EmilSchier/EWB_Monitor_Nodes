#if !defined(GENERALP_FUNCTION)
#define GENERALP_FUNCTION

// Includes
#include <Arduino.h>
#include <EEPROM.h>
#include "RHMesh.h"
#include "RH_RF95.h"

// Defines
#define routingTableFirstAddr  0

// Define each node address
#define NODE1_ADDRESS 1
#define NODE2_ADDRESS 2
#define NODE3_ADDRESS 3
#define NODE4_ADDRESS 4

//#define RH_MESH_MAX_MESSAGE_LEN 50
#define SENSORCH0 0
#define SENSORCH1 1
#define PAYLOADMAXSIZE 51
#define DYNJSONBUFFERSIZE 4096
#define STATJSONBUFFERSIZE 512  // use https://arduinojson.org/v5/assistant/ to calculate the proper size

//Pin definitions
#define CAP_MEAS_PIN        24
#define CAP_MEAS_ON_OFF_PIN 25

// Definitions for Supply status function
#define CAP_MEAS_R1                 10000 // R1 of the voltage devider for measuring Capacitor voltage
#define CAP_MEAS_R2                 2325 // R2 of the voltage devider for measuring Capacitor voltage
#define VCAP_THRESHHOLD_EXCELLENT   5000 // 
#define VCAP_THRESHHOLD_GOOD        4700 //
#define VCAP_THRESHHOLD_MODERATE    4400 //
#define VCAP_THRESHHOLD_BAD         4000 //
#define VCAP_THRESHHOLD_TERREBLE    3500 //
#define VCC_THRESHHOLD              2000 // 

enum supplyStatusFlag{
    SupplyIsExcellent,
    SupplyIsGood,
    SupplyIsModerate,
    SupplyIsBad,
    SupplyIsTerreble,
    };

typedef struct supplyStatusStruct
{
    uint16_t vcc;
    uint16_t vSupercap;
    enum supplyStatusFlag statusFlag;
};

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


#endif // GENERALP_FUNCTION
