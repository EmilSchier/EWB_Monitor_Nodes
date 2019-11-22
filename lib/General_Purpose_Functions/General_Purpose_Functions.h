#if !defined(GENERALP_FUNCTION)
#define GENERALP_FUNCTION

// Includes
#include <Arduino.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <EEPROM.h>

// Defines
#define routingTableFirstAddr  0



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
