#include "General_Purpose_Functions.h"

double measureVCC(bool in_mV)
{
    byte adcSettings = ADMUX; // copy ADC settings so as to reinsert them later
    ADMUX = 0;                // reset ADC settings

// Set the AVCC as reference
#if defined(__AVR_ATmega1284P__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATmega328P__)
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

    delay(2);            // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA, ADSC))
        ; // measuring

    uint8_t low = ADCL;  // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    double result = (high << 8) | low;
    if (in_mV)
    {
        result = (1.1 * 1023.0 * 1000.0) / result;
    }
    else
    {
        result = (1.1 * 1023.0) / result;
    }
    ADMUX = adcSettings;

    return result;
}

/***************
 * Updates the status of the voltage suply and determines if it is in good shape or not
 * 
 **************/
void updateSupplyStatus(supplyStatusStruct *p)
{
    analogRead(CAP_MEAS_PIN); // do a garbage read to make shure the users chosen reference is set in the ADMUX register
    p->vSupercap = measureUnregulatetVCC();
    p->vcc = measureVCC(true);
    delay(300);
    
    if(VCAP_THRESHHOLD_EXCELLENT <= p->vSupercap){
        p->statusFlag = SupplyIsExcellent;
    }else if(VCAP_THRESHHOLD_GOOD <= p->vSupercap){
        p->statusFlag = SupplyIsGood;
    }else if(VCAP_THRESHHOLD_MODERATE <= p->vSupercap){
        p->statusFlag = SupplyIsModerate;
    }else if(VCAP_THRESHHOLD_BAD <= p->vSupercap){
        p->statusFlag = SupplyIsBad;
    }else if(VCAP_THRESHHOLD_TERREBLE >= p->vSupercap || VCC_THRESHHOLD >= p->vcc){
        p->statusFlag = SupplyIsTerreble;
    }
}

/***************
 * Returns a measurement of the unregulatet 
 * voltage sauce of the system in mV
 **************/
bool pinSetOutput = false;
uint16_t measureUnregulatetVCC()
{
     byte adcSettings = ADMUX; // copy ADC settings so as to reinsert them later // may have to be removed
    if(!pinSetOutput){ // the first time this is called make shure to set the pin as an output
        pinMode(CAP_MEAS_ON_OFF_PIN,OUTPUT);
        pinSetOutput = true;
    }
    analogReference(INTERNAL1V1);
    double result = analogRead(CAP_MEAS_PIN); // Do a garbage reading to make the change of reference happen 
    delay(100);// wait for reference to settle
    result =  analogRead(CAP_MEAS_PIN); // useful read

    result = (1100.0/1023.0)*result; 
    result = (result*(CAP_MEAS_R1+CAP_MEAS_R2))/CAP_MEAS_R2;

    ADMUX = adcSettings; // Reinsert settings
    adcSettings = (adcSettings & 0b11000000)>>6; // make the adc settings into the analog reference mode;
    // set the analog reference to what it was before, this has to be done in case another setting is used
    // somewhere else and because of the way arduino does analog reads.
    analogReference(adcSettings); 
    delay(100); // delay to make the reference settle again, in case an analog read is some of the nect code to be run
    return result;
}

void getRoutingTable(int row_addr, RHMesh *ptrManager)
{
    // This functions gets the most recent routing table saved in EEPROM memory.

    // Local initializing of relative address incrementers.
    uint8_t dest_addr = 0;
    uint8_t next_hop_addr = 1;
    uint8_t state_addr = 2;
    uint8_t next_row = 3;
    uint8_t empty_addr = 0xFF;
    int first_addr = row_addr;

    // Local declaration of table (dest, next_hop, state), so EEPROM only has to be read once per address by instantly saving the read values.
    uint8_t dest, next_hop, state;

    while (((dest = EEPROM.read(row_addr + dest_addr)) & (next_hop = EEPROM.read(row_addr + next_hop_addr)) &
            (state = EEPROM.read(row_addr + state_addr))) != empty_addr)
    { // Read all addresses for the row of the routing table (dest, next_hop, state). If all contains 0xFF, then EEPROM addresses is empty
        if (row_addr > (first_addr + (RH_ROUTING_TABLE_SIZE * next_row)))
        { // Give error if routing table exceeds the default routing table size defined in RHRouter.h
#ifdef debug
            Serial.print("Error: Routing table in EEPRROM exceeds maximum size of ");
            Serial.print(RH_ROUTING_TABLE_SIZE);
            Serial.println(" entries.");
#endif
            break;
        }
        ptrManager->addRouteTo(dest, next_hop, state); // Add entry (dest, next_hop, state) to the routing table
        row_addr = row_addr + next_row;                // Get next rows address
    }
}

void saveRoutingTable(int row_addr, RHMesh *ptrManager)
{
    /********************************************************
A member function returning _routes must be added.
 
Add the following line(s) to RHRouter.h under public:
// Allocates memory on heap, adds _routes and returns a pointer to the memory containing the copy. 
RoutingTableEntry* getRoutingTable();

Add the following lines to RHRouter.cpp under public:
RHRouter::RoutingTableEntry* RHRouter::getRoutingTable()
{
 RHRouter::RoutingTableEntry* _routesPtr = (RHRouter::RoutingTableEntry*)calloc(3, sizeof(RHRouter::RoutingTableEntry));
    uint8_t i;
  for (i = 0; i < RH_ROUTING_TABLE_SIZE; i++)
  {
    _routesPtr[i] = _routes[i];
  }
  return _routesPtr;
}
*********************************************************/

    uint8_t dest_addr = 0;
    uint8_t next_hop_addr = 1;
    uint8_t state_addr = 2;
    uint8_t next_row = 3;

    for (int i = 0; i < (RH_ROUTING_TABLE_SIZE * next_row); i++)
    { // Clear all possibly existing routing table EEPROM addresses
        EEPROM.write((row_addr + i), 0);
    }

    RHRouter::RoutingTableEntry *tempRoutingTable = ptrManager->getRoutingTable(); // Gets pointer to a copy of the routing table in heap
    uint8_t e = 0;
    for (int i = 0; i < (RH_ROUTING_TABLE_SIZE * next_row); i = i + next_row)
    {
        if (tempRoutingTable[e].dest != 0 || tempRoutingTable[e].next_hop != 0 || tempRoutingTable[e].state != 0)
        {
            EEPROM.write((i + row_addr + dest_addr), tempRoutingTable[e].dest);
            EEPROM.write((i + row_addr + next_hop_addr), tempRoutingTable[e].next_hop);
            EEPROM.write((i + row_addr + state_addr), tempRoutingTable[e].state);
            e++;
        }
        else
            break;
    }
    free(tempRoutingTable);

    /*
   * Alternative version: make _routes public instead of adding a member function to return it. 
  uint8_t e = 0;
  for (int i = 0 ; i < (RH_ROUTING_TABLE_SIZE*next_row) ; i = i+next_row) {
    if(manager._routes[e].dest != 0 || manager._routes[e].next_hop != 0 || manager._routes[e].state != 0) {
      EEPROM.write((i+row_addr+dest_addr),manager._routes[e].dest);
      EEPROM.write((i+row_addr+next_hop_addr),manager._routes[e].next_hop);
      EEPROM.write((i+row_addr+state_addr),manager._routes[e].state);
      e++;
    }
    else
      break;
  }
  */

    /*  // Jes: et forsøg på at omgå at _routes[] er private, da der ikke findes en member function til at tilgå den. 
  RoutingTableEntry* ptr = (RoutingTableEntry*)&manager._routes;
  RoutingTableEntry entry0 = *ptr;
  //RoutingTableEntry entry1 = *(ptr+1);
  Serial.println("*ptr / entry0: ");
  Serial.println(entry0.dest);
  Serial.println(entry0.next_hop);
  Serial.println(entry0.state); */
}

void deleteRoutingTableEEPROM(int row_addr)
{
    uint8_t next_row = 3;

    for (int i = 0; i < (RH_ROUTING_TABLE_SIZE * next_row); i++)
    { // Clear all possibly existing routing table EEPROM addresses
        EEPROM.write((row_addr + i), 0);
    }
}
