#include "General_Purpose_Functions.h"

uint8_t messageBuf[RH_MESH_MAX_MESSAGE_LEN];

bufStruct dataBuf;

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
void updateSupplyStatus(statusflagsType *p, RV3028 *_rtc)
{
  analogRead(CAP_MEAS_PIN);                        // do a garbage read to make shure the users chosen reference is set in the ADMUX register
  p->vSupercap = /*measureUnregulatetVCC()*/ 5300; // done for testing pourposes!
  p->vcc = measureVCC(true);
  delay(300);

  if (VCAP_THRESHHOLD_EXCELLENT <= p->vSupercap)
  {
    p->statusFlag = SupplyIsExcellent;
  }
  else if (VCAP_THRESHHOLD_GOOD <= p->vSupercap)
  {
    p->statusFlag = SupplyIsGood;
  }
  else if (VCAP_THRESHHOLD_MODERATE <= p->vSupercap)
  {
    p->statusFlag = SupplyIsModerate;
  }
  else if (VCAP_THRESHHOLD_BAD <= p->vSupercap)
  {
    p->statusFlag = SupplyIsBad;
  }
  else if (VCAP_THRESHHOLD_TERRiBLE >= p->vSupercap || VCC_THRESHHOLD >= p->vcc)
  {
    p->statusFlag = SupplyIsTerrible;
  }
  _rtc->updateTime();
  p->tsSeconds = _rtc->getSeconds();
  p->tsMinutes = _rtc->getMinutes();
  p->tsHours = _rtc->getHours();

  return;
}

/***************
 * Returns a measurement of the unregulatet 
 * voltage sauce of the system in mV
 **************/
bool pinSetOutput = false;
uint16_t measureUnregulatetVCC()
{
  byte adcSettings = ADMUX; // copy ADC settings so as to reinsert them later // may have to be removed
  if (!pinSetOutput)
  { // the first time this is called make shure to set the pin as an output
    pinMode(CAP_MEAS_ON_OFF_PIN, OUTPUT);
    pinSetOutput = true;
  }
  digitalWrite(CAP_MEAS_ON_OFF_PIN, HIGH);
  analogReference(INTERNAL1V1);
  double result = analogRead(CAP_MEAS_PIN); // Do a garbage reading to make the change of reference happen
  delay(100);                               // wait for reference to settle
  result = analogRead(CAP_MEAS_PIN);        // useful read
  digitalWrite(CAP_MEAS_ON_OFF_PIN, LOW);

  result = (1100.0 / 1023.0) * result;
  result = (result * (CAP_MEAS_R1 + CAP_MEAS_R2)) / CAP_MEAS_R2;

  ADMUX = adcSettings;                           // Reinsert settings
  adcSettings = (adcSettings & 0b11000000) >> 6; // make the adc settings into the analog reference mode;
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
#ifdef DEBUGMODE
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

void listenForTime(RV3028 *_rtc, RHMesh *man, statusflagsType *status)
{

  uint8_t len = 3;
  uint8_t from;
  if (man->recvfromAck(messageBuf, &len, &from))
  {
#ifdef DEBUGMODE
    Serial.println("Recieved broadcast");
#endif

#ifdef DEBUGMODE
    Serial.println("recieved time");
    Serial.print(messageBuf[2]);
    Serial.print(":");
    Serial.print(messageBuf[1]);
    Serial.print(":");
    Serial.println(messageBuf[0]);
#endif
    _rtc->setSeconds(messageBuf[0]);
    _rtc->setMinutes(messageBuf[1]);
    _rtc->setHours(messageBuf[2]);
    status->connectet = true;
  }
  return;
}

void broardcastTime(RV3028 *_rtc, RHMesh *man)
{
  _rtc->updateTime();
  messageBuf[0] = _rtc->getSeconds();
  messageBuf[1] = _rtc->getMinutes();
  messageBuf[2] = _rtc->getHours();
  messageBuf[3] = 255;
  uint8_t len = 3;
  Serial.print("error: ");
  Serial.println(man->sendtoWait(messageBuf, len, RH_BROADCAST_ADDRESS));
#ifdef DEBUGMODE
  Serial.println("sending time: ");
  Serial.print(messageBuf[2]);
  Serial.print(":");
  Serial.print(messageBuf[1]);
  Serial.print(":");
  Serial.println(messageBuf[0]);
#endif
  return;
}
#ifdef DEBUGMODE
uint8_t testbuffer[RH_MESH_MAX_MESSAGE_LEN];
#endif
void sendMessage(CayenneLPP *_lpp, uint8_t adr, RHMesh *man, statusflagsType *status)
{

  if (0 < _lpp->getSize()) // is the buffer empty?
  {                        // if not send data
#ifdef DEBUGMODE
    Serial.print("Sending to adress: ");
    Serial.println(adr);
#endif
    // Send a message
    // A route to the destination will be automatically discovered.
    if (man->sendtoWait(_lpp->getBuffer(), _lpp->getSize(), adr) == RH_ROUTER_ERROR_NONE)
    {
      status->recievedAck = true; // indicate that the node connectet to the network
      _lpp->reset();
#ifdef DEBUGMODE
      // It has been reliably delivered to the next node.
      // Now wait for a reply from the other node

      uint8_t len = sizeof(testbuffer);
      uint8_t from;
      if (man->recvfromAckTimeout(testbuffer, &len, 3000, &from))
      {
        Serial.print("got reply from : 0x");
        Serial.print(from, HEX);
        Serial.print(": ");
        Serial.println((char *)testbuffer);
      }
      else
      {
        Serial.println("No reply, are other nodes running?");
      }
#endif
    }
    else
    {
#ifdef DEBUGMODE
      Serial.println("sendtoWait failed. Are the intermediate nodes running?");
#endif
    }
  }
}

// Listen for new messages
void listenForMessages(RV3028 *_rtc, RHMesh *man, statusflagsType *status)
{
  uint8_t len = sizeof(messageBuf);
  uint8_t from;
  if (man->recvfromAck(messageBuf, &len, &from))
  {
    status->recievedmsg = true;
#ifdef DEBUGMODE
    Serial.print("got request from : 0x");
    Serial.print(from, HEX);
    Serial.print(": ");
    Serial.println((char *)messageBuf);
    for (int i = 0; i < len; i++)
    {
      Serial.print(messageBuf[i]);
      Serial.print(":");
    }
    Serial.println();
#endif
    if (status->hasGSM)
    {
      for (int i = len - 1; i <= len; i++)
      { // coppy the data from message buffer to the data buffer

        dataBuf.buf[dataBuf.curser] = messageBuf[i];
        dataBuf.curser++;
      }
    }
  }
}

bool runOnce = false;

void runOnAlarmInterrupt(CayenneLPP *_lpp, RV3028 *_rtc, RHMesh *man, RH_RF95 *driv, statusflagsType *status, countdownTimerType *timSettings)
{
  if (!status->alarmINT)
    return;

  if (!runOnce)
  {
#ifdef DEBUGMODE
    Serial.println("Starting kommunications window");
#endif
    /**** initialize the LoRa module *****************/
    digitalWrite(EN_LORA_PIN, HIGH); // Enable the module
    delay(500);                      // let the module turne on
    if (!man->init())
    {
#ifdef DEBUGMODE
      Serial.println("RFM96 init failed");
#endif
    } // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
    driv->setFrequency(RADIO_FREQUENCY);
    //manager.setTxPower(20,false);

    man->clearRoutingTable();                    // clear routing table
    getRoutingTable(routingTableFirstAddr, man); // get routing table from EEPROM mem
    // start the countdown timer to set the syncronisation window
    runOnce = true;
    _rtc->setCountdownTimer(WINDOW_DURATION, UNIT_SECOND, false);
    _rtc->enableCountdownTimer();
  }
  // Check if this is first time after startup we are transmitting
  if (status->justRestartet)
  {
#ifdef DEBUGMODE
    Serial.println("Fisrt Window after restart");
#endif
    /*** check if data is saved in EEPROM
     * {
     *    // if it is read it to the transmit buffer
     *    // delete it from EEPROM
     * }else{
     *    // Nothing
     * }
     * 
     * */
    status->justRestartet = false; // Now we dont need to know
  }
  //check to see if this node has a GSM module, this changes how the node behaves
  if (!status->hasGSM)
  {
    while (!status->recievedAck)
    {
      sendMessage(_lpp, status->gsmNode, man, status); // NEEDS ANOTHER WAY TO DETERMINE end reserver
      if (status->recievedAck)
      {
        man->printRoutingTable();
        sendRoutingTable(routingTableFirstAddr, man, status->gsmNode);

        break;
      }
      else
      {
        delay(100); // Wait for a small time
      }
    }

    listenForMessages(_rtc, man, status);
  }
  else
  {
    listenForMessages(_rtc, man, status);
  }
  if (status->windowEnd)
  {
#ifdef DEBUGMODE
    Serial.println("Ending kommunications window");
#endif
    if (status->recievedmsg || status->recievedAck)
    {
      status->connectet = true;
      status->recievedmsg = false;
      status->recievedAck = false;
    }
    else
    {
      status->connectet = false;
    }
    digitalWrite(EN_LORA_PIN, LOW);
    status->alarmINT = false; // end this process
    status->windowEnd = false;
    runOnce = false; // reset runonce
    saveRoutingTable(routingTableFirstAddr, man);
    if (status->hasGSM)
    {
      if (SupplyIsExcellent == status->statusFlag) // if supply is optimal
      {
        // send data via GSM
#ifdef DEBUGMODE
        Serial.println("sending data Via GSM");
#endif
        // after sending delete data buffer
        uint16_t len = dataBuf.curser;
        for (int i = len - 1; i >= 0; i--)
        {
          dataBuf.buf[dataBuf.curser] = 0;
          dataBuf.curser--;
        }
        status->gsmNotSent = false;
      }
      else
      {
        //save data in EEPROM
#ifdef DEBUGMODE
        Serial.println("Save data buffer in EEPROM");
#endif
        status->gsmNotSent = true;
      }
    }
    else
    {
      _lpp->addAnalogInput(0, 3.3330);
      _lpp->addAnalogInput(1, 2.4);
    }
    // enable the timer with the users settings, as we have been repourposing it here
    _rtc->enableAlarmInterrupt(3, 12, 1, false, 4);
    _rtc->setCountdownTimer(timSettings->time, timSettings->unit, timSettings->repatMode);
    _rtc->enableCountdownTimer();
  }
}

void runOnTimerInterrupt(CayenneLPP *_lpp, RV3028 *_rtc, RHMesh *man, RH_RF95 *driv, statusflagsType *status, countdownTimerType *timSetting)
{
  if (!status->timerINT)
    return;
  //only run once per timer interrupt
  status->timerINT = false;
  // see if the timer time is very low, if it is, then do not count them as waking
  if ((10 <= timSetting->time && UNIT_SECOND == timSetting->unit) || (10000 <= timSetting->time && UNIT_M_SECOND == timSetting->unit))
  {
    /****************************************
     * Check status of VCC and UnregulatetVCC
     * if it is time for this
     ***************************************/
    if (WAKE_TIMES_BEFORE_STATUS_CHECK <= status->timesAwake)
    {

      updateSupplyStatus(status, _rtc);
      status->timesAwake = 0;
#ifdef DEBUGMODE
      Serial.print("Status updatet: ");
      Serial.println(status->statusFlag);
#endif
    }
    else
    {
      status->timesAwake++;
    }

    switch (status->statusFlag)
    {
    case SupplyIsExcellent:

      digitalWrite(EN_LORA_PIN, HIGH);
      delay(200); // let the module turne on
      if (!man->init())
      {
#ifdef DEBUGMODE
        Serial.println("RFM96 init failed");
#endif
      }

      // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
      driv->setFrequency(RADIO_FREQUENCY);
      //manager.setTxPower(20,false);
      if (status->connectet)
      {
#ifdef DEBUGMODE
        Serial.println("broadcasting time");
#endif
        broardcastTime(_rtc, man);
      }
      else
      {
#ifdef DEBUGMODE
        Serial.println("listening for time");
#endif
        _rtc->disableAlarmInterrupt();
        _rtc->disableCountdownTimer();
        while (!status->connectet)
        {
          listenForTime(_rtc, man, status);
        }
        _rtc->enableAlarmInterrupt();
        _rtc->enableCountdownTimer();
      }
      digitalWrite(EN_LORA_PIN, LOW);

      if (status->gsmNotSent && status->hasGSM)
      {
#ifdef DEBUGMODE
        Serial.println("Sending data with GSM");
#endif
        // Read saved data from EEPROM and send using GSM module
        status->gsmNotSent = false;
      }
      break;
    case SupplyIsGood:
      digitalWrite(EN_LORA_PIN, HIGH);
      delay(200); // let the module turn on
      if (!man->init())
      {
#ifdef DEBUGMODE
        Serial.println("RFM96 init failed");
#endif
      }
      // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
      driv->setFrequency(RADIO_FREQUENCY);
      //manager.setTxPower(20,false);
      if (status->connectet)
      {
        broardcastTime(_rtc, man);
#ifdef DEBUGMODE
        Serial.println("broadcasting time");
#endif
      }
      else
      {
#ifdef DEBUGMODE
        Serial.println("listening for time");
#endif
        _rtc->disableAlarmInterrupt();
        _rtc->disableCountdownTimer();
        while (!status->connectet)
        {
          listenForTime(_rtc, man, status);
        }
        _rtc->enableAlarmInterrupt();
        _rtc->enableCountdownTimer();
      }
      digitalWrite(EN_LORA_PIN, LOW);
      break;
    case SupplyIsModerate:
      if (!status->connectet)
      {
        digitalWrite(EN_LORA_PIN, HIGH);
        delay(200); // let the module turne on
        if (!man->init())
        {
#ifdef DEBUGMODE
          Serial.println("RFM96 init failed");
#endif
          // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
          driv->setFrequency(RADIO_FREQUENCY);
          //manager.setTxPower(20,false);

          // Listen for time pings from other nodes
#ifdef DEBUGMODE
          Serial.println("listening for time");
#endif
          _rtc->disableAlarmInterrupt();
          _rtc->disableCountdownTimer();
          while (!status->connectet)
          {
            listenForTime(_rtc, man, status);
          }
          _rtc->enableAlarmInterrupt();
          _rtc->enableCountdownTimer();
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
}

void sendRoutingTable(int row_addr, RHMesh *man, uint8_t adr)
{
  // This functions gets the most recent routing table saved in EEPROM memory.

  // Local initializing of relative address incrementers.
  uint8_t dest_addr = 0;
  uint8_t next_hop_addr = 1;
  uint8_t state_addr = 2;
  uint8_t next_row = 3;
  uint8_t empty_addr = 0xFF;
  int first_addr = row_addr;
  uint8_t buf[RH_ROUTING_TABLE_SIZE * next_row];
  // Local declaration of table (dest, next_hop, state), so EEPROM only has to be read once per address by instantly saving the read values.
  uint8_t dest, next_hop, state;

  while (((dest = EEPROM.read(row_addr + dest_addr)) & (next_hop = EEPROM.read(row_addr + next_hop_addr)) &
          (state = EEPROM.read(row_addr + state_addr))) != empty_addr)
  { // Read all addresses for the row of the routing table (dest, next_hop, state). If all contains 0xFF, then EEPROM addresses is empty
    if (row_addr > (first_addr + (RH_ROUTING_TABLE_SIZE * next_row)))
    { // Give error if routing table exceeds the default routing table size defined in RHRouter.h
#ifdef DEBUGMODE
      Serial.print("Error: Routing table in EEPRROM exceeds maximum size of ");
      Serial.print(RH_ROUTING_TABLE_SIZE);
      Serial.println(" entries.");
#endif
      break;
    }
    buf[row_addr + dest_addr] = dest; //
    buf[row_addr + next_hop_addr] = next_hop;
    buf[row_addr + state_addr] = state;
    row_addr = row_addr + next_row; // Get next rows address
  }
  uint8_t len = sizeof(buf);
  if (man->sendtoWait(buf, len, adr) == RH_ROUTER_ERROR_NONE)
  {

#ifdef DEBUGMODE
    // It has been reliably delivered to the next node.
    // Now wait for a reply from the other node

    uint8_t len = sizeof(testbuffer);
    uint8_t from;
    if (man->recvfromAckTimeout(testbuffer, &len, 3000, &from))
    {
      Serial.print("got reply from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char *)testbuffer);
    }
    else
    {
      Serial.println("No reply, are other nodes running?");
    }
#endif
  }
  else
  {
#ifdef DEBUGMODE
    Serial.println("sendtoWait failed. Are the intermediate nodes running?");
#endif
  }
}
// Function sends
void sendGSMData(const uint8_t *payload, uint8_t payloadSize)
{

  // Software serial object to communicate with SIM800L
  SoftwareSerial mySerial(TXSOFTSERIAL, RXSOFTSERIAL);

  digitalWrite(SWITCH_GSM, HIGH); // Set GSM load switch in ON state
  delay(7500);                    // Wait for module to be initialised and ready

  mySerial.begin(115200);
  delay(1000); // It takes time to begin serial

  //mySerial.println("AT+IPR=115200"); // Save fixed baud on non volatile memory. SIM800L will now give 'RDY' when initialised on start up. Can be used instead of delay.
  //mySerial.println("AT");  //Handshake
  //updateSerial(mySerial);

  // Text mode:
  mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial(mySerial);
  mySerial.print("AT+CMGS=\"+"); // Enter number with country code.
  mySerial.print(PHONENUMBERCOUNTRYCODE);
  mySerial.print(PHONENUMBER);
  mySerial.println("\"");
  updateSerial(mySerial);
  //lpp.decode(lpp.getBuffer(),lpp.getSize(),root);  // Decodes lpp buffer and saves it in the reserved memory 'JsonArray root'.
  //serializeJsonPretty(root,mySerial);  // Serializes/creates a JSON document with spaces and line-breaks between values.
  mySerial.write(payload, payloadSize); // Message text content. Maximum of 160 characters. ÅÆØ and other special characters will not work without a different encoding

  // PDU mode (binary mode - not implemented)
  // Reference: https://www.developershome.com/sms/cmgsCommand6.asp
  //  mySerial.println("AT+CMGF=0");  // Configuring PDU mode
  //  updateSerial();
  //  mySerial.println("AT+CMGS=17");
  //  updateSerial();
  //  mySerial.print("0691540493909911000A9154064262040004AA0454657374"); // Sending to +45 60 24 26 40
  //  // Used http://www.twit88.com/home/utility/sms-pdu-encode-decode
  //  mySerial.write(26);  // 26 = 'Ctrl+z character (ASCII)'
  //  updateSerial();

  mySerial.write(26); // 26 = 'Ctrl+z character (ASCII)' -> send
  delay(100);         // wait for data to be send
  //mySerial.flush(); den kan vi bruge i stedet for delay
  mySerial.end();
  digitalWrite(SWITCH_GSM, LOW); // Set load switch in OFF state
}

void updateSerial(SoftwareSerial mySerial)
{
  delay(500);
  while (Serial.available())
  {
    mySerial.write(Serial.read()); //Forward what Serial received to Software Serial Port
  }
  while (mySerial.available())
  {
    Serial.write(mySerial.read()); //Forward what Software Serial received to Serial Port
  }
}