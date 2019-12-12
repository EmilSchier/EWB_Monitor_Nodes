#include "RV-3028-C7.h"
#include <Wire.h>

// defines the time and date to be set
/*
#define SECONDS 0
#define MINUTES 0
#define HOURS 12
#define WEEKDAY 0
#define DATE 1
#define MONTH 1
#define YEAR 2019
*/
RV3028 rtc;
void setup()
{
    
    Serial.begin(9600);
    Wire.begin();
    rtc.begin();
    if (rtc.setToCompilerTime())
    {
        Serial.println("time set");
    }
    rtc.writeRegister(RV3028_CTRL1,0);
    rtc.writeRegister(RV3028_CTRL2,0);
    rtc.clearInterrupts();

    Serial.println("Setup of RTC time is done,");
    Serial.println("now dont disconnect the 3V coin cell battery");
    Serial.println("and upload the main sketch you want this unit to run");
}

void loop()
{

}