// Construct if prevent issues of someone #include 's this file twice
#ifndef GEMT_TestSuite_h
#define GEMT_TestSuite_h

#include "Arduino.h"

bool GEMT_test(String moduleID);
bool nRF24_test(void);
bool ultrasonicsensor_test(void);
void servoManual_test(void);
double ESR_test(void);
long readVcc(void);
unsigned long measureESR(void);

#endif
