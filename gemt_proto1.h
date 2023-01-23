#ifndef gemt_proto1_h
#define gemt_proto1_h

#include "Arduino.h"

// Function to get user menu selection from serial monitor input
unsigned short int menuSelection()
{
  unsigned short int selection;
  bool dataAvailable = 0;

  Serial.print(" MENU:\n");;
  Serial.print("1. nRF24 Test\n");
  Serial.print("2. Example 2\n");
  Serial.print("3. Example 3\n");

  Serial.println("Type item number of desired test: ");

  // Loop until we recieve data
  while(dataAvailable == 0)
  {
    // Read input if data is available
    if (Serial.available() > 0) 
    {
      selection = Serial.parseInt();

      // Ignore new line or carrige readings readings
      if (selection == 0)
      {
        dataAvailable = 0;
      }
      else
      {
        dataAvailable = 1; // exit loop with selection int
      }   
    }
  }

  return selection;
}


#endif