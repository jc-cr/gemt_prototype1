#ifndef gemt_proto1_h
#define gemt_proto1_h

#include "Arduino.h"


// Function to read user serial input
// Reads int > 0
unsigned short int getSerialInput_int(void)
{
  bool dataAvailable = 0;
  unsigned short int input;

  // Loop until we recieve data
  while(dataAvailable == 0)
  {
    // Read input if data is available
    if (Serial.available() > 0) 
    {
      input = Serial.parseInt();

      // Ignore new line or carrige readings readings
      // FIXME: Shitty thing is, now I can't grab 0 int! May change later
      if (input == 0)
      {
        dataAvailable = 0;
      }
      else
      {
        dataAvailable = 1; // exit loop with selection int
      }   
    }
  }

  return input;  
}

// Function to get user menu selection from serial monitor input
unsigned short int menuSelection(void)
{
  unsigned short int selection;
  
  Serial.println();
  Serial.println("********************************************************");
  Serial.println("MENU:");;
  Serial.println("1. nRF24 Test");
  Serial.println("2. Example 2");
  Serial.println("3. Example 3");
  Serial.println("********************************************************");
  Serial.println("Type item number of desired test: ");
  
  return selection = getSerialInput_int();
}


// Displays the instructions to a test (pins to connect to, etc.)
// bool return determines if test will proceed or go back to previous screen
bool infoScreen (String infoMsg)
{
  bool proceed = 0;
  unsigned short int selection;
  
  Serial.println();
  Serial.println("-------------------------------------------------------");
  Serial.println(infoMsg);
  Serial.println("1. OK");
  Serial.println("2. BACK");
  Serial.println("-------------------------------------------------------");
  Serial.println("Type item number of desired action: ");

  selection = getSerialInput_int();
  // Loop until we get correct input
  while (selection != (1 || 2))
  {
    Serial.print(selection); Serial.print(" is an invalid input! \n");
    Serial.println("Please try enter 1 or 2");
    selection = getSerialInput_int();
  }

  if (selection == 1)
  {
    proceed = true;
  }
  else if (selection == 2)
  {
    proceed = false;
  }


  return proceed;
}


#endif