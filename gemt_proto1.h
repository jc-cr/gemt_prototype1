#ifndef gemt_proto1_h
#define gemt_proto1_h

#include "Arduino.h"

//========================================================================
// Helper functions
//========================================================================

// Function to print horizontal line of a char
void printHline(char lineChar)
{
  for (int i = 0; i < 72; ++i)
  {
    Serial.print(lineChar);
  }

  Serial.println();
}

// Function to read user serial input
// Reads int > 0
// Adapted from https://forum.arduino.cc/t/serial-input-basics-updated/382007
short int getSerialInput_int(void)
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
      dataAvailable = 1;

      /*
      // Use this to ignore new line or carrige readings readings
      if (input == 0)
      {
        dataAvailable = 0;
      }
      else
      {
        dataAvailable = 1; // exit loop with selection int
      }   
      */
    }
  }
  return input;  
}

// Function to read user serial input
// Reads single char
char getSerialInput_char(void)
{
  char input = 0;
  bool dataAvailable = 0;

  while (dataAvailable == 0)
  {
    if (Serial.available() > 0) {
         dataAvailable = 1;
        input = Serial.read();      
    }
  }
  return input; 
}

//========================================================================
// Screen Display functions
//========================================================================

// Function to get user menu selection from serial monitor input
// menuName - Name for the desired menu (Main, Sub, Servo, etc.)
// menuOptions - List of options in your testing suite
// menuArraySize - number of elements in menuOptions
unsigned short int menuSelection(String menuName, const char* menuOptions[], size_t menuArraySize)
{
  unsigned short int selection;
  char buffer[50]; // init buffer of 50 bytes to hold expected string size

  printHline('*');
  
  Serial.println(menuName);

  // Assign item number to menuOption. Starts at 1.
  for (size_t i = 1; i <= menuArraySize; ++i)
  {    
    sprintf(buffer, "%d. %s", i, menuOptions[i-1]);
    Serial.println(buffer);
  }

  printHline('*');

  Serial.println("Type item number of desired test: \n"); 
  printHline('*');

  selection = getSerialInput_int();
  
  // Auto catch any invalid menu selection parameters
  // From getSerialInput_int() we assume 0 is not an option
  while (selection > menuArraySize || selection <= 0)
  {
    selection = getSerialInput_int();
  }
    
  // Return user selection input
  return selection; 
}

// Displays the instructions to a test (pins to connect to, etc.)
// bool return determines if test will proceed or go back to previous screen
bool infoScreen (String infoMsg)
{
  bool proceed = 0;
  unsigned short int selection;
  
  printHline('-');
  Serial.println("Test Info Screen: ");
  Serial.println(infoMsg);
  printHline('-');
  
  Serial.println("1. OK");
  Serial.println("2. BACK");
  printHline('-');
  Serial.println("Type item number of desired action:");
  printHline('-');

  selection = getSerialInput_int();
  
  // Loop until we get correct input
  while (selection != 1 && selection != 2)
  {
    Serial.print(selection); Serial.print(" is an invalid input! \n");
    Serial.println("Please try enter 1 or 2");
    selection = getSerialInput_int();
  }

  // Return param to continue to next screen
  if (selection == 1)
  {
    proceed = true;
  }
  // Return param to return to previous screen
  else if (selection == 2)
  {
    proceed = false;
  }

  return proceed;
}

#endif