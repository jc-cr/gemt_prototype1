#include "Arduino.h"
#include "gemt_proto1.h"

void setup (void)
{
  delay(500);
  Serial.begin(9600);
}

int main (void)
{
  // Must intialize the arduino firmware
  init();
  setup();

  // Init array of pointers for menus
  const char* mainMenu[] = 
  {
    "9G Servo Test",
    "ESR Test",
    "nRF24 Test",
    "L298N Test",
    "Ultrasonic Sensor Test"
    
  };
  const char* servoMenu[] = 
  {
    "Manual Operation Test",
    "Automatic Operation Test"
  };
  
  // infinite loop
  while (true)
  {
    // Get user Serial input for desired main menu test
    unsigned short int sel = menuSelection("Main Menu", mainMenu, (sizeof(mainMenu) / sizeof(char *))); 
        // Note: Division specifies the number of elements (ie, the number of char pointers) in the array of pointers
    
      switch (sel)
      {
        case 1: // 9G
        {
           Serial.println(sel); // DEBUG

           unsigned short int subSel = menuSelection("9G Servo Menu", servoMenu, (sizeof(servoMenu) / sizeof(char *))); 
           Serial.println(subSel); // DEBUG
          
          break;
        }
        case 2: // ESR
        {
          bool proceed;
          
          Serial.println(sel); // DEBUG

          proceed = infoScreen("This is a test msg with no real information. Sorry!");
          Serial.println(proceed);
          
          break;
        }
        case 3: // nRF
        {
          Serial.println(sel); // DEBUG

          break;
        }
        case 4: // L298N
        {
          Serial.println(sel); // DEBUG

          break;
        }
        case 5: // Ultrasonic
        {
          Serial.println(sel); // DEBUG

          break;
        }
        default:
        {
          Serial.println("Invalid input!");

          break;
        }
      }  

      delay(500);
  }
  

return 0;
}