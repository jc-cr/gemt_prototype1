#include "Arduino.h"
#include "gemt_proto1.h"
#include "gemt_testSuite.h"

void setup (void)
{
  delay(50);
  Serial.begin(19200);
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

  // Submenu for servo
  // Note: Submenus should have back to return to previos menu
  // TODO: Find better way of incorporating menu heirchy
  const char* servoMenu[] = 
  {
    "Manual Operation Test",
    "Automatic Operation Test",
    "Back"
  };
  
  // infinite loop
  while (true)
  {
    // Main Menu switch    
      // Get user Serial input for desired main menu test
    unsigned short int mainSel = menuSelection("Main Menu", mainMenu, (sizeof(mainMenu) / sizeof(char *))); 
                              // Note: Division specifies the number of elements (ie, the number of char pointers) in the array of pointers
    switch (mainSel)
    {
      case 1: // 9G
      {
        String servoConnectionInfoMsg = "Connections:\n+ -> 5V\n- -> GND\nPWM pin -> 9\n";
        bool localExit = false; // Exit flag when user selects 'Back'

        // while loop to allow user to go back to previous screen
        while (localExit != true)
        {
          // Servo submenu switch
          unsigned short int subSel = menuSelection("9G Servo Menu", servoMenu, (sizeof(servoMenu) / sizeof(char *)));  
          switch(subSel)
          {
            case 1: // Manual mode
            {
              bool proceed = infoScreen(servoConnectionInfoMsg);

              if (proceed == true)
              {
                //run test
                servoManualTest();
              }

              break;
            }
            case 2: // Auto mode
            {

              break;
            }
            case 3: // Back
            {
              localExit = true;
              break;
            }
            default:
            {
              Serial.println("Error: Invalid inputs should be caught by getSerrialInput_int() !");
              break;
            }
          }        
        }
        
        break;
      }
      case 2: // ESR
      {
        bool proceed;
        
        Serial.println(mainSel); // DEBUG

        proceed = infoScreen("This is a test msg with no real information. Sorry!");
        Serial.println(proceed); // DEBUG
        
        break;
      }
      case 3: // nRF
      {
        Serial.println(mainSel); // DEBUG

        break;
      }
      case 4: // L298N
      {
        Serial.println(mainSel); // DEBUG

        break;
      }
      case 5: // Ultrasonic
      {
        Serial.println(mainSel); // DEBUG

        break;
      }
      default:
      {
        Serial.println("Error: Invalid ipunts should be caught by getSerrialInput()!");

        break;
      }
    }  

    delay(50);
}
  

return 0;
}