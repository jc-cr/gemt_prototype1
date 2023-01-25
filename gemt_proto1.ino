#include "Arduino.h"
#include "gemt_proto1.h"

void setup (void)
{
  delay(50);
  Serial.begin(19200);
}

// FIXME: Getting stuck on first input for selections


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
    // Main Menu switch    
      // Get user Serial input for desired main menu test
    unsigned short int sel = menuSelection("Main Menu", mainMenu, (sizeof(mainMenu) / sizeof(char *))); 
                              // Note: Division specifies the number of elements (ie, the number of char pointers) in the array of pointers
    switch (sel)
    {
      case 1: // 9G
      {
        String servoConnectionInfoMsg = "Connections:\n+ -> 5V\n- -> GND\nPWM pin -> 9\n";

        // Servo submenu switch
        unsigned short int subSel = menuSelection("9G Servo Menu", servoMenu, (sizeof(servoMenu) / sizeof(char *)));  
       
        switch(subSel)
        {
          case 1: // Manual mode
          {
            bool proceed;
            proceed = infoScreen(servoConnectionInfoMsg);


            break;
          }
          case 2: // Auto mode
          {

            break;
          }
          default:
          {
            Serial.println("Error: Invalid inputs should be caught by getSerrialInput_int() !");
          }
        }
        
        break;
      }
      case 2: // ESR
      {
        bool proceed;
        
        Serial.println(sel); // DEBUG

        proceed = infoScreen("This is a test msg with no real information. Sorry!");
        Serial.println(proceed); // DEBUG
        
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
        Serial.println("Error: Invalid ipunts should be caught by getSerrialInput()!");

        break;
      }
    }  

    delay(50);
}
  

return 0;
}