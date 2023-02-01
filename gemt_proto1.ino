#include "Arduino.h"
#include "gemt_proto1.h"
#include "gemt_testSuite.h"

bool DEBUG = 0; // Variable to setup debugging


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


  // Space to Debug functions without going to main menu loop
  if (DEBUG == 1) 
  {
    int angle = 0;

    while (true) 
    {
      servoAutoTest(&angle);
    }

    return 0;
  }

  // Init array of pointers for menus
  const char* mainMenu[] = 
  {
    "Setup (run once)",
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
    Serial.println("\n\nPlease set Serial Inputs to \'No Line Ending\'");
    unsigned short int mainSel = menuSelection("Main Menu", mainMenu, (sizeof(mainMenu) / sizeof(char *))); 
                              // Note: Division specifies the number of elements (ie, the number of char pointers) in the array of pointers
    switch (mainSel)
    {
      case 1: // pinMode Setup
      {
        printHline('-');
        Serial.println("Setting up pins, hold tight!");
        gemTSetup();
        Serial.println("Setup completed, thank you for waiting");
        printHline('-');
        break;
      }
      case 2: // 9G
      { 
        String servoConnectionInfoMsg = "Connections:\n+ -> 5V\n- -> GND\nPWM pin -> 9\n";
        bool localExit = false; // Exit flag when user selects 'Back'

        int angle = 0;

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
                servoManualTest(&angle);
              }

              break;
            }
            case 2: // Auto mode
            {
              bool proceed = infoScreen(servoConnectionInfoMsg);

              if (proceed == true)
              {
                servoAutoTest(&angle);
              }
      
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
      case 3: // ESR
      {
        String ESRConnectionInfoMsg = "Connections:\nAnode -> Analog Pin 0\nCathode -> Purple Pin\n";
        bool   localExit            = false;
        double esrVal               = 0.0;
        while (localExit != true)
        {
          switch (infoScreen(ESRConnectionInfoMsg))
          {
            case true: 
            {
              esrVal = ESR_test();
              Serial.print("ESR Value: "); Serial.println(esrVal); Serial.println();
              break;
            }
            case false:
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
      case 4: // nRF
      {
        String nRFConnectionInfoMsg = "Connect functional nRF24 module into SPI1 port\
        \nConnect nRF24 module to be tested into SPI2";
    
        bool proceed = infoScreen(nRFConnectionInfoMsg);

        if (proceed == true)
        {
          nRFAutoTest();
        }

        break;
      }
      case 5: // L298N
      {
        String HBridgeConnectionInfoMsg = "Connections:\n+ -> 5V\n- -> GND\nenA -> 11\nenB -> 6\nout1 -> 2\n out2 -> 3\n";
        bool    localExit               = false;

        while (localExit != true)
        {
          switch (infoScreen(HBridgeConnectionInfoMsg))
          {
            case true: 
            {
              L8298_test();
              break;
            }
            case false:
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
      case 6: // Ultrasonic
      {
        String  UltraConnectionInfoMsg  = "Connections:\n+ -> 5V\n- -> GND\nTrig pin -> 27\nEcho pin -> 26\n";
        bool    localExit               = false;

        while (localExit != true)
        {
          switch (infoScreen(UltraConnectionInfoMsg))
          {
            case true: 
            {
              if (ultrasonicsensor_test())
              {
                Serial.println("Test Pass");
              }
              else
              {
                Serial.println("Test Fail");
              }
              break;
            }
            case false:
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
      default:
      {
        Serial.println("Error: Invalid inputs should be caught by getSerrialInput()!");

        break;
      }
    }  

    delay(50);
}


return 0;
}
