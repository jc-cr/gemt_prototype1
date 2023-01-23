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
  
  // infinite loop
  while (true)
  {
    unsigned short int sel = menuSelection();

      switch (sel)
      {
        case 1:
        {
          bool proceed;
          Serial.println (sel);
          proceed = infoScreen("This is a test msg with no real information. Sorry!");
          Serial.println(proceed);
          
          break;
        
        }
        case 2:
        {
          Serial.println(sel);
          break;
        }
        case 3:
        {
          Serial.println(sel);
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