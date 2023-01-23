#include "Arduino.h"
#include "gemt_proto1.h"


// Displays the instructions to a test (pins to connect to, etc.)
// bool return determines if test will proceed or go back to previous screen
bool infoScreen (String infoMsg)
{

}

void setup(void)
{
  delay(500);
  Serial.begin(9600);
}

int main(void)
{
  // Must intialize the arduino firmware
  init();
  setup();
  
  // infinite loop
  while(true)
  {
    unsigned short int sel = menuSelection();

      switch(sel)
      {
        case 1:
        {
          Serial.println(sel);
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