#include "Arduino.h"
#include <Servo.h>
#include "gemt_proto1.h"





// Function to manually turn 9g microservo through 180 deg using Serial Monitor input
// User can enter:
//    int, angle multiplier
//    '+' or '-', which will move servo CW or CCW; This will be a counter multiplied by multiplier
//    'h', return to home (0 deg)
//    'l', go to limit (180 deg)


void servoManualTest(void)
{
  // servo.attach(9);

  int angle = 0;
  int multiplier = 1;
  char buffer[50]; // init buffer of 50 bytes to hold expected string size
  char input = "";

  const char inputInstructions [] ={
  "Enter an integer > 1 to change multiplier (DEFAULT == 1)\
  \nEnter + to move CW by multiplier\
  \nEnter - to move CCW by multiplier\
  \nEnter h to go to 0 angle\
  \nEnter l to go to 180 angle\
  \nEnter x to exit test"};

  while (input != 'x')
  {
    printHline("#");
    Serial.println(inputInstructions);
    printHline("#");
    sprintf(buffer, "Current Angle: %d\
                      \nMultiplier: %d\
                      \n ", angle, multiplier);
    
    Serial.println(buffer);
    printHline("#");

    input = getSerialInput_char();

    if (angle < 0)
    {
      angle = 0;
    }

    else if (angle > 180)
    {
      angle = 180;
    }
  }

  
}


/*

// Encoder based servo test
// Will keep for ref. To be updated with arrow key version

void servoManual_test(void)
{
  #define CLK 2
  #define DT 3
  Servo servo;
  int counter = 0;
  int currentStateCLK;
  int lastStateCLK;

  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);
  servo.attach(9);
  servo.write(counter);
  lastStateCLK = digitalRead(CLK);

  // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);
  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DT) != currentStateCLK) {
      counter --;
      if (counter<0)
        counter=0;
    } else {
      // Encoder is rotating CW so increment
      counter ++;
      if (counter>180)
        counter=180;
    }
    // Move the servo
    servo.write(counter);
    Serial.print("Position: ");
    Serial.println(counter);
  }
  // Remember last CLK state
  lastStateCLK = currentStateCLK;
}

*/


/*
#include "GEMT_TestSuite.h"

//Include Libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include "Arduino.h"
#include "SR04.H" //What is this?

// ESR Settings:
//-----------------------------------------------------------------
#ifndef GEMT_ESR_h
#define GEMT_ESR_h

#define FASTADC 1
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//define the input and output pin we will use
#define DISCHARGE_PIN 42
#define ESR_PIN A0
#define PULSE_PIN 43
//#define BUTTON_PIN 4

#endif
//-----------------------------------------------------------------

// This artifact from previous development.. can delete
// Main test filter
bool GEMT_test(String moduleID)
{
  bool testResult;
  
  if (moduleID == "nRF24L01")
  {
    testResult = nRF24_test();
  }
  //
  // ... DO rest for other modules


  return testResult;
}


double ESR_test(void)
{
  unsigned long esrSamples;
  double        miliVolt;
  double        esrVal;
  double        esrCal;
  double        Vcc;
  double        totalAvg;
  double        vRef = 1.069;//voltage on the Aref pin 
  double        current = 0.088564;
  int           count = 0;

  Serial.begin(115200);
  Serial.println("ESR Meter");
  Serial.println("Setting up");
  
  Vcc = readVcc(); //sets Vcc to well defined and measured arduino power rail voltage
  analogReference(INTERNAL1V1);//setting vRef to internal reference 1.1V
 

  pinMode(ESR_PIN, INPUT);//reading miliVolt
  pinMode(PULSE_PIN, OUTPUT);
  digitalWrite(PULSE_PIN,HIGH);//low enables T1
  pinMode(DISCHARGE_PIN, OUTPUT);
  digitalWrite(PULSE_PIN,HIGH);//low disables T2
  //pinMode(BUTTON_PIN,INPUT_PULLUP);//setting up for a button (will use this for zeroing)
  delay(1000);
  Serial.println("Please wait...");

  
  if (FASTADC) 
  {
    sbi(ADCSRA,ADPS2);
    cbi(ADCSRA,ADPS1);
    sbi(ADCSRA,ADPS0);
  }

  
  //eeprom_read_block((void*)&esrCal, (void*)0, sizeof(esrCal));

  while (count < 4)
  {
    esrSamples = measureESR();//this function takes a while,)
    miliVolt = (esrSamples * vRef) / 65.535;//calculating voltage on AIN0 pin
    esrVal = 100 / ((Vcc/miliVolt)-1); //esr value in ohms
    //esrVal = (miliVolt*100)/((Vcc)-(miliVolt));
    esrVal = esrVal * 1000; //esrval in mOhms
  
    esrVal = esrVal - 286.77;
    totalAvg += esrVal;
    count++;
  }
  totalAvg /= 4;
  
  Serial.println("ESR Value: " + totalAvg);
  return totalAvg;
}

//oversampler function for measuring ESR value
unsigned long measureESR()
{
  unsigned long samples = 0;
  unsigned int acumulator = 0;
  int i = 0;
  //oversampling 4096 times (for 16 bit is 4^(desiredResolution - ADCresolution))
  while(i < 4096) {
    digitalWrite(DISCHARGE_PIN,HIGH);//discharge caps
    delayMicroseconds(600);
    digitalWrite(DISCHARGE_PIN,LOW); //disable discharging
    digitalWrite(PULSE_PIN,LOW);//making a miliVolt pulse of 50mA
    delayMicroseconds(5);//on the scope it looks that after enabling the pulse a litle delay is
    //recomended so the oscillations fade away
    acumulator = analogRead(ESR_PIN);//reading value on AIN0
    digitalWrite(PULSE_PIN,HIGH);//stopping pulse
    samples += acumulator;//acumulating the readings
    i++;
  }
  samples = samples >> 6;//decimating value
  return samples;
}

//function designed to find the true Vcc power rail voltage, used for ESR calculations
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
     ADMUX = _BV(MUX5) | _BV(MUX0) ;
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}


// Function to test nRF24 plugged into SPI2 port
// KNown working one goes in SPI1
// ToDo: figure out why first test is passing when shoudlbn't be... 
// 
bool nRF24_test(void) 
{
  bool result;
  
  //create an RF24 object
  // All test don on SPI2 port input
  
  RF24 spi1(22, 23);  // CE, CSN
  RF24 spi2(24, 25);  // CE, CSN

  spi1.begin();
  spi2.begin();
  
  //address through which two modules communicate.
  const byte address[6] = "00002";

  // Test connections, display feedback
  if (spi2.isChipConnected() == 1 && spi1.isChipConnected() == 1) 
  {
    // TEST 1: SPI2 TX
    //----------------------------------------------
    //set the address
    spi1.openReadingPipe(0, address);

    // RX Radio config:
    spi1.flush_tx(); // Clear 3 FIFO buffers
    spi1.setPALevel(0); // Set power amplifier low
    spi1.startListening(); //Set module as receiver
    
    // TX Radio config:
    spi2.setPALevel(0); // Set power amplifier min
    spi2.openWritingPipe(address); //set the address
    spi2.stopListening(); //Set module as transmitter

    // Transmission test. Loop few times because radios are weird
    const char text[] = "Test message";
    char msgBuf[32] = {0};
    for (int i = 0; i < 5; ++i)    
    {
      spi1.read(&msgBuf, sizeof(msgBuf));
      
      delay(10);
      
      if (spi2.write(&text, sizeof(text))) 
      { 
        // We got acknoledgement of message, test done
        result = 1;
        break;
      }
      else
      {
         result = 0;
      }
      
      delay(100);
    }

    // If first test passed, continue to next test
    if (result == 1)
    {
      // TEST 2: SPI2 RX
      //----------------------------------------------
    
      // RX radio config:
      spi2.openReadingPipe(0, address); //set the address   
      spi2.flush_tx();  // Clear 3 FIFO buffers  
      spi2.setPALevel(0);  // Set power amplifier low
      spi2.startListening();  //Set module as receiver
    
      // TX Radio config:
      spi1.setPALevel(0);     // Set power amplifier min 
      spi1.openWritingPipe(address); //set the address
      spi1.stopListening(); //Set module as transmitter
  
      // Transmission test. Loop few times because radios are weird
      const char text[] = "Test message";
      char msgBuf[32] = {0};
      for (int i = 0; i < 5; ++i)    
      {
        spi2.read(&msgBuf, sizeof(msgBuf));
        
        delay(10);
        
        if (spi1.write(&text, sizeof(text))) 
        { 
          // We got acknoledgement of message, test done
          result = 1;
          break;
        }
        else
        {
           result = 0;
        }
        
        delay(100);
      }
    }
  }
  
  else if (spi2.isChipConnected() == 1 && spi1.isChipConnected() == 0) 
  {
    // print radio 1 not detected
    result = 0;
  }
  
  else if ( spi2.isChipConnected() == 0 && spi1.isChipConnected() == 1)
  {
    //print radio 2 not detected
    result = 0;
  }
  
  else {
  // print No radio detected
    result = 0;
  }


  // Return final results as bool value
  return result;
}

bool ultrasonicsensor_test(void)
{
  long distance;
  SR04 sensor = SR04(26,27);

  distance = sensor.Distance();
  if (distance > 400) {
    Serial.print("Distance value: ");
    Serial.print(distance);
    Serial.print(" cm\n");
    Serial.print("Fail - Distance exceeds maximum limit of 400 cm\n");
    Serial.print("****************************************");
    Serial.print("\n");
    delay(1000);
    return false;
  } 
   else if (distance <= 0) {
    Serial.print("Distance value: ");
    Serial.print(distance);
    Serial.print(" cm\n");
    Serial.print("Fail - Check if pins are properly connected\n");
    Serial.print("****************************************");
    Serial.print("\n");
    delay(1000);
    return false;
   }
  else if (distance < 2 && distance > 0) {
    Serial.print("Distance value: ");
    Serial.print(distance);
    Serial.print(" cm\n");
    Serial.print("Fail - Distance lower than minimum limit of 2 cm\n");
    Serial.print("****************************************");
    Serial.print("\n");
    delay(1000);
    return false;
  }
  else if (distance >= 2 && distance <= 400) {
    Serial.print("Distance value: ");
    Serial.print(distance);
    Serial.print(" cm\n");
    Serial.print("Pass - Distance within range of 2-400 cm\n");
    Serial.print("****************************************");
    Serial.print("\n");
    delay(1000);
    return true;
  }  
}

void servoManual_test(void)
{
  #define CLK 2
  #define DT 3
  Servo servo;
  int counter = 0;
  int currentStateCLK;
  int lastStateCLK;
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);
  servo.attach(9);
  servo.write(counter);
  lastStateCLK = digitalRead(CLK);

  // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);
  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DT) != currentStateCLK) {
      counter --;
      if (counter<0)
        counter=0;
    } else {
      // Encoder is rotating CW so increment
      counter ++;
      if (counter>180)
        counter=180;
    }
    // Move the servo
    servo.write(counter);
    Serial.print("Position: ");
    Serial.println(counter);
  }
  // Remember last CLK state
  lastStateCLK = currentStateCLK;
}

*/