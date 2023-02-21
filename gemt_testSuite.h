#ifndef gemt_testSuite_h
#define gemt_testSuite_h

#include "Arduino.h"
#include <Servo.h>
#include <HCSR04.h>
#include "gemt_proto1.h"
#include "ESR.h"
#include <RF24.h>

// Define global vars if needed
enum testPins
{
  servoPWMPin = 9,
  triggerPin = 27,
  echoPin = 26,
};

Servo servo;

// Servo tests adapted from https://docs.arduino.cc/learn/electronics/servo-motors
/* 
  Nani comment "Major modifications include adding a 0 state in the manual sweep to 
  ensure the servo is starting at 0 degrees.
  The pins are also attached at the beginning of each test and then detached at the end."
*/

// Function to manually turn 9g microservo through 180 deg using Serial Monitor input
// Adjusts servo angles based on int inputs
void servoManualTest(int* anglePtr)
{
  *anglePtr = 0;
  servo.write(0);
  servo.attach(servoPWMPin);

  char buffer[50]; // init buffer of 50 bytes to hold expected string size
  short int input = 0;


  while (input != 999)
  {
    printHline('#');
    Serial.println("Enter desired angle\
  \nEnter 999 to return to Servo Menu");

    printHline('#');

    sprintf(buffer, "Current Angle: %d \n", *anglePtr);
    Serial.println(buffer);

    Serial.println(servo.read()); // DEBUG

    printHline('#');

    input = getSerialInput_int();

    
    // Servo angle limits
    if (input < 0)
    {
      *anglePtr = 0;
      servo.write(*anglePtr);
    }
    else if (input > 180)
    {
      *anglePtr = 180;
      servo.write(*anglePtr);
    }
    else
    {
      *anglePtr = input;
      servo.write(*anglePtr);
    }
    
  }

  Serial.println("Returning to Servo Menu..");
  servo.detach();
}

// Function for automatic servo testing
// Goes to 180 deg and then back
void servoAutoTest(int* anglePtr)
{
  *anglePtr = 0;
  servo.write(0);
  servo.attach(servoPWMPin);

  char buffer[50]; // init buffer of 50 bytes to hold expected string size

  printHline('#');
  Serial.println("Servo will now rotate to 180 degrees in 10 degree increments");
  delay(1000);

  for (int i = 0; i <= 18; ++i)
  {
    *anglePtr = (10 * i);

    sprintf(buffer, "Current Angle: %d", *anglePtr);
    Serial.println(buffer);
    servo.write(*anglePtr);
    delay(500);
  }

  Serial.println("Servo will now rotate to 0 in 10 degree increments");
  delay(1000);
  for (int i = 0; i <= 18; ++i)
  {
    *anglePtr = (180 - (10 * i));

    sprintf(buffer, "Current Angle: %d", *anglePtr);
    Serial.println(buffer);
    servo.write(*anglePtr);
    delay(500);
  }
  printHline('#');
  servo.detach();
}

//Function for electrolytic capacitor ESR measuring, sends a known well-defined and known current through the cap
//at a very well-defined voltage and measures the resistance through probes
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

  Serial.println("ESR Meter");
  Serial.println("Setting up");
  
  Vcc = readVcc(); //sets Vcc to well defined and measured arduino power rail voltage
  analogReference(INTERNAL);//setting vRef to internal reference 1.1V
 
  digitalWrite(PULSE_PIN,HIGH);//low enables T1
  
  digitalWrite(PULSE_PIN,HIGH);//low disables T2
  //pinMode(BUTTON_PIN,INPUT_PULLUP);//setting up for a button (will use this for zeroing)
  delay(1000);
  Serial.println("Please wait...\n");

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
// Known working one goes in SPI1
// Adapted from exampl on https://lastminuteengineers.com/nrf24l01-arduino-wireless-communication/
// Reworked to include simultanoes TX and RX with 2 modules attached to 1 arduino
void nRFAutoTest(void) 
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
  
  printHline('#');
  Serial.println("nRF24 Test started...");
  delay(1000);

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
    Serial.println("SPI1 not detected");
    result = 0;
  }
  
  else if ( spi2.isChipConnected() == 0 && spi1.isChipConnected() == 1)
  {
    Serial.println("SPI2 not detected");
    result = 0;
  }
  
  else {
    Serial.println("SPI1 and SPI2 not detected");
    result = 0;
  }


  // Return final results
  if (result == 1)
  {
     Serial.println("NRF TEST PASSED");
  }
  else
  {
    Serial.println("NRF TEST FAILED");
  }
  printHline('#');
}

// FIXME: Readout distances to serial
// FIXME: Provide some kind of start up message
//Function which tests if an ultrasonic sensor
//is measuring distance correctly
/*
  Ultrasonic Code Source: https://www.elegoo.com/blogs/arduino-projects/elegoo-uno-project-super-starter-kit-tutorial

  Used the source code file “SR04_Example.ino”  to determine the distance being read off the sensor. 
  Used the “SR04.h” library to simplify retrieving distance into one function call SR04(). 
  The source code only calculated the distance and printed its value out, the edited code used for GEMT has numerous “if” statements for each distance reading. 
  These “if” statements read a value and display a possible troubleshooting step based on what it is reading.
*/
bool ultrasonicsensor_test(void)
{ 
  long      duration;
  int       distance;
  //bool      measuring = true;
  double    permDistance;
  double    samples;


  Serial.println("Ultrasonic test starting, enter any character to end test...");

  while (Serial.available() == 0)
  {
     // Clears the trigPin
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2;
    // Prints the distance on the Serial Monitor
    delay(500);
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  
}

//Function which returns the output voltages
//on a L8298 motor]
float L8298_test(void)
{
  
  digitalWrite(L8in1, LOW);
  digitalWrite(L8in2, LOW);
  digitalWrite(L8in3, LOW);
  digitalWrite(L8in4, LOW);

  analogWrite(L8enA, 255);
  digitalWrite(L8in1, HIGH);
  digitalWrite(L8in2, LOW);
  delay(2000);
  voltL();
  digitalWrite(L8in1, LOW);
  digitalWrite(L8in2, HIGH);
  delay(2000);
  voltL();
  digitalWrite(L8in1, LOW);
  digitalWrite(L8in2, LOW);
  
  analogWrite(L8enB, 255);
  digitalWrite(L8in3, HIGH);
  digitalWrite(L8in4, LOW);
  delay(2000);
  voltR();
  digitalWrite(L8in3, LOW);
  digitalWrite(L8in4, HIGH);
  delay(2000);
  voltR();

  digitalWrite(L8in3, LOW);
  digitalWrite(L8in4, LOW);

}

void voltL()
{
  int value_in1 = analogRead(A1);
  float voltage_in1 = value_in1 * 5.0/1023;
  Serial.print("Voltage OUT1= ");
  Serial.println(voltage_in1);
  int value_in2 = analogRead(A2);
  float voltage_in2 = value_in2 * 5.0/1023;
  Serial.print("Voltage OUT2= ");
  Serial.println(voltage_in2);
}

void voltR()
{
  int value_in3 = analogRead(A3);
  float voltage_in3 = value_in3 * 5.0/1023;
  Serial.print("Voltage OUT3= ");
  Serial.println(voltage_in3);
  int value_in4 = analogRead(A4);
  float voltage_in4 = value_in4 * 5.0/1023;
  Serial.print("Voltage OUT4= ");
  Serial.println(voltage_in4);
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
#endif
