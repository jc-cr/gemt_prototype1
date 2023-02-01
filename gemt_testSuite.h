#include "Arduino.h"
#include <Servo.h>
#include <HCSR04.h>
#include "gemt_proto1.h"
#include "ESR.h"
#include <RF24.h>

void gemTSetup()
{
  //ESR Pins
  pinMode(ESR_PIN, INPUT);//reading miliVolt
  pinMode(PULSE_PIN, OUTPUT);
  pinMode(DISCHARGE_PIN, OUTPUT);

  //L8298 Pins
  pinMode(L8enA, OUTPUT);
  pinMode(L8in1, OUTPUT);
  pinMode(L8in2, OUTPUT);
  pinMode(L8enB, OUTPUT);
  pinMode(L8in3, OUTPUT);
  pinMode(L8in4, OUTPUT);
}

// Function to manually turn 9g microservo through 180 deg using Serial Monitor input
// Adjusts servo angles based on int inputs
void servoManualTest(int* anglePtr)
{
  Servo servo;
  servo.attach(9);

  char buffer[50]; // init buffer of 50 bytes to hold expected string size
  unsigned short int input = 0;

  while (input != 999)
  {
    printHline('#');
    Serial.println("Enter desired angle\
  \nEnter 999 to return to Servo Menu");
    printHline('#');
    sprintf(buffer, "Current Angle: %d \n", *anglePtr);
    Serial.println(buffer);
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
      servo.write(anglePtr);
    }
    else
    {
      *anglePtr = input;
      servo.write(*anglePtr);
    }
    
  }

  Serial.println("Returning to Servo Menu..");
}

// Function for automatic servo testing
// Goes to 180 deg and then back
void servoAutoTest(int* anglePtr)
{
  Servo servo;
  servo.attach(9);

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
    delay(1000);
  }

  Serial.println("Servo will now rotate to 0 in 10 degree increments");
  delay(1000);
  for (int i = 0; i <= 18; ++i)
  {
    *anglePtr = (180 - (10 * i));

    sprintf(buffer, "Current Angle: %d", *anglePtr);
    Serial.println(buffer);
    servo.write(*anglePtr);
    delay(1000);
  }
  printHline('#');
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
// KNown working one goes in SPI1
// FIXME: figure out why first test is passing when shoudlbn't be
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

//Function which tests if an ultrasonic sensor
//is measuring distance correctly
bool ultrasonicsensor_test(void)
{
  
  double*   distances;
  double    permDistance;
  double    samples;
  double    i = 0.0;
  int       trig_pin = 27;
  int       echo_pin = 26;
  
  HCSR04.begin(trig_pin, echo_pin); 
  while (i < 10)
  {
    distances = HCSR04.measureDistanceCm();
    samples += distances[0];
    i++;
    delay(1000);
  }
  permDistance = samples / i;
  
  if (permDistance > 400) {
    return false;
  } 
   else if (permDistance <= 0) {
    return false;
   }
  else if (permDistance < 2 && permDistance > 0) {
    return false;
  }
  else if (permDistance >= 2 && permDistance <= 400) {
    return true;
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
  
  analogWrite(L8enB, 255);
  digitalWrite(L8in3, HIGH);
  digitalWrite(L8in4, LOW);
  delay(2000);
  voltR();
  digitalWrite(L8in3, LOW);
  digitalWrite(L8in4, HIGH);
  delay(2000);
  voltR();
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
