
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

unsigned long measureESR(void);
long readVcc(void);
#endif