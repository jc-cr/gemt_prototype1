
#ifndef ESR_h
#define ESR_h

#define FASTADC 1
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//define the input and output pin we will use for ESR
#define DISCHARGE_PIN 42
#define ESR_PIN A0
#define PULSE_PIN 43

//define the in and out pins for L8298
#define L8in1 5
#define L8in2 6
#define L8enA 7

#define L8in3 8
#define L8in4 9
#define L8enB 10
void voltL(void);
void voltR(void);

unsigned long measureESR(void);
long readVcc(void);
#endif
