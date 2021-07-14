#define F_CPU 1000000  // This is used by delay.h library

#include <stdlib.h>
#include <EEPROM.h> 

#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>    // Adds delay_ms and delay_us functions
  
#include <avr/sleep.h>
#include <avr/interrupt.h>

// Routines to set and clear bits (used in the sleep code)
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Variables for the Sleep/power down modes:
volatile boolean f_wdt = 1;

const unsigned long durationMillis = 5000;
const byte buzzerPin = 1;
unsigned char bootTimes = 0x00;

const uint8_t dividers[] = { 0x32, 0x2D, 0x2A, 0x26, 0x24, 0x21 };

void setup() {
  pinMode(buzzerPin, OUTPUT);
  setup_watchdog(9);
}

void loop() {
  
  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
  if (f_wdt==1) {
    // Run code only each 120sec of sleep
    if(bootTimes == 0){
      // Notes played at 10th octave
      // Since every octave has 11 elements, virtually jumping to next octave requires 11*j+1
      for(int i = 0; i<6; i++){
        unsigned char divisor = dividers[i];
        divisor = divisor >> 1; //divides by 2 using bit shift (more efficient)
        TinyTone(divisor, durationMillis); //max Octave for 1MHz AtTiny85 is 7
        delay(durationMillis);
      }
    }
    pinMode(buzzerPin, INPUT); //set as INPUT to save energy
    system_sleep();  // Send the unit to sleep
    pinMode(buzzerPin, OUTPUT);
    // After wake up, these lines will be executed
    bootTimes++;
    // Reset counter each 15 times (15*8 sec = 120 sec)
    if(bootTimes == 0x0F){
      bootTimes = 0x00;
    }
  }
}

void TinyTone(unsigned char divisor, unsigned long duration)
{
  // 7-bit register
  // Last 4 bits is a register equivalent to prescaler, where
  // prescaler = 2^(n-1) (for n between 0001b and 1111b)
  // if n == 0 then it is disabled
  TCCR1 = 0x90 | 0x01; // for 1MHz clock
  OCR1C = divisor-1;         // set the OCR
  delay(duration);
  TCCR1 = 0x90;              // stop the counter
}

// set system into the sleep state 
// system wakes up when wtchdog is timed out
void system_sleep() {
  
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();

  sleep_mode();                        // System actually sleeps here

  sleep_disable();                     // System continues execution here when watchdog timed out 
  
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
  
}

// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;

  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}
  
// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  f_wdt=1;  // set global flag
}
