#include "currentcontrol.h"

void currentcontrol_init(void){
  // Initialize Timer2 interrupt for 5 kHz current control loop //
  T2CONbits.TCKPS = 0b011; // Timer2 prescaler N=8 (1:8)
  PR2 = 1999;              // period = (PR2+1) * N * 12.5 ns = 200 us => 5 kHz
  TMR2 = 0;                // initial Timer2 count is 0
  IPC2bits.T2IP = 5;       // priority for Timer2 interrupt
  IFS0bits.T2IF = 0;       // clear Timer2 interrupt flag
  IEC0bits.T2IE = 1;       // enable Timer2 interrupt
  T2CONbits.ON = 1;        // turn on Timer2

  // Initialize Timer3 and OutputCompare1 for 20 kHz PWM //
  T3CONbits.TCKPS = 0;     // Timer3 prescaler N=1 (1:1)
  PR3 = 3999;              // period = (PR3+1) * N * 12.5 ns = 50 us => 20 kHz
  TMR3 = 0;                // initial Timer3 count is 0
  OC1CONbits.OCTSEL = 1;   // use Timer 3
  OC1CONbits.OCM = 0b110;  // PWM mode without fault pin; other OC1CON bits are defaults
  OC1RS = 0;               // initial value of duty cycle (0 => zero current)
  OC1R = 0;                // initialize before turning OC1 on; afterward it is read-only
  T3CONbits.ON = 1;        // turn on Timer3
  OC1CONbits.ON = 1;       // turn on OC1

  // Initialize digital output pin to control motor direction //
  TRISDbits.TRISD8 = 0;    // pin D8 set as digital output
  LATDbits.LATD8 = 0;      // output = low => motor in forward
}