#include "positioncontrol.h"

void positioncontrol_init(void){
  // Initialize Timer4 interrupt for 200 Hz position control loop //
  T4CONbits.TCKPS = 0b110; // Timer4 prescaler N=64 (1:64)
  PR4 = 6249;              // period = (PR4+1) * N * 12.5 ns = 0.005 s => 200 Hz
  TMR4 = 0;                // initial Timer4 count is 0
  IPC4bits.T4IP = 7;       // priority for Timer4 interrupt
  IFS0bits.T4IF = 0;       // clear Timer4 interrupt flag
  IEC0bits.T4IE = 1;       // enable Timer4 interrupt
  T4CONbits.ON = 1;        // turn on Timer4    
}