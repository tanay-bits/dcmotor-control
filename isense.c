#include "isense.h"

#define SAMPLE_TIME 10     // ADC sample time in core timer ticks (1 tick = 25 ns)

void adc_init(void) {
  AD1PCFGbits.PCFG0 = 0;  //configure RB0 (AN0) as analog input
  AD1CON3bits.ADCS = 2;   // ADC clock period is Tad = 2*(ADCS+1)*Tpb = 2*3*12.5ns = 75ns 
  AD1CON1bits.ADON = 1;   // turn on A/D converter
  AD1CHSbits.CH0SA = 0;   // connect AN0 to MUXA for sampling
  AD1CON1bits.SSRC = 0b111; // auto conversion mode
}

unsigned int adc_read(void) {
  AD1CON1bits.SAMP = 1; // start sampling (manual)
  _CP0_SET_COUNT(0);
  while (_CP0_GET_COUNT() < SAMPLE_TIME){
    ; // sample for more than 25*SAMPLE_TIME ns
  }
  while (!AD1CON1bits.DONE){
    ; // wait for conversion to finish
  }
  return ADC1BUF0;
}