#ifndef ISENSE__H__
#define ISENSE__H__

#include <xc.h>   // processor SFR definitions

void adc_init(void);        // initialize ADC
unsigned int adc_read(void); // read ADC count

#endif // ISENSE__H__
