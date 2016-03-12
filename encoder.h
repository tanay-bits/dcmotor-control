#ifndef ENCODER__H__
#define ENCODER__H__

#include <xc.h>    // processor SFR definitions

void encoder_init(void);
int encoder_counts(void);
void encoder_reset(void);

#endif // ENCODER__H__
