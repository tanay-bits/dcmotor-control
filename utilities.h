#ifndef UTILITIES__H__
#define UTILITIES__H__

typedef enum {IDLE=0, PWM=1, ITEST=2, HOLD=3, TRACK=4} Mode_datatype;

void set_mode(Mode_datatype m);
Mode_datatype get_mode(void);

#endif // UTILITIES__H__