///////////////////////////
// Imports and constants //
///////////////////////////
#include "NU32.h"          // config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "utilities.h"     
#include "isense.h"   
#include "currentcontrol.h"
#include "positioncontrol.h" 

#define BUF_SIZE 200       // max UART message length
#define MAXSAMPS 2000      // max number of samples in ref trajectory

//////////////////////
// Global variables //
//////////////////////
static volatile int dutycycle = 0;                          // duty cycle [-100 to 100] for PWM
static volatile int ang_target = 0;                         // target position (deg)
static volatile int u_pos = 0;                              // position control signal (= current control ref)
static volatile int SENarray[100];                          // array of measured I for ITEST
static volatile int REFarray[100];                          // ref array for ITEST
static volatile float KpI = 0.75, KiI = 0.05;               // current control gains
static volatile float KpP = 150.0, KiP = 0.0, KdP = 5000;   // position control gains
static volatile int EIint = 0, EPint = 0;                   // integral (sum) of control error
static volatile int e_pos_prev = 0;                         // previous position error (for D control)
static volatile int num_samples = 0;                        // # of samples in ref trajectory
static volatile int REFtraj[MAXSAMPS];                      // ref trajectory for position control
static volatile int SENtraj[MAXSAMPS];                      // measured trajectory for position control

////////////////////////////////
// Interrupt Service Routines //
////////////////////////////////
void __ISR(_TIMER_2_VECTOR, IPL5SOFT) CurrentController(void){
  static int counter = 0;   // initialize counter once
  int sensed_cur;           // sensed current in mA
  int e, u, unew;

  switch (get_mode()) {
    case IDLE:
    {
      EIint = 0;
      dutycycle = 0;
      OC1RS = 0;            // 0 duty cycle => H-bridge in brake mode
      break;
    }

    case PWM:
    {
      if (dutycycle < 0){
        LATDbits.LATD8 = 1;    // output = high => motor in reverse
      }
      else{
        LATDbits.LATD8 = 0;    // output = low => motor in forward
      }

      // Cap to 100% if user input magnitude greater than a threshold:
      if (abs(dutycycle) > 100){
        dutycycle = 100;
      }
      
      OC1RS = (unsigned int)((abs(dutycycle)/100.0)*PR3);
      break; 
    }

    case ITEST:
    {
      // Reference signal:
      int ref;
      if (counter < 25){ref = 200;}
      else if (counter < 50){ref = -200;}
      else if (counter < 75){ref = 200;}
      else{ref = -200;}
      
      // PI current control signal:
      sensed_cur = read_cur_amps();
      e = ref - sensed_cur;

      // if (EIint < 800){
      //   EIint = EIint + e;
      // }

      EIint = EIint + e;
      u = KpI*e + KiI*EIint;
      if (u < 0){
        LATDbits.LATD8 = 1;    // output = high => motor in reverse
      }
      else{
        LATDbits.LATD8 = 0;    // output = low => motor in forward
      }
      unew = abs(u);
      
      if (unew > 100){unew = 100;}   
      OC1RS = unew * 40;

      // if (unew > 600){unew = 600;}
      // OC1RS = (unsigned int)((unew/600.0)*PR3);

      // Store data for MATLAB:
      SENarray[counter] = sensed_cur;
      REFarray[counter] = ref;

      counter++;
      if (counter == 100){
        EIint = 0;     // reset integral of control error
        counter = 0;   // reset counter
        set_mode(IDLE);
      }
      break;
    }

    case HOLD:
    {
      // PI current control signal:
      sensed_cur = read_cur_amps();
      e = u_pos - sensed_cur;

      EIint = EIint + e;
      
      // if (EIint < 1000){
      //   EIint = EIint + e;
      // }

      u = KpI*e + KiI*EIint;
      if (u < 0){
        LATDbits.LATD8 = 1;    // output = high => motor in reverse
      }
      else{
        LATDbits.LATD8 = 0;    // output = low => motor in forward
      }
      unew = abs(u);
      if (unew > 100){unew = 100;}
      
      OC1RS = (unsigned int)(((float)(unew)/100.0)*PR3);
      break;
    }

    case TRACK:
    {
      // PI current control signal:
      sensed_cur = read_cur_amps();
      e = u_pos - sensed_cur;

      EIint = EIint + e;
      
      // if EIint < 1000{
      //   EIint = EIint + e;
      // }

      u = KpI*e + KiI*EIint;
      if (u < 0){
        LATDbits.LATD8 = 1;    // output = high => motor in reverse
      }
      else{
        LATDbits.LATD8 = 0;    // output = low => motor in forward
      }
      unew = abs(u);
      if (unew > 100){unew = 100;}
      
      OC1RS = (unsigned int)(((float)(unew)/100.0)*PR3);
      break;
    }

    default:
    {
      NU32_LED2 = 0;  // turn on LED2 to indicate an error
      break;
    }

  }
  IFS0bits.T2IF = 0;    // clear interrupt flag
}

void __ISR(_TIMER_4_VECTOR, IPL7SOFT) PositionController(void){
  static int ctr = 0;          // initialize counter once
  int sensed_ang, ref_ang;     // angles in deg
  int e_pos, u_pos_proto;

  switch (get_mode()) {
    case IDLE:
    {
      break;
    }

    case PWM:
    {
      break;
    }
    
    case ITEST:
    {
      break;
    }
    
    case HOLD:
    {
      sensed_ang = encoder_degs();
      ref_ang = ang_target;
      e_pos = ref_ang - sensed_ang;
      EPint = EPint + e_pos;

      // position control signal:
      u_pos_proto = KpP*e_pos + KiP*EPint + KdP*(e_pos - e_pos_prev);
      if (u_pos_proto > 300){u_pos = 300;}
      else if (u_pos_proto < -300){u_pos = -300;}
      else u_pos = u_pos_proto;
      e_pos_prev = e_pos;
      break;
    }

    case TRACK:
    {
      sensed_ang = encoder_degs();
      ref_ang = REFtraj[ctr];
      e_pos = ref_ang - sensed_ang;
      EPint = EPint + e_pos;

      // position control signal:
      u_pos_proto = KpP*e_pos + KiP*EPint + KdP*(e_pos - e_pos_prev);
      if (u_pos_proto > 300){u_pos = 300;}
      else if (u_pos_proto < -300){u_pos = -300;}
      else u_pos = u_pos_proto;
      e_pos_prev = e_pos;

      // Store data for MATLAB:
      SENtraj[ctr] = sensed_ang;

      ctr++;
      if (ctr == num_samples){
        ang_target = REFtraj[num_samples-1];  // set last ref as target for HOLD
        ctr = 0;                              // reset counter
        set_mode(HOLD);
      }
      break;
    }

    default:
    {
      NU32_LED2 = 0;  // turn on LED2 to indicate an error
      break;
    }

  }
  IFS0bits.T4IF = 0;    // clear interrupt flag
}

///////////////////
// Main function //
///////////////////
int main() 
{
  char buffer[BUF_SIZE];
  NU32_Startup();         // cache on, min flash wait, interrupts on, LED/button init, UART init
  NU32_LED1 = 1;          // turn off the LEDs
  NU32_LED2 = 1;        
  __builtin_disable_interrupts();
  encoder_init();         // initialize SPI4 for encoder
  set_mode(IDLE);         // initialize PIC32 to IDLE mode
  adc_init();             // initialize ADC
  currentcontrol_init();  // initialize peripherals for current control
  positioncontrol_init(); // initialize timer4 for position control
  __builtin_enable_interrupts();

  while(1)
  {
    NU32_ReadUART3(buffer,BUF_SIZE); // we expect the next character to be a menu command
    NU32_LED2 = 1;                   // clear the error LED
    switch (buffer[0]) {
      case 'a':                      // read current sensor (ADC counts)
      {
        unsigned int adc_counts;
        adc_counts = adc_read();
        sprintf(buffer, "%d\r\n", adc_counts);
        NU32_WriteUART3(buffer);
        break;
      }

      case 'b':                      // read current sensor (mA)
      {
        int current;
        current = read_cur_amps();
        sprintf(buffer, "%d\r\n", current);
        NU32_WriteUART3(buffer);
        break;
      }

      case 'c':                      // read encoder (counts)
      {
        sprintf(buffer, "%d\r\n", encoder_counts());
        NU32_WriteUART3(buffer);
        break;
      }

      case 'd':                      // read encoder (deg)
      {
        sprintf(buffer, "%d\r\n", encoder_degs());
        NU32_WriteUART3(buffer);    
        break;
      }

      case 'e':                      // reset encoder counts
      {
        encoder_reset();        
        break;
      }

      case 'f':                      // set PWM (-100 to 100)
      {
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%d", &dutycycle);
        set_mode(PWM);
        break;
      }

      case 'g':                      // set current gains
      {
        float m, n;
        NU32_ReadUART3(buffer, BUF_SIZE);
        // __builtin_disable_interrupts();
        sscanf(buffer, "%f %f", &m, &n);
        KpI = m;
        KiI = n;
        // __builtin_enable_interrupts();
        break;
      }

      case 'h':                      // get current gains
      {
        sprintf(buffer, "%f\r\n", KpI);
        NU32_WriteUART3(buffer);
        sprintf(buffer, "%f\r\n", KiI);
        NU32_WriteUART3(buffer);
        break;
      }

      case 'i':                      // set position gains
      {
        float m, n, o;
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%f %f %f", &m, &n, &o);
        KpP = m;
        KiP = n;
        KdP = o;
        break;
      }

      case 'j':                      // get position gains
      {
        sprintf(buffer, "%f\r\n", KpP);
        NU32_WriteUART3(buffer);
        sprintf(buffer, "%f\r\n", KiP);
        NU32_WriteUART3(buffer);
        sprintf(buffer, "%f\r\n", KdP);
        NU32_WriteUART3(buffer);
        break;
      }


      case 'k':                      // test current control
      {
        // Switch to ITEST mode:
        __builtin_disable_interrupts();
        set_mode(ITEST);
        __builtin_enable_interrupts();
        while (get_mode()==ITEST){
          ;   // do nothing
        }

        // Send plot data to MATLAB:
        __builtin_disable_interrupts();       
        sprintf(buffer, "%d\r\n", 100);
        NU32_WriteUART3(buffer);
        int idx = 0;
        for (idx=0; idx<100; idx++){
          sprintf(buffer, "%d %d\r\n", REFarray[idx], SENarray[idx]);
          NU32_WriteUART3(buffer);
        }
        __builtin_enable_interrupts();
        break;
      }

      case 'l':                      // go to angle (deg)
      {
        __builtin_disable_interrupts();
        encoder_reset();
        e_pos_prev = 0;
        EPint = 0;
        EIint = 0;
        u_pos = 0;
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%d", &ang_target);
        set_mode(HOLD);
        __builtin_enable_interrupts();
        break;
      }

      case 'm':                      // load step trajectory
      {
        NU32_ReadUART3(buffer, BUF_SIZE);
        __builtin_disable_interrupts();
        int i, ref_deg;
        sscanf(buffer, "%d", &num_samples);
        for (i = 0; i < num_samples; i++){
          NU32_ReadUART3(buffer, BUF_SIZE);
          sscanf(buffer, "%d", &ref_deg);
          REFtraj[i] = ref_deg;
        }
        __builtin_enable_interrupts();        
        break;
      }

      case 'n':                      // load cubic trajectory
      {
        NU32_ReadUART3(buffer, BUF_SIZE);
        __builtin_disable_interrupts();
        int i, ref_deg;
        sscanf(buffer, "%d", &num_samples);
        for (i = 0; i < num_samples; i++){
          NU32_ReadUART3(buffer, BUF_SIZE);
          sscanf(buffer, "%d", &ref_deg);
          REFtraj[i] = ref_deg;
        }
        __builtin_enable_interrupts();        
        break;
      }

      case 'o':                      // execute trajectory
      {
        __builtin_disable_interrupts();
        encoder_reset();
        e_pos_prev = 0;
        EPint = 0;
        EIint = 0;
        u_pos = 0;
        __builtin_enable_interrupts();

        // Track, then hold:
        set_mode(TRACK);
        while (get_mode()==TRACK){;}

        // Send plot data to MATLAB:
        // __builtin_disable_interrupts();       
        sprintf(buffer, "%d\r\n", num_samples);
        NU32_WriteUART3(buffer);
        int i = 0;
        for (i=0; i<num_samples; i++){
          sprintf(buffer, "%d %d\r\n", REFtraj[i], SENtraj[i]);
          NU32_WriteUART3(buffer);
        }
        // __builtin_enable_interrupts();
        break;
      }

      case 'p':                      // unpower the motor
      {
        set_mode(IDLE);
        break;
      }

      case 'q':                      // quit
      {
        set_mode(IDLE);
        break;
      }

      case 'r':                      // get mode
      {
        sprintf(buffer, "%d\r\n", get_mode());
        NU32_WriteUART3(buffer);
        break;
      }
      
      default:
      {
        NU32_LED2 = 0;  // turn on LED2 to indicate an error
        break;
      }
    }
  }
  return 0;
}