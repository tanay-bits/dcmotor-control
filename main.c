///////////////////////////
// Imports and constants //
///////////////////////////
#include "NU32.h"          // config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "utilities.h"     
#include "isense.h"   
#include "currentcontrol.h"   

#define BUF_SIZE 200       // max UART message length
// #define NUMSAMPS 100       // number of points in waveform
// #define PLOTPTS 100        // number of data points to plot
// #define PLOTEVERY 1        // plot data every PLOTEVERYth control iteration

//////////////////////
// Global variables //
//////////////////////
static volatile int dutycycle;
// static volatile int Waveform[NUMSAMPS];
static volatile int SENarray[100];
static volatile int REFarray[100];
// static volatile int StoringData = 0;     // if this flag=1, currently storing plot data
static volatile float KpI = 0.0, KiI = 0.0;
static volatile int Eint = 0;            // integral (sum) of control error

/////////////////////////
// Auxiliary functions //
/////////////////////////
// void makeWaveform(){
//   int i=0, center=0, A=200;
//   for (i=0; i<NUMSAMPS; ++i){
//     if (i>0 && i%25==0){
//       A = -A;
//     }
  
//     Waveform[i] = center + A;
    
//   }
// }

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
      dutycycle = 0;
      OC1RS = 0;   // 0 duty cycle => H-bridge in brake mode
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

      // Cap to max valid if user input magnitude greater than 100:
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
      Eint = Eint + e;
      u = KpI*e + KiI*Eint;
      if (u < 0){
        LATDbits.LATD8 = 1;    // output = high => motor in reverse
      }
      else{
        LATDbits.LATD8 = 0;    // output = low => motor in forward
      }
      // unew = u + 50;
      unew = abs(u);
      if (unew > 100){unew = 100;}
      OC1RS = unew * 40;

      // if (unew > 400){unew = 400;}
      // OC1RS = unew * 10;

      // Store data for MATLAB:
      SENarray[counter] = sensed_cur;
      REFarray[counter] = ref;

      counter++;
      if (counter == 100){
        Eint = 0;     // reset integral of control error
        counter = 0;  // reset counter
        set_mode(IDLE);
      }
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

///////////////////
// Main function //
///////////////////
int main() 
{
  char buffer[BUF_SIZE];
  NU32_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  NU32_LED1 = 1;  // turn off the LEDs
  NU32_LED2 = 1;        
  __builtin_disable_interrupts();
  encoder_init();   // initialize SPI4 for encoder
  set_mode(IDLE);   // initialize PIC32 to IDLE mode
  adc_init();     // initialize ADC
  currentcontrol_init();  // initialize peripherals for current control
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
        NU32_WriteUART3(buffer); // send encoder count to client
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
        set_mode(PWM);
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%d", &dutycycle);
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

      case 'k':                      // test current control
      {
        // Switch to ITEST mode:
        // __builtin_disable_interrupts();
        set_mode(ITEST);
        // __builtin_enable_interrupts();
        while (get_mode()==ITEST){
          ;   // do nothing
        }

        // Send plot data to MATLAB:       
        sprintf(buffer, "%d\r\n", 100);
        NU32_WriteUART3(buffer);
        int idx = 0;
        // char message[100];
        for (idx=0; idx<100; idx++){
          sprintf(buffer, "%d %d\r\n", REFarray[idx], SENarray[idx]);
          NU32_WriteUART3(buffer);
        }

        break;
      }

      case 'r':                      // get mode
      {
        sprintf(buffer, "%d\r\n", get_mode());
        NU32_WriteUART3(buffer);
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

      default:
      {
        NU32_LED2 = 0;  // turn on LED2 to indicate an error
        break;
      }
    }
  }
  return 0;
}




// case 'x':                      // dummy command for demonstration purposes
// {
//   int n = 0, m = 0;
//   NU32_ReadUART3(buffer,BUF_SIZE);
//   sscanf(buffer, "%d %d", &n,&m);
//   sprintf(buffer,"%d\r\n", n + m); // return the sum of numbers
//   NU32_WriteUART3(buffer);
//   break;
// }