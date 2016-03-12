#include "NU32.h"          // config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "utilities.h"     
#include "isense.h"   
#include "currentcontrol.h"   

#define BUF_SIZE 200
#define ADC_AVG_OVER 10

static volatile int dutycycle;

//////////////////////////////////
// ISR for current control loop //
//////////////////////////////////
void __ISR(_TIMER_2_VECTOR, IPL5SOFT) CurrentController(void){
  switch (get_mode()) {
    case IDLE:
    {
      OC1RS = 0;   // 0 duty cycle => H-bridge in brake mode
      break;
    }

    case PWM:
    {
      if (dutycycle < 0){
        LATDbits.LATD8 = 1;     // output = high => motor in reverse
      }
      else{
        LATDbits.LATD8 = 0;    // output = low => motor in forward
      }

      // Cap to max valid if user input magnitude greater than 100:
      if (abs(dutycycle) > 100){
        dutycycle = 100;
      }
      
      OC1RS = (unsigned int)((abs(dutycycle)/100.0)*PR3); 
    }

    default:
    {
      NU32_LED2 = 0;  // turn on LED2 to indicate an error
      break;
    }

  }


  IFS0bits.T2IF = 0;    // clear interrupt flag
}


int main() 
{
  char buffer[BUF_SIZE];
  NU32_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  NU32_LED1 = 1;  // turn off the LEDs
  NU32_LED2 = 1;        
  __builtin_disable_interrupts();
  encoder_init();	// initialize SPI4 for encoder
  set_mode(IDLE);	// initialize PIC32 to IDLE mode
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
        unsigned int sum, i, adc_avg;
        sum = 0;
        for (i=0; i<ADC_AVG_OVER; i++){
          sum = sum + adc_read();
        }
        adc_avg = sum/ADC_AVG_OVER;
        sprintf(buffer, "%d\r\n", adc_avg);
        NU32_WriteUART3(buffer);
        break;
      }

      case 'b':                      // read current sensor (mA)
      {
        unsigned int sum, i, adc_avg;
        int current;
        sum = 0;
        for (i=0; i<ADC_AVG_OVER; i++){
          sum = sum + adc_read();
        }
        adc_avg = sum/ADC_AVG_OVER;
        current = 2.04 * adc_avg - 1024;    // from calibration
        // if (-3 < current < 3){current = 0;} // deadband
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

      case 'r':                      // get mode
      {
        sprintf(buffer, "%d\r\n", get_mode());
        NU32_WriteUART3(buffer);
        break;
      }
      
      case 'q':
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