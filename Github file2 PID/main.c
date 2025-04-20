#include "intrinsics.h"
#include "msp430g2553.h"
#include <msp430.h>

/*
 * main.c
 */

// myscopemain
// 2019.02.13-Kenrick Chin
// 2025.02.06-Modified
//-----------------------------------------------------------------
// ReadMe
//-----------------------------------------------------------------
// myscope main.c code works with your
// MATLAB code under the following conditions:
// MSP430 is configured for 16MHz clock.
// UART is set for 115200baud.
// NPOINTS=400
// ADC input is on P1.4
// MATLAB code mustbe changed to connect to the proper COMx port.
//-----------------------------------------------------------------
#define NPOINTS 400
void Init(void);
void Init_ADC(void);
void Init_UART(void);
void Config_Pins(void);

#define DIR1    BIT6  // P1.6 (Heating: DIR1 = LOW)
#define DIR2    BIT6  // P2.6 (Cooling: DIR2 = LOW)

//--------------------------------------------------------
// GlobalVariables
//--------------------------------------------------------
// This is where you store the ADC10MEM readings
volatile unsigned char v[NPOINTS];
// This is where you store the synchronization character
char received_char = 0;
// This is where you indicate the task should start
volatile int task_ready = 0;
// Provides changing count
//Provides timer value
volatile int cntrlsig=0;
volatile int sign=0;
volatile int vavg=0;
volatile int sum=0;

//--------------------------------------------------------
// Main
//--------------------------------------------------------
void main(void) {
  // We are calling the Init() function.
  Init();
  // We are calling the Init_UART() function.
  Init_UART();
  // We are calling the Init_ADC() function.
  Init_ADC();

  Config_Pins();

  // Configure Timer0_A3
  TACTL |= TASSEL_2;           // Select SMCLK as the clock source
  TACTL |= ID_0;               // Divider = 1 (no division)
  TACTL |= MC_2;               // Continuous mode (count up to 0xFFFF)
  /*
   * To Do: Enable USCI_A0 RX interrupt (see section 15.4.12 of the user's
   * guide). This (its ISR) will help us ensure the process starts only after
   * receiving the synchronization character. In other words, we wait to receive
   * a one-time transmission from MATLAB to begin.
   */
  IE2 |= UCA0RXIE;
  // To Do: Activate the General Interrupt Enable bit (GIE)
  __bis_SR_register(GIE);

  // This is our main loop
  while (1) {
    // To Do: Check if task_ready is 1, indicating that a character has been received. 
    if (task_ready) {
      
      // To Do: Set the ADC10 interrupt enable bit in ADC10CTL0.
      ADC10CTL0 |= ADC10IE; // Enable ADC10 interrupt

      /* Note (This implementation is not necessary): Synchronization can be improved by 
       * clearing the task_ready and setting it back to zero. However, this requires MATLAB
       * to send a character in the beginning of each loop.
       */ 
      //ADC10CTL0 |= ENC | ADC10SC;

      //task_ready = 0;
    } else {
      /* 
       * Since the character has not been received yet, we go to sleep 
       * and wait until something is received.
       * To Do: Enter Low Power Mode 0 (LPM0_bits) with General Interrupt Enable (GIE).
       */
      __bis_SR_register(LPM0_bits | GIE);
    }
  }
}

void Config_Pins(void){     
  P1SEL |= BIT6;
  P1SEL2 &= ~BIT6;

  // H-Bridge Direction Pins
  P1DIR |= DIR1;    // P1.6 as output (DIR1)
  P2DIR |= DIR2;    // P2.6 as output (DIR2)
  P2OUT=0;
  P2SEL &= ~DIR2;
  P2SEL &= ~BIT7;
  P2SEL2 &= ~DIR2;
  P2SEL2 &= ~BIT7;

  
}
//--------------------------------------------------------
// Initialization
//--------------------------------------------------------
void Init(void) {
  // To Do: Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW | WDTHOLD;
  // To Do: Set system clock to calibrated values for 16 MHz
  DCOCTL = CALDCO_16MHZ;    
  BCSCTL1 = CALBC1_16MHZ;

  TA0CTL = TASSEL_2 + MC_1;
  TA0CCR0 = 1000;

  // To Do: Set P1.0 to output direction. This will help us use the green LED for troubleshooting.
  P1DIR |= BIT0;

  // To Do: Clear all Port 1 outputs.
  P1OUT &= ~BIT0;

  //duty cycle need to be originally 0 or 100, channel 1 PWM, chanel 2 ground,
  //we dont want the heating and cooling begin the moment we turn on the systme 
}
//--------------------------------------------------------
// Initializing the 10-bit ADCModule
//--------------------------------------------------------
void Init_ADC(void) {
  /*
   * To Do: Set up the ADC10CTL1 register to use CONSEQ_2 (Mode 2 - Repeat
   * single channel) and select ADC10 on P1.4.
   */
  ADC10CTL1 = CONSEQ_2 | INCH_4;
  // To Do: Set up the ADC10AE0 register to enable ADC10 on P1.4
  ADC10AE0 = BIT4;
  /* 
   * To Do: Set up the ADC10CTL0 register and select 4 ADC10CLK cycles
   * sample-and-hold time, select multiple sample and conversion (MSC), and turn
   * on the ADC10.
   */
  ADC10CTL0 |= ADC10SHT_0 | MSC | ADC10ON;
  // To Do: Clear ADC10 Interrupt Flag. 
  ADC10CTL0 &= ~ADC10IFG;
  // To Do: Set up the ADC10CTL0 register to Start Conversion and Enable Conversion.
  ADC10CTL0 |= ADC10SC | ENC;
}
//--------------------------------------------------------
// UARTModule
//--------------------------------------------------------
void Init_UART(void) {
  // Initialize the USCI
  // RXD is on P1.1
  // TXD is on P1.2
  // To Do: Configure P1.2 and P1.2 for secondary peripheral function
  P1SEL |= BIT1 | BIT2;      // Enable peripheral function
  P1SEL2 |= BIT1 | BIT2;     // Select secondary peripheral (UART)
  /*
  To Do: Set the prescalers to hold data with 115200 baud rate.
  Because we have a 16MHz  clock, operating at 115200 baud rate will mean
  16MHz/115200 ~ 138. The 16-bit value of (UCAxBR0 + UCAxBR1 × 256) forms the
  prescaler value. See table 15-4 from the the user's guide.
  */
  UCA0BR0 = 138; 
  UCA0BR1 = 0;
  /*
   * To Do: Since the step above (16MHz/115200) results in a non-integer, 
   * to improve the accuracy, set UCA0MCTL to UCBRS_7.
   * See table 15-4 from the the user's guide.
   */
  UCA0MCTL |= UCBRS_7;
  // To Do: Set up the UCA0CTL1 register to use the SMCLK.
  UCA0CTL1 |= UCSSEL_2; //%%do i need UCSSEL1 or 2
  // To Do: Set up the UCA0CTL1 to release UART RESET
  UCA0CTL1 &= ~UCSWRST;  // Reset the UART
}
//--------------------------------------------------------
// ISRs
//--------------------------------------------------------
/* 
 * To Do: Write the ISR for USCIAB0RX_VECTOR. 
 * The synchronization character has been received.
 * Read the character from the receive buffer and store it in received_char.
 * Then, change task_ready to 1, to indicate the rest of the process should start.
 */
// ISR declaration and definition here

#pragma vector = USCIAB0RX_VECTOR //Is the if statement necessary here?
__interrupt void TSCIAB0RX_ISR(void)
 {
  // To Do: Read the character from the receive buffer and store it is received_char.
  //1st Time
  received_char = UCA0RXBUF;

  if (task_ready==0) {
    // To Do: Set task_ready to 1 to start the process
    task_ready = 1;

  // This exits low power mode.
    __bic_SR_register_on_exit(LPM0_bits);
  }

  //2nd Time
  else{ //This is creating the PWM
    if(received_char == 0 || received_char == 1)
    {
      sign=received_char;
    }

    else
    {
      cntrlsig=received_char;

      if(sign == 1)
      {
        if(cntrlsig>2)
        {
          P1OUT &= ~DIR1;      // DIR1 = LOW
          P2OUT |= DIR2;       // DIR2 = HIGH
          TA0CCTL1 = OUTMOD_7;
          TA0CCR1=abs(cntrlsig)*1000/255;
        }
        else
        {
          P1OUT &= ~DIR1;      // DIR1 = LOW
          P2OUT |= DIR2;       // DIR2 = HIGH
          TA0CCTL1 = OUTMOD_3;
          TA0CCR1=200;

          /*
          if(vavg>0)
          {
            TA0CCR1=0;
          }
            
          else
          {
            TA0CCR1=300;
          }
          */
        }
      }

      else
      {
        if(cntrlsig>2)
        {
          P1OUT |= DIR1;       // DIR1 = HIGH
          P2OUT &= ~DIR2;      // DIR2 = LOW
          TA0CCTL1 = OUTMOD_3;
          TA0CCR1=abs(cntrlsig)*1000/255;
        }
        else 
        {
          P1OUT |= DIR1;      // DIR1 = HIGH
          P2OUT &= ~DIR2;       // DIR2 = LOW
          TA0CCTL1 = OUTMOD_7;
          TA0CCR1=200;
        }

        /*
        if(vavg>0)
        {
          TA0CCR1=0;
        }
          
        else
        {
          TA0CCR1=300;
        }
        */
      }
      
      /*
      if(sign == 1)
      {
        P1OUT &= ~DIR1;      // DIR1 = LOW (Heating)
        P2OUT |= DIR2;       // DIR2 = HIGH
        TA0CCTL1 = OUTMOD_7;
        TA0CCR1=abs(cntrlsig)*1000/255;
      }

      else
      {
        P1OUT |= DIR1;       // DIR1 = HIGH (cooling)
        P2OUT &= ~DIR2;      // DIR2 = LOW
        TA0CCTL1 = OUTMOD_3;
        TA0CCR1=abs(cntrlsig)*1000/255;
      }

      if(cntrlsig < 3)
      {
          TA0CCR1=0;
      } 
      */
    }
  }
}
/* 
 * To Do: Write the ISR for ADC10_VECTOR. 
 * This takes care of the analog to digital conversion of data.
 * Once analog to digital conversion is done, it should enable the transmission interrupt. 
 */
//ISR declaration and definition here 

#pragma vector = ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
  // To Do: Turn on the green LED to help with troubleshooting.
  P1OUT |= BIT0;
  // Declaring a variable for a loop.
  unsigned int i;
  /* To Do: Write a for loop and store the ADC10MEM values in the global
   * variable you declared in the beginning. Remember to perform bit-shifting
   * to transfer most of the 10-bit data to your 8-bit variable.
   */
  for(i = 0; i<NPOINTS; i++){
      v[i] = (ADC10MEM >> 2); // Store 8 MSBs (discard 2 LSBs)
      sum += v[i];
  }

  vavg=sum/400;
  sum=0;

  // To Do: Count until NPOINTS) {
    // To Do: Code here

  /*
   * To Do: Enable USCI_A0 TX interrupt (see section 15.4.12 of the user's
   * guide). This (its ISR) will transmit data from MCU to MATLAB.
   */
  IE2 |= UCA0TXIE;
  //Do I need to disable the ADC interupt? No not for this one.
}

/* 
 * To Do: Write the ISR for USCIAB0TX_VECTOR. 
 * This takes care of transmission of data from MCU to MATLAB.
 * Read the character from the receive buffer and store it is received_char.
 * Then, change task_ready to 1, to indicate the rest of the process should start.
 */
//ISR declaration and definition here 

#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
  // To Do: Turn off the green LED to help with troubleshooting.
  P1OUT &= ~BIT0;
  // Declaring a variable for a loop. 
  unsigned int i;
  /* To Do: Write a for loop and transmit the values you stored in the global
   * variable using the transmit buffer.
   */
  for // To Do: Count until NPOINTS) 
  (i=0; i<NPOINTS; i++)
  {
    // To Do: before transmitting the values, make sure they have been received fully.
    while(!(IFG2 & UCA0TXIFG));
    // To Do: Transmit the values through the transmit buffer. 
      UCA0TXBUF = v[i];
    }


  // To Do: Clear the USCI_A0 transmit interrupt enable to avoid excessive triggers.
  IE2 &= ~UCA0TXIE;  //to prevent continuous transmitting
}


