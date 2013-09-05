#include <plib.h>
#include "drive_motors.h"





void InitializeMotors(){

   TRISBCLR = 0XFFFF;        /* PORT B12-B15 output */
  // TRISDCLR = 0XFFFF;        /* PORT D0-D3 & D8-D11 output */

       LATBbits.LATB12 = 1;        //Set Motor 1 coil 1
       LATBbits.LATB13 = 0;        //
       LATBbits.LATB14 = 1;        //Set Motor 1 coil 2
       LATBbits.LATB15 = 0;        //

       LATBbits.LATB8  = 1;        //Set Motor 2 coil 1
       LATBbits.LATB9  = 0;        //
       LATBbits.LATB10  = 1;        //Set Motor 2 coil 2
       LATBbits.LATB11  = 0;        //

       LATBbits.LATB4  = 1;        //Set Motor 3 coil 1
       LATBbits.LATB5  = 0;        //
       LATBbits.LATB6 = 1;        //Set Motor 3 coil 2
       LATBbits.LATB7 = 0;        //
       
       return;
}

void configure_motor1(){
 
 INTDisableInterrupts();             // INT step 2: disable interrupts at CPU
 PR3 = 2000;                        // INT step 3: setup peripheral: set period register
 TMR3 = 0;                           //             initialize count to 0
 T3CONbits.TCKPS = 7;                //             set prescaler to 1
 T3CONbits.TGATE = 0;                //             not gated input (the default)
 T3CONbits.TCS = 0;                  //             PCBLK input (the default)
 T3CONbits.ON = 1;                   //             turn on Timer1
 IPC3bits.T3IP = 6;                  // INT step 4: priority
 IPC3bits.T3IS = 1;                  //             subpriority
 IFS0bits.T3IF = 0;                  // INT step 5: clear interrupt flag
 IEC0bits.T3IE = 1;                  // INT step 6: enable interrupt
 INTEnableSystemMultiVectoredInt();  // INT step 7: enable interrupts at CPU
}

void configure_motor2(){

   //Timer 4/Motor 2 Interrupt:
 INTDisableInterrupts();             // INT step 2: disable interrupts at CPU
 PR4 = 4000; // INT step 3: setup peripheral: set period register
 TMR4 = 0;                           //             initialize count to 0
 T4CONbits.TCKPS = 7;                //             set prescaler to 256
 T4CONbits.TGATE = 0;                //             not gated input (the default)
 T4CONbits.TCS = 0;                  //             PCBLK input (the default)
 T4CONbits.ON = 1;                   //             turn on Timer2
 IPC4bits.T4IP = 6;                  // INT step 4: priority
 IPC4bits.T4IS = 2;                  //             subpriority
 IFS0bits.T4IF = 0;                  // INT step 5: clear interrupt flag
 IEC0bits.T4IE = 1;                  // INT step 6: enable interrupt
 INTEnableSystemMultiVectoredInt();  // INT step 7: enable interrupts at CPU
  
}

void configure_motor3(){

   //Timer 4/Motor 2 Interrupt:
 INTDisableInterrupts();             // INT step 2: disable interrupts at CPU
 PR5= 4000; // INT step 3: setup peripheral: set period register
 TMR5 = 0;                           //             initialize count to 0
 T5CONbits.TCKPS = 7;                //             set prescaler to 256
 T5CONbits.TGATE = 0;                //             not gated input (the default)
 T5CONbits.TCS = 0;                  //             PCBLK input (the default)
 T5CONbits.ON = 1;                   //             turn on Timer2
 IPC5bits.T5IP = 6;                  // INT step 4: priority
 IPC5bits.T5IS = 2;                  //             subpriority
 IFS0bits.T5IF = 0;                  // INT step 5: clear interrupt flag
 IEC0bits.T5IE = 1;                  // INT step 6: enable interrupt
 INTEnableSystemMultiVectoredInt();  // INT step 7: enable interrupts at CPU

}