/*
* File:   DriveStepperv2.c
* Author: clairemelvin
*
* Created on April 17, 2013, 3:46 PM
*/

/******************************************************************************
* PREPROCESSOR COMMANDS
******************************************************************************/
#include "NU32.h"          // plib.h, config bits, constants, funcs for startup and UART
#include "drive_motors.h"
#include "feedback.h"
#include "sensorSPI.h"
#include <stdio.h>
#include <stdlib.h>
#define SQRT2 0.7071067811865475244;
#define SQRT3 1.7320508075688772935;
#define SQRT6 2.4494897427831780981;
#define R 7.0;

/******************************************************************************
* FUNCTION PROTOTYPES
******************************************************************************/
int getUserInput();

/******************************************************************************
* GLOBAL VARIABLES
******************** **********************************************************/
//int time;
int motor_1_dir=1;    //CCW = 2, CW = 1
int motor_2_dir=2;    //
int motor_3_dir=2;    //

int motor_1_speed;
int motor_2_speed;
int motor_3_speed;

int CASE=1;

float tot_dist=0;

//system coordinates

//sensor coordinates
//sensor1 (0, 7*sqrt(2)/2, -7*sqrt(2)/2)
float sensor1[3]={0, 4.9497474683058326708,-4.9497474683058326708};
//sensor2 (-7*sqrt(6)/4, 7*sqrt(2)/4, 7sqrt(2)/2)
float sensor2[3]={-4.2866070498705616718, 2.47487373415291633540,-4.9497474683058326708};
//sensor2 (7*sqrt(6)/4, 7*sqrt(2)/4, 7sqrt(2)/2)
float sensor3[3]={4.2866070498705616718, 2.47487373415291633540,-4.9497474683058326708};


//angular velocity basis coordinates
//motor one axis {0 sqrt2/2 sqrt2/2}
float m1axis[3]={0,0.7071067811865475244,0.7071067811865475244};
//motor two axis {sqrt6/4, -sqrt4/2, sqrt2/2}
float m2axis[3]={0.6123724356957945245,-0.3535533905932737622004,0.7071067811865475244};
//motor three axis {-sqrt6/4, -sqrt4/2, sqrt2/2}
float m3axis[3]={-0.6123724356957945245,-0.3535533905932737622004,0.7071067811865475244};



//feedback variables
float error1prev=0; //previous angular velocity along axis of motor 1
float error2prev=0; // ''       ''          ''      ''     of motor 2
float error3prev=0; // ''       ''          ''      ''     of motor 3
float current_w[3]={0,0,0}; //angular velocity reading from sensors
float desired_w[3]={0,0,0}; //desired angular velocity vector
float vx1=0, vy1=0, vx3=0,vy3=0;

//temp variables
int intcount=0;
float x[1000];
float y[1000];
float x1[1000];
float y1a[1000];
int stata1[1000];
int stata2[1000];


/******************************************************************************
* INTERRUPT SERVICE ROUTINES
******************************************************************************/

/***************************** RUN MOTOR 1 *****************************/
void __ISR(_TIMER_3_VECTOR, IPL6SOFT) Timer3ISR(void) {  // 100Hz ISR
   //check where coils are &
   //change coils to next step (forward)

   //run CCW:
   if (motor_1_dir == 2){
  if (LATBbits.LATB12 == LATBbits.LATB14){
   LATBbits.LATB14 = !LATBbits.LATB14;     //(1/2) Switch motor 1 coil 2
   LATBbits.LATB15 = !LATBbits.LATB15;     //(2/2)
   }
  else {
   LATBbits.LATB12 = !LATBbits.LATB12;    //(1/2) Switch motor 1 coil 1
   LATBbits.LATB13 = !LATBbits.LATB13;    //(2/2)
  }}

   //run CW:
   if (motor_1_dir == 1){
  if (LATBbits.LATB12 != LATBbits.LATB14){
   LATBbits.LATB14 = !LATBbits.LATB14;     //(1/2) Switch motor 1 coil 2
   LATBbits.LATB15 = !LATBbits.LATB15;     //(2/2)
   }
  else {
   LATBbits.LATB12 = !LATBbits.LATB12;    //(1/2) Switch motor 1 coil 1
   LATBbits.LATB13 = !LATBbits.LATB13;    //(2/2)
  }}
 IFS0bits.T3IF = 0;                  // clear interrupt flag
 }


/***************************** RUN MOTOR 2 *****************************/
void __ISR(_TIMER_4_VECTOR, IPL6SOFT) Timer4ISR(void) {  // 100Hz ISR
   //check where coils are &
   //change coils to next step (forward)

   //run CCW:
   if (motor_2_dir == 2){
   if (LATBbits.LATB8 == LATBbits.LATB10){
   LATBbits.LATB10 = !LATBbits.LATB10;     //(1/2) Switch motor 1 coil 2
   LATBbits.LATB11 = !LATBbits.LATB11;     //(2/2)
   }
  else {
   LATBbits.LATB8 = !LATBbits.LATB8;    //(1/2) Switch motor 1 coil 1
   LATBbits.LATB9 = !LATBbits.LATB9;    //(2/2)
  }}

   //run CW:
    if (motor_2_dir == 1){
   if (LATBbits.LATB8 == LATBbits.LATB10){
   LATBbits.LATB8 = !LATBbits.LATB8;     //(1/2) Switch motor 1 coil 2
   LATBbits.LATB9 = !LATBbits.LATB9;     //(2/2)
   }
  else {
   LATBbits.LATB10 = !LATBbits.LATB10;    //(1/2) Switch motor 1 coil 1
   LATBbits.LATB11 = !LATBbits.LATB11;    //(2/2)
  }}

 IFS0bits.T4IF = 0;                  // clear interrupt flag
}


/***************************** RUN MOTOR 3 *****************************/
void __ISR(_TIMER_5_VECTOR, IPL6SOFT) Timer5ISR(void) {  // 100Hz ISR
   //check where coils are &
   //change coils to next step (forward)

   //run CCW:
   if (motor_3_dir == 2){
   if (LATBbits.LATB4 == LATBbits.LATB6){
   LATBbits.LATB6 = !LATBbits.LATB6;     //(1/2) Switch motor 1 coil 2
   LATBbits.LATB7 = !LATBbits.LATB7;     //(2/2)
   }
  else {
   LATBbits.LATB4 = !LATBbits.LATB4;    //(1/2) Switch motor 1 coil 1
   LATBbits.LATB5 = !LATBbits.LATB5;    //(2/2)
  }}

   //run CW:
    if (motor_3_dir == 1){
   if (LATBbits.LATB4 == LATBbits.LATB6){
   LATBbits.LATB4 = !LATBbits.LATB4;     //(1/2) Switch motor 1 coil 2
   LATBbits.LATB5 = !LATBbits.LATB5;     //(2/2)
   }
  else {
   LATBbits.LATB6 = !LATBbits.LATB6;    //(1/2) Switch motor 1 coil 1
   LATBbits.LATB7 = !LATBbits.LATB7;    //(2/2)
  }}

 IFS0bits.T5IF = 0;                  // clear interrupt flag
 }

void __ISR(_TIMER_1_VECTOR, IPL7SOFT) Timer1ISR(void) {  // x Hz ISR
    intcount=intcount+1;

    if(intcount==1000){
        if(CASE==1){
        INTDisableInterrupts();
        IEC0bits.T3IE = 0;
        PR4=2000;
        PR5=2000;
        motor_2_dir=1;
        motor_3_dir=2;

        INTEnableSystemMultiVectoredInt();
        intcount=0;
        CASE=2;
        }
        else  {
        INTDisableInterrupts();
        IEC0bits.T3IE = 1;
         PR3=2000;
        PR4=4000;
        PR5=4000;
        motor_1_dir=1;
        motor_2_dir=2;
        motor_3_dir=2;

        INTEnableSystemMultiVectoredInt();

        intcount=0;
        CASE=1;
        }

//    int stat1=0;
//    int stat2=0;
//
//    sense(&stat1,&stat2,&vx1,&vy1,&vx3,&vy3);       //sense velocity on sesnors 1 and 3
//
//    update_w(current_w,vx1,vy1,vy3);
//    x[intcount]=vx1;
//    y[intcount]=vy1;
//    y1a[intcount]=vy3;
//    stata1[intcount]=stat1;
//
//    intcount=intcount+1;
//
//    if(intcount==900){
//
//    PID1(current_w,m1axis, -1, &error1prev, &motor_1_dir);
//    PID2(current_w,m2axis, 0, &error2prev, &motor_2_dir);
//    PID3(current_w,m3axis, 1, &error3prev, &motor_3_dir);
//    int i=0;
//    for(i=0;i<900; i++){
//    char msg[100];
//    sprintf(msg,"PR3,PR4, PR5: %d %d %d %f %f %f \n", stata1[i], stata2[i],x[i], y[i],y1a[i]);
//   NU32_WriteUART1(msg);
//    }
   //intcount=0;
        }
 IFS0bits.T1IF = 0;                  // clear interrupt flag
 }



//1952
/******************************************************************************
* MAIN FUNCTION
******************************************************************************/
int main() {
   NU32_Startup();                    // cache on, min flash wait, interrupts on, LED/button init, UART init
   InitializeMotors();               //Set digital output ports and initialize motor coils
  init_sensorSPI();                //prepare SPI ports for sensing
 //  getUserInput();  //initilize all interupts

   INTDisableInterrupts();

    //use timer2 to measure time between sensor readings
    T2CON = 0x0; // Stop Timer and clear control register,
    // set prescaler at 1:1, internal clock source
    TMR2 = 0x0; // Clear timer register
    PR2 = 0xFFFF; // Load period register
    T2CONSET = 0x8000; // Start Timer

   //configure interupts for motors
   configure_motor1();          //set interupt for motor1
   configure_motor2();          //set interupt for motor2
   configure_motor3();          //set interupt for motor3


   //set sampling rate for sensors
   sensor_samprate(3124);            //set interupt for feedback control timer 1
   write_data(0x00, 0x40);
   WriteCoreTimer(0);
    while(ReadCoreTimer()<80000);
   INTEnableSystemMultiVectoredInt();
   int seq_status=0;                //determine whether tumbler is in a rolling sequence

 
   
  INTDisableInterrupts();
   IEC0bits.T3IE = 1;
     IEC0bits.T4IE = 1;
    IEC0bits.T5IE = 1;

   INTEnableSystemMultiVectoredInt();
   while(1){
       

       
       
   }




   return 0;
}

/******************************************************************************
* HELPER FUNCTIONS
******************************************************************************/


int getUserInput() {

   //ask for user input
   char msg1[100];
   sprintf(msg1, "\n Please enter your desired axis of rotation.(rad/sec) \n");
   NU32_WriteUART1(msg1);
   //scan user input
   char msg2[20];
   float wx,wy,wz;
   NU32_ReadUART1(msg2, 20);
   sscanf(msg2, "%f %f %f", &wx,&wy,&wz);

   desired_w[0]=wx; desired_w[1]=wy; desired_w[2]=wz;





   return 0;
}