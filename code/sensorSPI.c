#include"sensorSPI.h"

void sensor_samprate(int rate){     //ISR for this interupt is in the feedback library
                                       // INT Step 3: set up peripheral
  PR1 = rate;                       // set period register
  TMR1 = 0;                         //initalize timer 1 to 0
  T1CONbits.TCKPS = 0b11;              // prescaler 256
  T1CONbits.TGATE = 0;              // not gated input (default)
  T1CONbits.TCS = 0;                //PBLK clock input (default)
  T1CONbits.ON =1;                  // turn on Timer 1
  IPC1bits.T1IP =7;                 //INT step 4: priority
  IPC1bits.T1IS =0;                  // subpriortity
  IFS0bits.T1IF=0;                   // INT step 5: clear interupt flag
  IEC0bits.T1IE=1;                  // INT step 6: enable interrupt


}

void init_sensorSPI(void){

    //initialize SPI for Sensor 1
    SPI1CON = 0;
    SPI1BRG = 31;
    SPI1STATCLR=0x40;
    SPI1CON=0x100082E0;// MSSEN ON to enable SS, SPI ON, 8 bit xfer, SMP=1, Master Mode

    

    //initalize SPI for sensor 3
   SPI4CON = 0;
    SPI4BRG = 31;
   SPI4STATCLR=0x40;

   SPI4CON=0x100082E0;// MSSEN ON to enable SS, SPI ON, 8 bit xfer, SMP=1, Master Mode

}


void read_addr(char addr ){
    char data;
    SPI1CONbits.DISSDO=0;           //output on
    SPI1BUF=addr;                   //sensor address
    while(!SPI1STATbits.SPIRBF);    //wait for SPI to finish

    data=SPI1BUF;                   //read garbage

    

    
    SPI4CONbits.DISSDO=0;           //output on
    SPI4BUF=addr;                   //sensor address
    while(!SPI4STATbits.SPIRBF);    //wait for SPI to finish trans

    data=SPI4BUF;                   //read garbage
}

char recieve_data1(){
    char value;
    // turn off output at SPI out pins and change them to floating
    SPI1CONbits.DISSDO=1;       //output off
    TRISDbits.TRISD0=1;         //sensor 1 SPI output pin to hi-Z

    SPI1BUF= 0x00;              //transmit garbage
    while(!SPI1STATbits.SPIRBF){//wait to finish transmitting

    }
    value=SPI1BUF;              //read value from recieve buffer
    return(value);

}

char recieve_data2(){

    return(0);

}

char recieve_data3(){
    char value;
    // turn off output at SPI out pins and change them to floating

    SPI4CONbits.DISSDO=1;
    TRISFbits.TRISF5=1;         //sensor 3 SPI output pin to hi-Z

    SPI4BUF= 0x00;              //transmit garbage
    while(!SPI4STATbits.SPIRBF){//wait to finish

    }
    value=SPI4BUF;              //read buffer
    return(value);

}


void write_data(char addr, char val){
    char data;

    //sensor 1 write
    SPI1BUF=addr;
    while(!SPI1STATbits.SPIRBF);
    data=SPI1BUF;

    SPI1BUF=val;
    while(!SPI1STATbits.SPIRBF);
    data=SPI1BUF;

    
    //sensor 3 write
    SPI4BUF=addr;
    while(!SPI4STATbits.SPIRBF);
    data=SPI4BUF;

    SPI4BUF=val;
   while(!SPI4STATbits.SPIRBF);
    data=SPI4BUF;



}




// send a 1 to the dsPIC to ask for the current encoder count
void sense(int* stat1, int* stat2, float* vx1, float* vy1, float* vx3, float* vy3 ) {
    char data1, data2, data3;
    int dt=0, dx1=0, dx3=0, dy1=0, dy3=0;
    read_addr(0x16);//check overflow

    WriteCoreTimer(0);              //100 us delay
    while(ReadCoreTimer()<80000);

    data1=recieve_data1();
    data3=recieve_data3();

    if((data1==-128)&&(data3==-128)){
        //update dx
        read_addr(0x02);

        WriteCoreTimer(0);              //100 us delay
        while(ReadCoreTimer()<80000);

        dx1=recieve_data1();
        if(dx1>127)
        dx1=dx1-256;
    

    
        dx3=recieve_data3();
        if(dx3>127)                     //convert from 2s complement
        dx3=dx3-256;
    


    WriteCoreTimer(0);              //100 us delay
    while(ReadCoreTimer()<80000);

 
    //update dy
        read_addr(0x03);

        WriteCoreTimer(0);              //100 us delay
        while(ReadCoreTimer()<80000);

        dy1=recieve_data1();
        if(dy1>127)                     //convert from 2s complement
        dy1=dy1-256;
    

   

        dy3=recieve_data3();
        if(dy3>127)                     //convert from 2s complement
        dy3=dy3-256;
     
    dt=TMR2;                        // time since last read
    TMR2CLR=0xFFFF;

    //convert to cm/s
    *stat1=data1;
    *stat2=data3;
    *vx1=(2.54*(float)dx1/(float)1000)/((float) dt * (2.0)/((float) 80000000));
   *vy1=(2.54*(float)dy1/(float)1000)/((float) dt * (2.0)/((float) 80000000));
    *vx3=(2.54*(float)dx3/(float)1000)/((float) dt * (2.0)/((float) 80000000));
    *vy3=(2.54*(float)dy3/(float)1000)/((float) dt * (2.0)/((float) 80000000));
    //char msg[100];
   // sprintf(msg,"status: %d %d \n", data1, data3);
  // NU32_WriteUART1(msg);

    }

    if(data1==0){
        *vx1=0;
        *vy1=0;
    }
    if(data3==0){
        *vx3=0;
        *vy3=0;
    }

    
   
}




