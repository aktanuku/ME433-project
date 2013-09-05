#include"feedback.h"
#include <plib.h>
float proj(float v1[], float v2[], float normv2){
        //calculates projection of vector v1 on vector v2
        //v1 is the measured angular velocity
        //v2 is a basis vector of our system

    float dot=v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2];

    float proj=dot/normv2;

    return (proj);

}
void update_w( float w[],float v1x, float v1y, float v3y){
    //computes angular velocity vector from sensor velocity readings


    //save old values


    //transformation matrix see documentation for definition
    float A1[3]={ 0, -0.14285714285714285714, 0};
    float A2[3]={0,0.0824786098842322520,-0.16495721976846450414547};
    float A3[3]={0.20203050891044214982,-0.0824786098842322520,-0.16495721976846450414547};

    //compute your angular velocity vector
    w[0]=A1[0]*v1x+A1[1]*v1y+A1[2]*v3y;
    w[1]=A2[0]*v1x+A2[1]*v1y+A2[2]*v3y;
    w[2]=A3[0]*v1x+A3[1]*v1y+A3[2]*v3y;








}

int setspeed(int* dir, float Control){
    int speed=0;
    if(Control<0){
        Control=-Control;

        speed=.0314/(256*Control*(.0000000125))-1;

        if(speed<1000){
            speed=1000;

        }

        if(speed>3000){
            speed=3000;
        }


        *dir=1;
    }
    else{
         speed=.0314/(256*Control*(.0000000125))-1;

        if(speed<1000){
            speed=1000;

        }

       // if(speed>3000){
         //   speed=3000;
        //}



        *dir=2;

    }

    return speed;
};

void PID1(float w[],float base[], float dp, float* errorprev, int *dir){
    //PID for motor 1


    float p1=proj(w,base,1);

    float error= p1-dp;

    float kp=.005;

    float kd=0;
    float Control= kp*(error)+kd*(error-*errorprev);
    int speed=setspeed(dir,Control);

    PR3=speed;

    *errorprev=error;



}

void PID2(float w[],float base[], float dp, float* errorprev, int *dir){
    //PID for motor 2
    float p2=proj(w,base,1);

    float error= p2-dp;

    float kp=.005;

    float kd=0;
    float Control= kp*(error)+kd*(error-*errorprev);
    int speed=setspeed(dir,Control);

    PR4=speed;

    *errorprev=error;
}

void  PID3(float w[],float base[], float dp, float* errorprev, int *dir){
    //PID for motor 3
    float p3=proj(w,base,1);

    float error= p3-dp;

    float kp=.005;

    float kd=0;
    float Control= kp*(error)+kd*(error-*errorprev);
    int speed=setspeed(dir,Control);

    PR5=speed;

    *errorprev=error;
}