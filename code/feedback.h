#ifndef FEEBACK_H
#define  FFEDBACK_H

float proj(float v1[], float v2[], float normv2);
void update_w( float w[],float v1x, float v1y, float v3y);

int setspeed(int* dir, float Control);
void PID1(float w[],float base[], float dp, float* errorprev, int *dir);
void PID2(float w[],float base[], float dp, float* errorprev, int *dir);
void PID3(float w[],float base[], float dp, float* errorprev, int *dir);


#endif