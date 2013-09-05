/* 
 * File:   sensorspi.h
 * Author: Mechatronics
 *
 * Created on May 27, 2013, 2:55 PM
 */
#include<plib.h>
#ifndef SENSORSPI_H
#define	SENSORSPI_H

void sensor_samprate(int rate);

void init_sensorSPI(void);


void read_addr(char addr );
char recieve_data1();
char recieve_data2();

char recieve_data3();


void write_data(char addr, char val);




// send a 1 to the dsPIC to ask for the current encoder count
void sense(int *stat1, int *stat2, float* vx1, float* vy1, float* vx3, float* vy3 ) ;




#endif	/* SENSORSPI_H */

