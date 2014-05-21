/*
 * tes rs232 demo
 * compile :
 * gcc -Wall -O2 -o tes_rs232 tes_rs232.c ../driver/rs232/rs232.c -I../driver/rs232/
 * */

//standar lib
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/unistd.h>

//rs232 lib
#include "rs232.h"

//delay buffer rf module
#define	RF_DELAY_US				4000
#define RF_DELAY_LESS_PACKET_US	35000
#define RF_DELAY_HALF_DUPLEX_US	8000

//com port
#define COM_BUFF_NUM    20
#define COM_SPEED       57600

#ifdef __linux__
#define COM_PORT        22        //22 utk ttyAMA0
#else
#define COM_PORT        0
#endif

/*
 * main program
 */
int main(int argc, char** argv) {
	
	unsigned char buffdata[150000];
	
	//open port
    unsigned char buffRX[COM_BUFF_NUM];
    unsigned int buffNumRX;
    unsigned int delayPacket = atoi(argv[2]);
    
    //make it as null string
    buffRX[COM_BUFF_NUM-1] = 0;

    if (OpenComport(COM_PORT, COM_SPEED)) {
        printf("Open port %d failed : %s\r\n!!!", COM_PORT, strerror(errno));
        return errno;
    }
    
    usleep(500000);
    //susun data
    for(buffNumRX=0;buffNumRX<atoi(argv[1]);buffNumRX++)
    {
		buffdata[buffNumRX] = (buffNumRX%10) + 0x30;
	}
    
    for(buffNumRX=0;buffNumRX<atoi(argv[1]);buffNumRX++)
    {
		SendByte(COM_PORT, buffdata[buffNumRX]);
		if(buffNumRX%31==0) usleep(delayPacket);
	}
	usleep(500000);
    
    //release port
    CloseComport(COM_PORT);
	
	return EXIT_SUCCESS;
}
