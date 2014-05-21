/*
 * hm trp radio config
 * compile with :
 * gcc -Wall -O2 -o trp_cfg trp_cfg.c driver/rs232/rs232.c -Idriver/rs232
 * 
 * */
//standar lib
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/unistd.h>

//rs232 lib
#include "rs232.h"

//com port
#define COM_BUFF_NUM    100
#define COM_SPEED       57600

#ifdef __linux__
#define COM_PORT        22        //22 utk ttyAMA0
#else
#define COM_PORT        0
#endif

#define	CMD_READ_CFG	"\xaa\xfa\xe1"
#define CMD_SET_RF_115200	"\xaa\xfa\xc3\x00\x01\xC2\x00"
#define CMD_SET_RF_57600	"\xaa\xfa\xc3\x00\x00\xe1\x00"

/*
 * main program
 * */
int main(int argc, char** argv) {
	
	//open port
    unsigned char buffRX[COM_BUFF_NUM];
    int buffNumRX, i;
    
    //~ //check argument
    //~ if(argc==1)
    //~ {
		//~ printf("Enter command\r\n");
		//~ return(EXIT_SUCCESS);
	//~ }
	//~ 
	//~ printf("cmd = %s\r\n", argv[1]);
	
	if (OpenComport(COM_PORT, COM_SPEED)) {
        printf("Open port %d failed : %s\n", COM_PORT, strerror(errno));
        return errno;
    }
    printf("Open port %d OK \x30 \n", COM_PORT);
    
    //send command
    sleep(1);
    SendBuf(COM_PORT,(unsigned char*) CMD_SET_RF_57600, 7); 
    sleep(1);
    SendBuf(COM_PORT,(unsigned char*) CMD_READ_CFG, 3); 
    
    while(1)
    {
		//ambil perintah dari comport
		buffNumRX = PollComport(COM_PORT, buffRX, (COM_BUFF_NUM - 2));
    
		if(buffNumRX>0)
		{
			for(i=0;i<buffNumRX;i++)
			{
				printf("%02x ", buffRX[i]);
			}
			printf("\n");
		}
	}
    
    //release port
    CloseComport(COM_PORT);
	
	return(EXIT_SUCCESS);
}
