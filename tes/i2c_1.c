/*
i2c tes
*/

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>

#include "libNativeI2C.h"

//main program
int main(int argc, char** argv)
{
	//open bus i2c
	int i2c_hdl;
	char *i2c_dev = "/dev/i2c-1";
	if((i2c_hdl = openBus(i2c_dev)) < 0)	//fail opening
	{
		perror("Failed opening i2c device");
		exit(1);
	}

	//write register
	unsigned char buff[10];
	buff[0] = 0x31;
	buff[1] = 0x0b;
	if(writeBytes(i2c_hdl, 0x53, buff, 2)>0)
	{
		printf("write 1 ok\n");
		/*
		buff[0] = 0x0b;
		if(writeBytes(i2c_hdl, 0x53, buff, 1)>0)
			printf("write 2 ok\n");
		*/
	}

	//close bus
	closeBus(i2c_hdl);

	return 0;
}
