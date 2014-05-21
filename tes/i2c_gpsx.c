/*
 * i2c gps test mode
 * compile command :
 * gcc -Wall -O2 -o i2c_gps i2c_gps.c driver/i2c/libNativeI2C.c -Idriver/i2c driver/i2c_util/i2c_util.c
 * */

//standar lib
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/unistd.h>

#ifdef __linux__
#include <wiringPi.h>
//#include <wiringSerial.h>

#include "libNativeI2C.h"
//#include "adxl345.h"
//#include "itg3200.h"
#endif

//periferal bit status
typedef enum {
	BIT_CAM_READY,
	BIT_I2C_BUS_READY,
	BIT_ADXL345_READY,
	BIT_ITG3200_READY
} STATUS_PERIPH_ENUM;

/*
 * var and constant
 * */
#define i2c_dev			"/dev/i2c-1"

/*
 * init i2c bus
 * */
int I2C_Init_Bus(char *dev_i2c_name, unsigned char *statusPeripheralOutput)
{
	int i2c_hdlx;
	
	if((i2c_hdlx = openBus(dev_i2c_name)) < 0)	//fail opening
	{
		printf("Failed opening i2c device");
		//return errno;
		*statusPeripheralOutput &= ~(1<<BIT_I2C_BUS_READY);
	}
	else
	{
		*statusPeripheralOutput |= (1<<BIT_I2C_BUS_READY);
	}
	
	return i2c_hdlx;
}

/*
 * main program
 */
int main(int argc, char** argv) {
	//local var
	unsigned char statusPeripheral = 0;
	
	printf("going to init i2c bus\n");
	
	//init bus i2c
	int i2c_hdl = I2C_Init_Bus(i2c_dev, &statusPeripheral);
	
	printf("close i2c bus\n");
	
	//close bus
	closeBus(i2c_hdl);
	
	//return
    return (EXIT_SUCCESS);
}
