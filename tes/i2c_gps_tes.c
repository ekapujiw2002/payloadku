/*
 * i2c gps test mode
 * compile command :
 * gcc -Wall -O2 -o i2cx *.c -I/usr/local/include -L/usr/local/lib
 * */

//standar lib
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/unistd.h>

#ifdef __linux__
//~ #include <wiringPi.h>
//#include <wiringSerial.h>

#include "libNativeI2C.h"
#include "i2c_gps.h"

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

//gps data struct
typedef struct {
  long      lat;            //degree*10 000 000
  long      lon;            //degree*10 000 000
} GPS_COORDINATES;

typedef struct {
	unsigned char gps_status;
	GPS_COORDINATES gps_loc;
	unsigned short gps_speed, gps_altitude;
	unsigned long gps_time;  
} GPS_DATA;

/*
 * var and constant
 * */
#define i2c_dev			"/dev/i2c-1"
#define	i2c_gps_addr	0x20

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
	//~ GPS_COORDINATES gps_posx;
	GPS_DATA gpsdtx;
	
	//init bus i2c
	int i2c_hdl = I2C_Init_Bus(i2c_dev, &statusPeripheral);
	
	//read status
	if(!(gpsdtx.gps_status = i2c_gps_read_status(i2c_hdl)))
	{
		printf("read gps status fail\n");
	}
	
	//read speed and altitude
	if(!i2c_gps_read_speed_altitude(i2c_hdl, &gpsdtx.gps_speed, &gpsdtx.gps_altitude))
	{
		printf("read gps speed and altitude fail\n");
	}
	
	//read lat lon
	if(!i2c_gps_read_lat_lon(i2c_hdl, &gpsdtx.gps_loc.lat, &gpsdtx.gps_loc.lon))
	{
		printf("read gps loc fail\n");
	}
	
	//read time
	if(!i2c_gps_read_time(i2c_hdl, &gpsdtx.gps_time))
	{
		printf("read gps time fail\n");
	}
	
	printf("(lat,lon) = %f, %f ( %08lx, %08lx )\n", (float) gpsdtx.gps_loc.lat / 1e7, (float) gpsdtx.gps_loc.lon / 1e7, gpsdtx.gps_loc.lat, gpsdtx.gps_loc.lon);
	printf("s = %d\nv = %d m/s\nh = %d m\ntime = %ld\n", gpsdtx.gps_status, gpsdtx.gps_speed, gpsdtx.gps_altitude, gpsdtx.gps_time);
	
	//close bus
	closeBus(i2c_hdl);
	
	//return
    return (EXIT_SUCCESS);
}
