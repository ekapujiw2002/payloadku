/*
 i2c driver for gps neo-6m
 */

//lib
#include "i2c_gps.h"

/*
 * read i2c gps status
 * return :
 * 	0 = read fail
 * */
unsigned char i2c_gps_read_status(int i2c_handle)
{
	unsigned char buff[1];
	
	//read status gps
	buff[0] = 0;
	if(!i2c_read(i2c_handle, I2C_GPS_ADDRESS, I2C_GPS_STATUS_00, buff, 1))
		return 0;
		
	return buff[0];
}

/*
 * read i2c gps lat lon
 * return :
 * 	0 = read fail
 * */
unsigned char i2c_gps_read_lat_lon(int i2c_handle, long *latx, long *lonx)
{
	unsigned char buff[8], read_res = 1;
	int i;
	
	//read lat lon gps
	for(i=0;i<8;i++)
	{
		if(!i2c_read(i2c_handle, I2C_GPS_ADDRESS, I2C_GPS_LOCATION+i,&buff[i], 1))
		{
			buff[i] = 0;
			read_res = 0;
		}
	}
	
	//convert ke long, big endian data
	*latx = buff[0] + (buff[1]<<8) + (buff[2]<<16) + (buff[3]<<24);
	*lonx = buff[4] + (buff[5]<<8) + (buff[6]<<16) + (buff[7]<<24); 
		
	return read_res;
}

/*
 * read i2c gps ground 2d speed(m/s), altitude(m)
 * return :
 * 	0 = read fail
 * */
unsigned char i2c_gps_read_speed_altitude(int i2c_handle, unsigned short *speed, unsigned short *altitude)
{
	unsigned char buff[4], read_res = 1;
	int i;
	
	//read lat lon gps
	for(i=0;i<4;i++)
	{
		if(!i2c_read(i2c_handle, I2C_GPS_ADDRESS, I2C_GPS_GROUND_SPEED+i,&buff[i], 1))
		{
			buff[i] = 0;
			read_res = 0;
		}
	}
	
	//convert ke unsigned short, big endian data 
	*speed = buff[0] + (buff[1]<<8); 
	*altitude = buff[2] + (buff[3]<<8); 
		
	return read_res;
}

/*
 * read i2c gps time in msec
 * return :
 * 	0 = read fail
 * */
unsigned char i2c_gps_read_time(int i2c_handle, unsigned long *gps_time)
{
	unsigned char buff[4], read_res = 1;
	int i;
	
	//read lat lon gps
	for(i=0;i<4;i++)
	{
		if(!i2c_read(i2c_handle, I2C_GPS_ADDRESS, I2C_GPS_TIME+i,&buff[i], 1))
		{
			buff[i] = 0;
			read_res = 0;
		}
	}
	
	//convert ke unsigned long, big endian data 
	*gps_time = buff[0] + (buff[1]<<8) + (buff[2]<<16) + (buff[3]<<24);
		
	return read_res;
}
