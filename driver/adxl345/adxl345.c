/*
 i2c driver for adxl345
 */

//lib
#include "adxl345.h"

//detail void and func
/*
 * init adxl345 = +-16g, high resolution, isr data ready on
 * return = 0 : fail ; 1 : success
 * */
unsigned char adxl345_init(int i2c_handle, int i2c_device_address)
{
	unsigned char buff[1];
	
	//enable isr
	buff[0] = 0x80;
	if(!i2c_write(i2c_handle, i2c_device_address, 0x2e, buff, 1))
		return 0;
		
	//data format = full res, +-16g
	buff[0] = 0x0b;
	if(!i2c_write(i2c_handle, i2c_device_address, 0x31, buff, 1))
		return 0;
		
	return 1;
}

/*
 * start measurement adxl345
 * */
unsigned char adxl345_start(int i2c_handle, int i2c_device_address)
{
	unsigned char buff[1];
	
	//enable isr
	buff[0] = 0x08;
	return i2c_write(i2c_handle, i2c_device_address, 0x2d, buff, 1);
}

/*
 * get data from adxl
 * */
unsigned char adxl345_get_gdata(int i2c_handle, int i2c_device_address, unsigned int *gx, unsigned int *gy, unsigned int *gz)
{
	unsigned char buff[6];
	
	//init result
	*gx = *gy = *gz = 0;
	
	//read status
	if(!i2c_read(i2c_handle, i2c_device_address, 0x30, buff, 1))
		return 0;
		
	//check ready bit
	if((buff[0] & 0x80) == 0x80)
	{
		//read data
		if(!i2c_read(i2c_handle, i2c_device_address, 0x32, buff, 6))
			return 0;
			
		//convert data
		*gx = buff[0]+(buff[1]<<8);
		*gy = buff[2]+(buff[3]<<8);
		*gz = buff[4]+(buff[5]<<8);
	}
	
	return 1;
}

/*
 * convert adxl345 g data to g value
 * */
float adxl345_calc_gvalue(unsigned int gdata)
{
	//undefine
	//if((gdata>0x0fff) && (gdata<0xf000))
	//	return 0;
		
	//minus g
	if((gdata>=0xf000) && (gdata<=0xffff))
		return (float) -1*((0xffff - gdata + 1)<<4) / 4095;
		
	//plus g
	if((gdata>=0x0000) && (gdata<=0x0fff))
		return (float) (gdata<<4) / 4095;

	//undefine
	return 0;
}

/*
 * normalize g data to specified range
 * */
unsigned int adxl345_normalize_gdata(float gvalue, unsigned int min_norm_value, unsigned int max_norm_value)
{		
	//normalize
	return (unsigned int) (min_norm_value<max_norm_value) ? (((gvalue+16)/(16+16))*(max_norm_value-min_norm_value))+min_norm_value : 0;
}
