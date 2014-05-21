/*
 i2c driver for itg3200
 */

//lib
#include "itg3200.h"

//detail void and func
/*
 * init itg3200
 * 
 * */
unsigned char itg3200_init(int i2c_handle, int i2c_device_address)
{
	unsigned char buff[1];
	
	//reset to default
	buff[0] = 0x80;
	if(!i2c_write(i2c_handle, i2c_device_address, PWR_M, buff, 1))
		return 0;
		
	//sample divider=0
	buff[0] = 0x00;
	if(!i2c_write(i2c_handle, i2c_device_address, SMPL, buff, 1))
		return 0;
		
	//full scale cfg
	buff[0] = 0x18;
	if(!i2c_write(i2c_handle, i2c_device_address, DLPF, buff, 1))
		return 0;
		
	//int config
	buff[0] = 0x05;
	if(!i2c_write(i2c_handle, i2c_device_address, INT_C, buff, 1))
		return 0;
		
	//reset hw
	buff[0] = 0x00;
	return i2c_write(i2c_handle, i2c_device_address, PWR_M, buff, 1);
}

/*
 * get data from itg
 * */
unsigned char itg3200_get_gyro_data(int i2c_handle, int i2c_device_address, unsigned int *gix, unsigned int *giy, unsigned int *giz)
{
	unsigned char buff[6];
	
	//init result
	*gix = *giy = *giz = 0;
	
	//read status
	if(!i2c_read(i2c_handle, i2c_device_address, INT_S, buff, 1))
		return 0;
		
	//check ready bit
	if((buff[0] & 0x01) == 0x01)
	{
		//read data
		if(!i2c_read(i2c_handle, i2c_device_address, GX_H, buff, 6))
			return 0;
			
		//convert data
		*gix = buff[1]+(buff[0]<<8);
		*giy = buff[3]+(buff[2]<<8);
		*giz = buff[5]+(buff[4]<<8);
	}
	
	return 1;
}

/*
 * convert itg3200 gyro data to g value
 * */
float itg3200_calc_gyro_value(unsigned int gyrodata)
{		
	//minus g
	if((gyrodata>=0x8000) && (gyrodata<=0xffff))
		return (float) -1*((0xffff - gyrodata + 1)*2000) / 32767;
		
	//plus g
	if((gyrodata>=0x0000) && (gyrodata<=0x7fff))
		return (float) (gyrodata*2000) / 32767;
		
	//undefine
	return 0;
}

/*
 * normalize gyro data to specified range
 * */
unsigned int itg3200_normalize_gyrodata(float gyrovalue, unsigned int min_norm_value, unsigned int max_norm_value)
{		
	//normalize
	return (unsigned int) (min_norm_value<max_norm_value) ? (((gyrovalue+2000)/(2000+2000))*(max_norm_value-min_norm_value))+min_norm_value : 0;
}
