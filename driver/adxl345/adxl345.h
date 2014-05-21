/* 
 * File:   adxl345.h
 * Author: EX4
 *
 * Created on February 19, 2013, 1:40 PM
 */

#ifndef ADXL345_H
#define	ADXL345_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "i2c_util.h"

/*
 * proto void function
 * */
extern unsigned char adxl345_init(int i2c_handle, int i2c_device_address);
extern unsigned char adxl345_start(int i2c_handle, int i2c_device_address);
extern unsigned char adxl345_get_gdata(int i2c_handle, int i2c_device_address, unsigned int *gx, unsigned int *gy, unsigned int *gz);
extern float adxl345_calc_gvalue(unsigned int gdata);
extern unsigned int adxl345_normalize_gdata(float gvalue, unsigned int min_norm_value, unsigned int max_norm_value);

#ifdef	__cplusplus
}
#endif

#endif	/* ADXL345_H */

