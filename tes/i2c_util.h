/* 
 * File:   i2c_util.h
 * Author: EX4
 *
 * Created on February 19, 2013, 2:25 PM
 */

#ifndef I2C_UTIL_H
#define	I2C_UTIL_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "libNativeI2C.h"
    
extern unsigned char i2c_read(int i2c_handle, int i2c_device_address, unsigned char i2c_register_address, byte* i2c_data, int i2c_data_len);
extern unsigned char i2c_write(int i2c_handle, int i2c_device_address, unsigned char i2c_register_address, byte* i2c_data, int i2c_data_len);


#ifdef	__cplusplus
}
#endif

#endif	/* I2C_UTIL_H */
