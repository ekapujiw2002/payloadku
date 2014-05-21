/* 
 * File:   itg3200.h
 * Author: EX4
 *
 * Created on February 19, 2013, 2:25 PM
 */

#ifndef ITG3200_H
#define	ITG3200_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "i2c_util.h"

// ITG3200 Register Defines
#define WHO		0x00
#define	SMPL	0x15
#define DLPF	0x16
#define INT_C	0x17
#define INT_S	0x1A
#define	TMP_H	0x1B
#define	TMP_L	0x1C
#define	GX_H	0x1D
#define	GX_L	0x1E
#define	GY_H	0x1F
#define	GY_L	0x20
#define GZ_H	0x21
#define GZ_L	0x22
#define PWR_M	0x3E

extern unsigned char itg3200_init(int i2c_handle, int i2c_device_address);
extern unsigned char itg3200_get_gyro_data(int i2c_handle, int i2c_device_address, unsigned int *gix, unsigned int *giy, unsigned int *giz);
extern float itg3200_calc_gyro_value(unsigned int gyrodata);
extern unsigned int itg3200_normalize_gyrodata(float gyrovalue, unsigned int min_norm_value, unsigned int max_norm_value);

#ifdef	__cplusplus
}
#endif

#endif	/* ITG3200_H */
