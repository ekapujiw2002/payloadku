/* 
 * File:   i2c_gps.h
 * Author: EX4
 *
 * Created on February 20, 2014, 8:06 AM
 */

#ifndef I2C_GPS_H
#define	I2C_GPS_H

#ifdef	__cplusplus
extern "C" {
#endif

//lib
#include "i2c_util.h"

//alamat i2c gps
#define I2C_GPS_ADDRESS						0x20                      //7 bit address 0x40 write, 0x41 read

//alamat data GPS
#define I2C_GPS_STATUS_00						00 		//(Read only)
        #define I2C_GPS_STATUS_NEW_DATA       0x01      // New data is available (after every GGA frame)
        #define I2C_GPS_STATUS_2DFIX          0x02      // 2dfix achieved
        #define I2C_GPS_STATUS_3DFIX          0x04      // 3dfix achieved
        #define I2C_GPS_STATUS_WP_REACHED     0x08      // Active waypoint has been reached (not cleared until new waypoint is set)
        #define I2C_GPS_STATUS_NUMSATS        0xF0      // Number of sats in view
        
#define I2C_GPS_LOCATION						07   // current location 8 byte (lat, lon) int32_t
#define I2C_GPS_NAV_LAT							15   // Desired banking towards north/south int16_t
#define I2C_GPS_NAV_LON							17   // Desired banking toward east/west    int16_t
#define I2C_GPS_WP_DISTANCE						19   // Distance to current WP in cm uint32
#define I2C_GPS_WP_TARGET_BEARING				23   // bearing towards current wp 1deg = 1000 int16_t
#define I2C_GPS_NAV_BEARING						25   // crosstrack corrected bearing towards current wp 1deg = 1000 int16_t
#define I2C_GPS_HOME_TO_COPTER_BEARING			27   // bearing from home to copter 1deg = 1000 int16_t
#define I2C_GPS_DISTANCE_TO_HOME				29   // distance to home in m int16_t
        
#define I2C_GPS_GROUND_SPEED					31   // GPS ground speed in m/s*100 (uint16_t)      (Read Only)
#define I2C_GPS_ALTITUDE						33   // GPS altitude in meters (uint16_t)           (Read Only)
#define I2C_GPS_GROUND_COURSE					35   // GPS ground course (uint16_t)
#define I2C_GPS_RES1							37   // reserved for future use (uint16_t)
#define I2C_GPS_TIME							39   // UTC Time from GPS in hhmmss.sss * 100 (uint32_t)(unneccesary precision) (Read Only)    

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
 * proto function
 * */    
extern unsigned char i2c_gps_read_status(int i2c_handle);
extern unsigned char i2c_gps_read_lat_lon(int i2c_handle, long *latx, long *lonx);
extern unsigned char i2c_gps_read_speed_altitude(int i2c_handle, unsigned short *speed, unsigned short *altitude);
extern unsigned char i2c_gps_read_time(int i2c_handle, unsigned long *gps_time);

#ifdef	__cplusplus
}
#endif

#endif	/* I2C_GPS_H */

