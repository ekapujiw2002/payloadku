/*
 i2c utlity
 */

//lib
#include "i2c_util.h"

//void & func

//read from i2c,
//return 0 = false, 1 = success
unsigned char i2c_read(int i2c_handle, int i2c_device_address, unsigned char i2c_register_address, byte* i2c_data, int i2c_data_len) {
//    unsigned char bx[2];

//    bx[0] = i2c_register_address;
    if (writeBytes(i2c_handle, i2c_device_address, &i2c_register_address, 1) != 1) {
        return 0;
    } else //success
    {
        return readBytes(i2c_handle, i2c_device_address, i2c_data, i2c_data_len) == i2c_data_len;
    }
}

//write to i2c,
//return 0 = false, 1 = success
unsigned char i2c_write(int i2c_handle, int i2c_device_address, unsigned char i2c_register_address, byte* i2c_data, int i2c_data_len) {
    int i;

    //insert addr to data array
    for (i = i2c_data_len; i > 0; i--) {
        *(i2c_data + i) = *(i2c_data + i - 1);
    }
    *i2c_data = i2c_register_address;

    return writeBytes(i2c_handle, i2c_device_address, i2c_data, i2c_data_len + 1) == (i2c_data_len + 1);
}
