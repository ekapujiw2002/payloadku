#!/bin/bash
rm $1
#gcc -Wall -O2 -o $@ `pkg-config --cflags opencv --libs opencv` \
gcc -Wfatal-errors -O2 -o $@ `pkg-config --cflags opencv --libs opencv` \
driver/i2c/libNativeI2C.c \
-Idriver/i2c \
driver/i2c_util/i2c_util.c \
-Idriver/i2c_util \
driver/adxl345/adxl345.c \
-Idriver/adxl345 \
driver/itg3200/itg3200.c \
-Idriver/itg3200 \
driver/rs232/rs232.c \
-Idriver/rs232 \
driver/ini/iniparser.c \
driver/ini/dictionary.c \
driver/i2c_gps/i2c_gps.c \
-Idriver/i2c_gps \
-Idriver/ini \
-I/usr/local/include \
-Ldriver/ini \
-L/usr/local/lib \
-lwiringPi \
-liniparser
