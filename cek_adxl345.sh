#!/bin/bash
echo 0x2d = 
sudo i2cget -y 1 0x53 0x2d b
echo 0x2e = 
sudo i2cget -y 1 0x53 0x2e b
echo 0x30 = 
sudo i2cget -y 1 0x53 0x30 b
echo 0x31 = 
sudo i2cget -y 1 0x53 0x31 b