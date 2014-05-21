#!/bin/bash
rm $1
gcc -Wall -O2 -o $@ `pkg-config --cflags opencv --libs opencv`
