#!/bin/bash

avr-gcc -mmcu=attiny13a -Os -DF_CPU=9600000UL *.c -o main.o

avr-objcopy -j .text -j .data -O ihex main.o main.hex
avrdude -c usbasp -p t13 -U flash:w:main.hex 

rm main.hex
rm main.o