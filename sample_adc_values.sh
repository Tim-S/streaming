#!/bin/sh
# read values of one adc (ch0 only) in blocks of 256 at a sample rate of 2000 and pipe them through a fifo at /tmp/adc.fifo 
sudo ./rpi_adc_stream -n 256 -r 2000 -i 1 -s /tmp/adc.fifo
