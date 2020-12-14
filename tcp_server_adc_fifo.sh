#!/bin/sh
# Servces the adc-values from /tmp/adc.fifo on port 2000
cat /tmp/adc.fifo | nc -l -k 2000
