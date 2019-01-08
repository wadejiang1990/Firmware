#! /usr/bin/python

import serial, time
import subprocess
from subprocess import call, Popen
from argparse import ArgumentParser
import re

def do_tests(port, baudrate):     
    databits = serial.EIGHTBITS
    stopbits = serial.STOPBITS_ONE
    parity = serial.PARITY_NONE
    ser = serial.Serial(port, baudrate, databits, parity, stopbits, 100)
    ser.write('\n\n')
    
    finished = 0
    while finished == 0:
        serial_line = ser.readline()
        print(serial_line.replace('\n','')) 

        if "nsh>" in serial_line:
            finished = 1
        time.sleep(0.05)

    # perf
    ser.write('tests perf\n')

    finished = 0
    while finished == 0:
        serial_line = ser.readline()
        print(serial_line.replace('\n','')) 

        if "perf PASSED" in serial_line:
            finished = 1

        time.sleep(0.05)

    # free
    ser.write('ver all\n')
    ser.write('help\n')
    ser.write('free\n')
    ser.write('df\n')
    ser.write('ps\n')
    ser.write('ls /\n')
    ser.write('ls /etc \n')
    ser.write('ls /dev \n')
    ser.write('ls /fs \n')
    ser.write('ls /obj \n')
    ser.write('ls /proc \n')
    ser.write('uorb top -a \n')
    ser.write('top once \n')

    finished = 0
    while finished == 0:
        serial_line = ser.readline()
        print(serial_line.replace('\n',''))

        if "Uptime:" in serial_line:
            finished = 1
        time.sleep(0.05)

    ser.close() 

def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--device', "-d", nargs='?', default = None, help='')
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int, help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()

    do_tests(args.device, args.baudrate)

if __name__ == "__main__":
   main()
