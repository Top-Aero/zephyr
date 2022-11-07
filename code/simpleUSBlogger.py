import os
import serial

BAUD = 9600
FICHIER = 'zephyrSerial.csv'
PORT = '/dev/ttyACM0'

logSerial = serial.Serial(PORT, BAUD, timeout=200)

with open(FICHIER, 'a') as f:
    while(True):    
        trame = logSerial.readline()
        f.write(trame)
        print(trame)