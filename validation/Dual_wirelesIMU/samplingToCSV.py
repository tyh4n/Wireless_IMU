#receive data from wireless IMU station and plot data from both station and wireless unit
#modified from air bladder sampling
import csv
import serial
import numpy as np
import time
header = ['diff','wireless','station']
airBladderSerial = serial.Serial('COM15', 115200)
isRecording = False
isReceivingSync = True
time.sleep(1)
with open('imuData.csv', 'w') as f:
    writer = csv.writer(f)
    writer.writerow(header)
    while True:
        my_string = airBladderSerial.readline()#receive data from arduino
        my_string = my_string.decode() # convert to a string
        my_string = my_string.strip() # remove whitespace characters
        my_list = my_string.split("\t") # split by tab character
        my_list = [float(x) for x in my_list] # convert to floats
        print(my_list) # [0.33, 1.2, 0.0]
        writer.writerow(my_list)
