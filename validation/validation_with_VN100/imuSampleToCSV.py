# Import libraries
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import asyncio
import platform
import sys
import socket
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import RadioButtons, Button
import struct
import queue
import datetime
import csv

from bleak import BleakClient
from bleak.uuids import uuid16_dict

from scipy import linalg

#wireless imu 1
#ADDRESS = "FE:03:48:54:0B:15"
#wireless imu 2
ADDRESS = "D1:60:5B:C6:73:6A"


IMU_SERVICE_UUID =     "efb96352-01f0-11ee-be56-0242ac120002"
IMU_EULAR_X_UUID =     "efb966b8-01f0-11ee-be56-0242ac120002"
IMU_EULAR_Y_UUID =     "efb96848-01f0-11ee-be56-0242ac120002"
IMU_EULAR_Z_UUID =     "efb96974-01f0-11ee-be56-0242ac120002"
IMU_EULAR_ALL_UUID =   "efb96874-01f0-11ee-be56-0242ac120002"
IMU_CALIB_STAT_UUID =  "efb96a82-01f0-11ee-be56-0242ac120002"

IMU_DEBUG_UUID =       "efb96353-01f0-11ee-be56-0242ac120002"
IMU_MAG_ALL_UUID =     "efb966a8-01f0-11ee-be56-0242ac120002"

x = 0
y = 0
z = 0
magx = 0
magy = 0
magz = 0
time_prev = datetime.datetime.now()
queuesize = 4
xqueue = queue.Queue(queuesize)
yqueue = queue.Queue(queuesize)
zqueue = queue.Queue(queuesize)

IMUDataRecord = [['x','y','z']]

async def notification_handler_all(characteristic, data):
    global x,y,z,time_prev
    x = struct.unpack('fff', data)[0]
    y = struct.unpack('fff', data)[1]
    z = struct.unpack('fff', data)[2]
    IMUDataRecord.append([x,y,z])
    deltaT = datetime.datetime.now() - time_prev
    print(deltaT.microseconds/1000)
    time_prev = datetime.datetime.now()
    


async def main(address):
    #while(True):
        async with BleakClient(address, timeout=20.0) as client:
            print(f"Connected: {client.is_connected}")

            await client.start_notify(IMU_EULAR_ALL_UUID, notification_handler_all)
            await asyncio.sleep(120)
            with open('IMUData.csv', 'w') as f:
                writer = csv.writer(f)
                writer.writerows(IMUDataRecord)
            await client.disconnect() 

            
if __name__ == "__main__":
    asyncio.run(main(ADDRESS))




