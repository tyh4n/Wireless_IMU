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
import datetime
import queue

from bleak import BleakClient
from bleak.uuids import uuid16_dict

from scipy import linalg

plottingCube = False

ADDRESS1 = "FE:03:48:54:0B:15"
ADDRESS2 = "D1:60:5B:C6:73:6A"

IMU_SERVICE_UUID =     "efb96352-01f0-11ee-be56-0242ac120002"
IMU_EULAR_X_UUID =     "efb966b8-01f0-11ee-be56-0242ac120002"
IMU_EULAR_Y_UUID =     "efb96848-01f0-11ee-be56-0242ac120002"
IMU_EULAR_Z_UUID =     "efb96974-01f0-11ee-be56-0242ac120002"
IMU_EULAR_ALL_UUID =   "efb96874-01f0-11ee-be56-0242ac120002"
IMU_CALIB_STAT_UUID =  "efb96a82-01f0-11ee-be56-0242ac120002"

IMU_DEBUG_UUID =       "efb96353-01f0-11ee-be56-0242ac120002"
IMU_MAG_ALL_UUID =     "efb966a8-01f0-11ee-be56-0242ac120002"

x_1 = 0
y_1 = 0
z_1 = 0
x_2 = 0
y_2 = 0
z_2 = 0

queuesize = 4

xqueue_1 = queue.Queue(queuesize)
yqueue_1 = queue.Queue(queuesize)
zqueue_1 = queue.Queue(queuesize)

xqueue_2 = queue.Queue(queuesize)
yqueue_2 = queue.Queue(queuesize)
zqueue_2 = queue.Queue(queuesize)

angleDifferenceData = []
angleData_1 = []
angleData_2 = []

async def notification_handler_all_imu1(characteristic, data):
    global x_1,y_1,z_1
    x_1 = struct.unpack('fff', data)[0]
    y_1 = struct.unpack('fff', data)[1]
    z_1 = struct.unpack('fff', data)[2]
    if xqueue_1.full():
        xqueue_1.get()
        yqueue_1.get()
        zqueue_1.get()
        xqueue_1.put(x_1)
        yqueue_1.put(y_1)
        zqueue_1.put(z_1)
    else:
        xqueue_1.put(x_1)
        yqueue_1.put(y_1)
        zqueue_1.put(z_1)
    # print("sensor1")
    # print(x_1,y_1,z_1)

    
async def update_angle_plot_imu1():
    while (True):
        if plottingCube:
            if xqueue_1.empty():
                ax.view_init(elev=-z_1, azim=x_1, roll = y_1)
            else:
                ax.view_init(elev=-zqueue_1.get(), azim=xqueue_1.get(), roll = yqueue_1.get())
            plt.pause(0.00001)
        await asyncio.sleep(0.01)
        #print(xqueue.qsize())


async def notification_handler_all_imu2(characteristic, data):
    global x_2,y_2,z_2
    x_2 = struct.unpack('fff', data)[0]
    y_2 = struct.unpack('fff', data)[1]
    z_2 = struct.unpack('fff', data)[2]
    if xqueue_2.full():
        xqueue_2.get()
        yqueue_2.get()
        zqueue_2.get()
        xqueue_2.put(x_2)
        yqueue_2.put(y_2)
        zqueue_2.put(z_2)
    else:
        xqueue_2.put(x_2)
        yqueue_2.put(y_2)
        zqueue_2.put(z_2)
    # print("sensor2")
    # print(x_2,y_2,z_2)

    
async def update_angle_plot_imu2():
    while (True):
        if plottingCube:
            if xqueue_2.empty():
                ax2.view_init(elev=-z_2, azim=x_2, roll = y_2)
            else:
                ax2.view_init(elev=-zqueue_2.get(), azim=xqueue_2.get(), roll = yqueue_2.get())
            plt.pause(0.00001)
        await asyncio.sleep(0.01)
        print(x_1, x_2, x_2 - x_1)
        angleDifferenceData.append(x_2 - x_1)
        angleData_1.append(x_1)
        angleData_2.append(x_2)


async def connect_imu_1(address):
    async with BleakClient(address, timeout=20.0) as client:
        print(f"Connected 1: {client.is_connected}")
        if plottingCube:
            axes = [5, 5, 5]
            data = np.ones(axes)
            alpha = 0.9
            colors = np.empty(axes + [4], dtype=np.float32)
            colors[:] = [1, 0, 0, alpha]  # red
            global ax,fig
            fig = plt.figure(figsize=(12,6))
            ax = fig.add_subplot(121, projection='3d')
            ax.voxels(data, facecolors=colors)

        await client.start_notify(IMU_EULAR_ALL_UUID, notification_handler_all_imu1)
        asyncio.create_task(update_angle_plot_imu1())

        await asyncio.sleep(10000)#need sleeping to keep the task and variables alive


async def connect_imu_2(address):
    async with BleakClient(address, timeout=20.0) as client:
        print(f"Connected 2: {client.is_connected}")
        if plottingCube:
            axes = [5, 5, 5]
            data = np.ones(axes)
            alpha = 0.9
            colors = np.empty(axes + [4], dtype=np.float32)
            colors[:] = [1, 0, 0, alpha]  # red
            global ax2,fig
            ax2 = fig.add_subplot(122, projection='3d')
            ax2.voxels(data, facecolors=colors)

        await client.start_notify(IMU_EULAR_ALL_UUID, notification_handler_all_imu2)
        asyncio.create_task(update_angle_plot_imu2())

        await asyncio.sleep(10000)#need sleeping to keep the task and variables alive


async def main():
    asyncio.create_task(connect_imu_1(ADDRESS1))
    await asyncio.sleep(5)
    asyncio.create_task(connect_imu_2(ADDRESS2))
    await asyncio.sleep(60)
    plt.plot(angleDifferenceData)
    plt.figure()
    plt.scatter(angleData_1, angleDifferenceData)
    
    plt.show()
            
if __name__ == "__main__":
    asyncio.run(main())




