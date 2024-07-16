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

#wireless imu 1
ADDRESS = "FE:03:48:54:0B:15"
#wireless imu 2
# ADDRESS = "D1:60:5B:C6:73:6A"


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
magxData = []
magyData = []
magzData = []

def fitEllipsoid(magX, magY, magZ):
    a1 = magX ** 2
    a2 = magY ** 2
    a3 = magZ ** 2
    a4 = 2 * np.multiply(magY, magZ)
    a5 = 2 * np.multiply(magX, magZ)
    a6 = 2 * np.multiply(magX, magY)
    a7 = 2 * magX
    a8 = 2 * magY
    a9 = 2 * magZ
    a10 = np.ones(len(magX)).T
    D = np.array([a1, a2, a3, a4, a5, a6, a7, a8, a9, a10])

    # Eqn 7, k = 4
    C1 = np.array([[-1, 1, 1, 0, 0, 0],
                   [1, -1, 1, 0, 0, 0],
                   [1, 1, -1, 0, 0, 0],
                   [0, 0, 0, -4, 0, 0],
                   [0, 0, 0, 0, -4, 0],
                   [0, 0, 0, 0, 0, -4]])

    # Eqn 11
    S = np.matmul(D, D.T)
    S11 = S[:6, :6]
    S12 = S[:6, 6:]
    S21 = S[6:, :6]
    S22 = S[6:, 6:]

    # Eqn 15, find eigenvalue and vector
    # Since S is symmetric, S12.T = S21
    tmp = np.matmul(np.linalg.inv(C1), S11 - np.matmul(S12, np.matmul(np.linalg.inv(S22), S21)))
    eigenValue, eigenVector = np.linalg.eig(tmp)
    u1 = eigenVector[:, np.argmax(eigenValue)]

    # Eqn 13 solution
    u2 = np.matmul(-np.matmul(np.linalg.inv(S22), S21), u1)

    # Total solution
    u = np.concatenate([u1, u2]).T

    Q = np.array([[u[0], u[5], u[4]],
                  [u[5], u[1], u[3]],
                  [u[4], u[3], u[2]]])

    n = np.array([[u[6]],
                  [u[7]],
                  [u[8]]])

    d = u[9]

    return Q, n, d
def calculateSIC():
    global magxData,magyData,magzData
    magX = np.array(magxData)
    magY = np.array(magyData)
    magZ = np.array(magzData)

    fig1 = plt.figure()
    ax1 = fig1.add_subplot(121, projection='3d')

    ax1.scatter(magX, magY, magZ, s=5, color='r')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')

    # plot unit sphere
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))
    ax1.plot_wireframe(x, y, z, rstride=10, cstride=10, alpha=0.5)
    ax1.plot_surface(x, y, z, alpha=0.3, color='b')

    Q, n, d = fitEllipsoid(magX, magY, magZ)

    Qinv = np.linalg.inv(Q)
    b = -np.dot(Qinv, n)
    Ainv = np.real(1 / np.sqrt(np.dot(n.T, np.dot(Qinv, n)) - d) * linalg.sqrtm(Q))

    print("A_inv: ")
    print(Ainv)
    print()
    print("b")
    print(b)
    print()

    calibratedX = np.zeros(magX.shape)
    calibratedY = np.zeros(magY.shape)
    calibratedZ = np.zeros(magZ.shape)

    totalError = 0
    for i in range(len(magX)):
        h = np.array([[magX[i], magY[i], magZ[i]]]).T
        hHat = np.matmul(Ainv, h-b)
        calibratedX[i] = hHat[0]
        calibratedY[i] = hHat[1]
        calibratedZ[i] = hHat[2]
        mag = np.dot(hHat.T, hHat)
        err = (mag[0][0] - 1)**2
        totalError += err
    print("Total Error: %f" % totalError)

    ax2 = fig1.add_subplot(122, projection='3d')

    ax2.scatter(calibratedX, calibratedY, calibratedZ, s=1, color='r')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')

    # plot unit sphere
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))
    ax2.plot_wireframe(x, y, z, rstride=10, cstride=10, alpha=0.5)
    ax2.plot_surface(x, y, z, alpha=0.3, color='b')
    plt.show()

async def notification_handler_x(characteristic, data):
    global x 
    x = struct.unpack('f', data)[0]
    
async def notification_handler_y(characteristic, data):
    global y 
    y = struct.unpack('f', data)[0]
    
async def notification_handler_z(characteristic, data):
    global z 
    z = struct.unpack('f', data)[0]
    print((x,y,z))
async def notification_handler_all(characteristic, data):
    global x,y,z,time_prev
    x = struct.unpack('fff', data)[0]
    y = struct.unpack('fff', data)[1]
    z = struct.unpack('fff', data)[2]
    if xqueue.full():
        xqueue.get()
        yqueue.get()
        zqueue.get()
        xqueue.put(x)
        yqueue.put(y)
        zqueue.put(z)
    else:
        xqueue.put(x)
        yqueue.put(y)
        zqueue.put(z)

    print((x,y,z))
    deltaT = datetime.datetime.now() - time_prev
    #print(deltaT.microseconds/1000)
    time_prev = datetime.datetime.now()
    
async def update_angle_plot():
    while (True):
        if xqueue.empty():
            ax.view_init(elev=-z, azim=x, roll = y)
        else:
            ax.view_init(elev=-zqueue.get(), azim=xqueue.get(), roll = yqueue.get())
        plt.pause(0.00001)
        await asyncio.sleep(0.01)
        #print(xqueue.qsize())

async def mag_debug_handler_all(characteristic, data):
    global magx,magy,magz,magxData,magyData,magzData
    magx = struct.unpack('fff', data)[0]
    magy = struct.unpack('fff', data)[1]
    magz = struct.unpack('fff', data)[2]
    magxData.append(magx)
    magyData.append(magy)
    magzData.append(magz)
    

async def update_mag_plot():
    global magx,magy,magz,ax2,magxData,magyData,magzData
    while (True):
        print(magx,magy,magz)
        ax2.cla()
        ax2.scatter(magxData, magyData, magzData)
        plt.pause(0.00001)
        await asyncio.sleep(0.5)
        #print(xqueue.qsize())

async def main(address):
    #while(True):
        async with BleakClient(address, timeout=20.0) as client:
            print(f"Connected: {client.is_connected}")

            axes = [5, 5, 5]
            data = np.ones(axes)
            alpha = 0.9
            colors = np.empty(axes + [4], dtype=np.float32)
            colors[:] = [1, 0, 0, alpha]  # red
            fig = plt.figure(figsize=(12,6))
            global ax
            ax = fig.add_subplot(121, projection='3d')
            ax.voxels(data, facecolors=colors)

            global ax2
            ax2 = fig.add_subplot(122, projection='3d')
            ax2.voxels
            ax2.set_xlabel('X')
            ax2.set_ylabel('Y')
            ax2.set_zlabel('Z')
            ax2.set_aspect('equal', 'box')

            #await client.start_notify(IMU_EULAR_X_UUID, notification_handler_x)
            #await client.start_notify(IMU_EULAR_Y_UUID, notification_handler_y)
            #await client.start_notify(IMU_EULAR_Z_UUID, notification_handler_z)
            # await client.start_notify(IMU_EULAR_ALL_UUID, notification_handler_all)
            # asyncio.create_task(update_angle_plot())

            await client.start_notify(IMU_MAG_ALL_UUID, mag_debug_handler_all)
            asyncio.create_task(update_mag_plot())

            await asyncio.sleep(600)
            await client.disconnect() 
            calculateSIC()
            await asyncio.sleep(10)
            plt.show()
            
if __name__ == "__main__":
    asyncio.run(main(ADDRESS))




