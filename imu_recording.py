import numpy as np
import asyncio
import matplotlib.pyplot as plt
import struct
import queue
import serial
from bleak import BleakClient

# Flag to control whether to plot the cube
plottingCube = False

# Bluetooth addresses of the devices
ADDRESS1 = "FE:03:48:54:0B:15"
ADDRESS2 = "D1:60:5B:C6:73:6A"

# UUIDs for the Bluetooth services and characteristics
IMU_SERVICE_UUID = "efb96352-01f0-11ee-be56-0242ac120002"
IMU_EULAR_X_UUID = "efb966b8-01f0-11ee-be56-0242ac120002"
IMU_EULAR_Y_UUID = "efb96848-01f0-11ee-be56-0242ac120002"
IMU_EULAR_Z_UUID = "efb96974-01f0-11ee-be56-0242ac120002"
IMU_EULAR_ALL_UUID = "efb96874-01f0-11ee-be56-0242ac120002"
IMU_CALIB_STAT_UUID = "efb96a82-01f0-11ee-be56-0242ac120002"
IMU_DEBUG_UUID = "efb96353-01f0-11ee-be56-0242ac120002"
IMU_MAG_ALL_UUID = "efb966a8-01f0-11ee-be56-0242ac120002"

# Initial sensor values
x_1 = 0
y_1 = 0
z_1 = 0
x_2 = 0
y_2 = 0
z_2 = 0

# Queue size for storing sensor data
queuesize = 4

# Queues for sensor 1 data
xqueue_1 = queue.Queue(queuesize)
yqueue_1 = queue.Queue(queuesize)
zqueue_1 = queue.Queue(queuesize)

# Queues for sensor 2 data
xqueue_2 = queue.Queue(queuesize)
yqueue_2 = queue.Queue(queuesize)
zqueue_2 = queue.Queue(queuesize)

# Lists to store angle differences and sensor data
angleDifferenceData = []
angleData_1 = []
angleData_2 = []

recording = False
recorded_data = []

# Serial port setup
serial_port = serial.Serial('COM3', baudrate=9600, timeout=1)

# Handler for notifications from IMU 1
async def notification_handler_all_imu1(characteristic, data):
    global x_1, y_1, z_1
    x_1, y_1, z_1 = struct.unpack('fff', data)
    if xqueue_1.full():
        xqueue_1.get()
        yqueue_1.get()
        zqueue_1.get()
    xqueue_1.put(x_1)
    yqueue_1.put(y_1)
    zqueue_1.put(z_1)
    if recording:
        recorded_data.append((x_1, y_1, z_1, 'imu1'))

# Update plot for IMU 1
async def update_angle_plot_imu1():
    while True:
        if plottingCube:
            if xqueue_1.empty():
                ax.view_init(elev=-z_1, azim=x_1, roll=y_1)
            else:
                x_vals = list(xqueue_1.queue)
                y_vals = list(yqueue_1.queue)
                z_vals = list(zqueue_1.queue)
                ax.view_init(elev=-z_vals[-1], azim=x_vals[-1], roll=y_vals[-1])
            plt.draw()
        await asyncio.sleep(0.1)

# Handler for notifications from IMU 2
async def notification_handler_all_imu2(characteristic, data):
    global x_2, y_2, z_2
    x_2, y_2, z_2 = struct.unpack('fff', data)
    if xqueue_2.full():
        xqueue_2.get()
        yqueue_2.get()
        zqueue_2.get()
    xqueue_2.put(x_2)
    yqueue_2.put(y_2)
    zqueue_2.put(z_2)
    if recording:
        recorded_data.append((x_2, y_2, z_2, 'imu2'))

# Update plot for IMU 2
async def update_angle_plot_imu2():
    while True:
        if plottingCube:
            if xqueue_2.empty():
                ax2.view_init(elev=-z_2, azim=x_2, roll=y_2)
            else:
                x_vals = list(xqueue_2.queue)
                y_vals = list(yqueue_2.queue)
                z_vals = list(zqueue_2.queue)
                ax2.view_init(elev=-z_vals[-1], azim=x_vals[-1], roll=y_vals[-1])
            plt.draw()
        await asyncio.sleep(0.1)

# Connect to IMU 1
async def connect_imu_1(address):
    # Create a client connection to the BLE device with the specified address
    async with BleakClient(address, timeout=20.0) as client:
        print(f"Connected 1: {client.is_connected}")  # Check if connected
        if plottingCube:
            # Plotting setup for a 3D cube
            fig = plt.figure()
            axes = [5, 5, 5]
            data = np.ones(axes)
            alpha = 0.9
            colors = np.empty(axes + [4], dtype=np.float32)
            colors[:] = [0, 1, 0, alpha]  # green
            global ax
            ax = fig.add_subplot(121, projection='3d')
            ax.voxels(data, facecolors=colors)

        # Start notifications for IMU 1
        await client.start_notify(IMU_EULAR_ALL_UUID, notification_handler_all_imu1)
        # Create a task to update the plot for IMU 1
        asyncio.create_task(update_angle_plot_imu1())
        # Keep the connection and tasks alive
        await asyncio.sleep(10000)

# Connect to IMU 2
async def connect_imu_2(address):
    # Create a client connection to the BLE device with the specified address
    async with BleakClient(address, timeout=20.0) as client:
        print(f"Connected 2: {client.is_connected}")  # Check if connected
        if plottingCube:
            # Plotting setup for a 3D cube
            axes = [5, 5, 5]
            data = np.ones(axes)
            alpha = 0.9
            colors = np.empty(axes + [4], dtype=np.float32)
            colors[:] = [1, 0, 0, alpha]  # red
            global ax2
            ax2 = fig.add_subplot(122, projection='3d')
            ax2.voxels(data, facecolors=colors)

        # Start notifications for IMU 2
        await client.start_notify(IMU_EULAR_ALL_UUID, notification_handler_all_imu2)
        # Create a task to update the plot for IMU 2
        asyncio.create_task(update_angle_plot_imu2())
        # Keep the connection and tasks alive
        await asyncio.sleep(10000)

# Handle serial commands
async def handle_serial_commands():
    global recording, recorded_data
    while True:
        if serial_port.in_waiting > 0:
            command = serial_port.readline().decode('utf-8').strip()
            if command == "Start Recording":
                print("Start Recording command received")
                recording = True
                recorded_data = []
            elif command == "End Recording":
                print("End Recording command received")
                recording = False
                # Save the recorded data to a file
                with open('recorded_data.txt', 'w') as f:
                    for record in recorded_data:
                        f.write(f"{record}\n")
        await asyncio.sleep(0.1)

# Main function to start the connections and handle serial commands
async def main():
    # Create a task to handle serial commands
    asyncio.create_task(handle_serial_commands())
    # Create a task to connect to IMU 1
    asyncio.create_task(connect_imu_1(ADDRESS1))
    await asyncio.sleep(5)  # Wait before connecting to the second IMU
    # Create a task to connect to IMU 2
    asyncio.create_task(connect_imu_2(ADDRESS2))
    await asyncio.sleep(10000)  # Keep the program running to maintain connections and recording

if __name__ == "__main__":
    asyncio.run(main())  # Run the main function using asyncio
