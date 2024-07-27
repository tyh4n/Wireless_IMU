import sys
import struct
import asyncio
import os
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QTextEdit, QVBoxLayout, QHBoxLayout, QWidget, QCheckBox, QLineEdit
)
from PyQt6.QtCore import QThread, pyqtSignal, QObject
from bleak import BleakClient
from concurrent.futures import ThreadPoolExecutor

# Bluetooth addresses of the devices
ADDRESS1 = "FE:03:48:54:0B:15"
ADDRESS2 = "D1:60:5B:C6:73:6A"

# UUIDs for the Bluetooth services and characteristics
IMU_EULAR_ALL_UUID = "efb96874-01f0-11ee-be56-0242ac120002"

recording = False
recorded_data = []

class Worker(QObject):
    data_signal = pyqtSignal(str)
    imu1_signal = pyqtSignal(str)
    imu2_signal = pyqtSignal(str)

    async def notification_handler_all_imu1(self, characteristic, data):
        x_1, y_1, z_1 = struct.unpack('fff', data)
        message = f"X: {x_1:.2f}, Y: {y_1:.2f}, Z: {z_1:.2f}"
        self.imu1_signal.emit(message)
        if recording:
            recorded_data.append((x_1, y_1, z_1, 'imu1'))

    async def notification_handler_all_imu2(self, characteristic, data):
        x_2, y_2, z_2 = struct.unpack('fff', data)
        message = f"X: {x_2:.2f}, Y: {y_2:.2f}, Z: {z_2:.2f}"
        self.imu2_signal.emit(message)
        if recording:
            recorded_data.append((x_2, y_2, z_2, 'imu2'))

    async def connect_imu(self, address, handler, imu_name):
        print(f"Attempting to connect to {imu_name} at address {address}...")
        try:
            async with BleakClient(address, timeout=20.0) as client:
                if client.is_connected:
                    connected_message = f"{imu_name} is connected: {client.is_connected}"
                    print(connected_message)
                    self.data_signal.emit(connected_message)
                    await client.start_notify(IMU_EULAR_ALL_UUID, handler)
                    await asyncio.sleep(100000)  # Adjust the sleep time for a longer connection
                else:
                    print(f"Failed to connect to {imu_name} at address {address}.")
        except Exception as e:
            print(f"Exception occurred while connecting to {imu_name} at address {address}: {e}")

    def run_task(self, func):
        asyncio.run(func)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU Recorder")

        # Create layout and central widget
        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        right_layout = QVBoxLayout()
        button_layout = QHBoxLayout()  # Changed to QHBoxLayout to align buttons horizontally
        file_name_layout = QVBoxLayout()
        check_layout = QHBoxLayout()
        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        # Create checkboxes
        self.imu1_checkbox = QCheckBox("IMU1")
        self.imu2_checkbox = QCheckBox("IMU2")

        # Create buttons
        self.connect_imus_button = QPushButton("Connect IMU")
        self.record_button = QPushButton("Record")
        self.end_button = QPushButton("End")
        self.save_button = QPushButton("Save to File")

        # Create terminal windows
        self.main_terminal = QTextEdit()
        self.main_terminal.setReadOnly(True)
        self.imu1_terminal = QTextEdit()
        self.imu1_terminal.setReadOnly(True)
        self.imu2_terminal = QTextEdit()
        self.imu2_terminal.setReadOnly(True)

        # Create input field for file name
        self.file_name_input = QLineEdit()
        self.file_name_input.setPlaceholderText("Enter file name")

        # Add checkboxes to check layout
        check_layout.addWidget(self.imu1_checkbox)
        check_layout.addWidget(self.imu2_checkbox)

        # Add buttons to button layout
        button_layout.addWidget(self.record_button)
        button_layout.addWidget(self.end_button)
        button_layout.addWidget(self.save_button)

        # Add widgets to left layout
        left_layout.addLayout(check_layout)
        left_layout.addWidget(self.connect_imus_button)
        left_layout.addWidget(self.main_terminal)

        # Add file name input and buttons to file name layout
        file_name_layout.addWidget(self.file_name_input)
        file_name_layout.addLayout(button_layout)

        left_layout.addLayout(file_name_layout)

        # Add IMU terminals to right layout
        right_layout.addWidget(self.imu1_terminal)
        right_layout.addWidget(self.imu2_terminal)

        # Add left and right layouts to main layout
        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)

        # Connect buttons to functions
        self.connect_imus_button.clicked.connect(self.connect_imus)
        self.record_button.clicked.connect(self.start_recording)
        self.end_button.clicked.connect(self.end_recording)
        self.save_button.clicked.connect(self.save_to_file)

        # Initialize worker for async tasks
        self.worker = Worker()
        self.worker.data_signal.connect(self.update_terminal)
        self.worker.imu1_signal.connect(self.update_imu1_terminal)
        self.worker.imu2_signal.connect(self.update_imu2_terminal)

        # Initialize a ThreadPoolExecutor
        self.executor = ThreadPoolExecutor()

    def connect_imus(self):
        self.main_terminal.append("Attempting to connect...")
        if self.imu1_checkbox.isChecked():
            print("Submitting task to connect to IMU1")
            self.executor.submit(self.connect_imu_task, ADDRESS1, self.worker.notification_handler_all_imu1, "IMU1")
        if self.imu2_checkbox.isChecked():
            print("Submitting task to connect to IMU2")
            self.executor.submit(self.connect_imu_task, ADDRESS2, self.worker.notification_handler_all_imu2, "IMU2")

    def connect_imu_task(self, address, handler, imu_name):
        print(f"Running task for {imu_name}")
        self.worker.run_task(self.worker.connect_imu(address, handler, imu_name))
        print(f"Finished task for {imu_name}")

    def start_recording(self):
        global recording, recorded_data
        recording = True
        recorded_data = []
        self.main_terminal.append("Recording started...")
        print("Recording started...")

    def end_recording(self):
        global recording
        recording = False
        self.main_terminal.append("Recording ended.")
        print("Recording ended.")

    def update_terminal(self, data):
        self.main_terminal.append(data)
        print(f"Terminal updated with: {data}")

    def update_imu1_terminal(self, data):
        self.imu1_terminal.append(data)
        print(f"IMU1 terminal updated with: {data}")

    def update_imu2_terminal(self, data):
        self.imu2_terminal.append(data)
        print(f"IMU2 terminal updated with: {data}")

    def save_to_file(self):
        file_name = self.file_name_input.text().strip()
        if not file_name:
            self.main_terminal.append("Please enter a file name.")
            return

        directory = "./data"
        if not os.path.exists(directory):
            os.makedirs(directory)

        file_path = os.path.join(directory, f"{file_name}.txt")
        with open(file_path, 'w') as file:
            for record in recorded_data:
                file.write(f"{record}\n")
        self.main_terminal.append(f"Data saved to {file_path}")
        print(f"Data saved to {file_path}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()

    sys.exit(app.exec())
