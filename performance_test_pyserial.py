import random
import numpy as np
from cobs import cobs
from PySide6.QtCore import QTimer, Signal, QElapsedTimer
from PySide6.QtWidgets import QPushButton, QVBoxLayout, QHBoxLayout, QPlainTextEdit, QWidget
import pyqtgraph as pg
from queue import Queue
from time import time, sleep
from PySide6.QtCore import QIODevice, QObject, Signal
from PySide6.QtCore import QThread
from PySide6.QtWidgets import QApplication
from dataclasses import dataclass
from serial import Serial, STOPBITS_ONE, PARITY_NONE, EIGHTBITS
import sys


#region variablen
NUM_BUF = 2000
ADXL_NUM_BYTES = 252
BASE_NUM_BYTES = 25
BASE_NUM_BYTES_NEW = 223
TIMING_NUM_BYTES = 13
GPS_NUM_BYTES = 44

BUS_DATA_PAYLOAD_AD = 0
BUS_DATA_PAYLOAD_355 = 1
BUS_DATA_PAYLOAD_357 = 2
BUS_INFO_PAYLOAD = 3
BUS_CONFIG_PAYLOAD = 6
BUS_TIMING_PAYLOAD = 7
BASE_DATA_PAYLOAD = 8
GPS_DATA_PAYLOAD = 9
BASE_DATA_PAYLOAD_NEW = 10
BUS_DATA_PAYLOAD_AD_NEW = 11
BUS_DATA_PAYLOAD_IMU = 12
BUS_DATA_PAYLOAD_DIN = 13

reset_USB_MCU = [
    np.uint16(0),
    np.uint16(0),
    np.uint16(0),
    np.uint8(250),
    np.uint8(250),
    np.uint8(250),
    np.uint8(250),
    np.uint8(250),
    np.uint8(250),
    np.uint8(6)
]
reset_RF_MCU = [
    np.uint16(0),
    np.uint16(0),
    np.uint16(0),
    np.uint8(251),
    np.uint8(251),
    np.uint8(251),
    np.uint8(251),
    np.uint8(251),
    np.uint8(251),
    np.uint8(6)
]
setRFChannel = [
    np.uint16(0), # Funk/Bitrate: 0 = 2 MBit/s, 1 = 1 MBit/s
    np.uint16(0), # Max. Sensoranzahl pro Funkkanal: 0 = 3 Sensoren, 1 = 8 Sensoren
    np.uint16(0), # reserviert
    np.uint8(5),  # Funk-Kanal 0 / 80 / 2400 / 2480 MHz
    np.uint8(251),# adxl_range
    np.uint8(251),# adxl_odr
    np.uint8(251),# imu_acc_range
    np.uint8(251),# gyr_range
    np.uint8(251),# mag_range
    np.uint8(6),  # id_byte
]
settings_node_tab_1 = [
    np.uint16(30),  # sleep_time default 30
    np.uint16(30),  # sample_time default 30
    np.uint16(0),  # min_rpm * 10 default 1,5
    np.uint8(1),    # mode default 1 = continous
    np.uint8(224),  # adxl_range default HR / 100 Hz
    np.uint8(82),  # adxl_odr default 104 Hz
    np.uint8(2),  # imu_acc_range default 4 g
    np.uint8(2),  # gyr_range default 500 °/s
    np.uint8(3),  # mag_range default 8 gauss
    np.uint8(22),  # id_byte
]
settings_node_tab_2 = [
    np.uint16(30),  # sleep_time default 30
    np.uint16(30),  # sample_time default 30
    np.uint16(0),  # min_rpm * 10 default 1,5
    np.uint8(1),    # mode default 1 = continous
    np.uint8(224),  # adxl_range default HR / 100 Hz
    np.uint8(82),  # adxl_odr default 104 Hz
    np.uint8(2),  # imu_acc_range default 4 g
    np.uint8(2),  # gyr_range default 500 °/s
    np.uint8(3),  # mag_range default 8 gauss
    np.uint8(38),  # id_byte
]
settings_node_tab_3 = [
    np.uint16(30),  # sleep_time default 30
    np.uint16(30),  # sample_time default 30
    np.uint16(0),  # min_rpm * 10 default 1,5
    np.uint8(1),    # mode default 1 = continous
    np.uint8(224),  # adxl_range default HR / 100 Hz
    np.uint8(82),  # adxl_odr default 104 Hz
    np.uint8(2),  # imu_acc_range default 4 g
    np.uint8(2),  # gyr_range default 500 °/s
    np.uint8(3),  # mag_range default 8 gauss
    np.uint8(54),  # id_byte
]
standard_base_settings = [
    np.uint16(0),
    np.uint16(0),
    np.uint16(0),
    np.uint8(0),
    np.uint8(0),  # adxl_range
    np.uint8(4),  # adxl_odr
    np.uint8(2),  # imu_acc_range
    np.uint8(2),  # gyr_range
    np.uint8(2),  # mag_range
    np.uint8(6),  # id_byte
]

#imu_acc_sensitivity = 16393.44
imu_acc_sensitivity = 8196.72
imu_gyr_sensitivity = 114.2857
imu_mag_sensitivity = 6842.0
#endregion


@dataclass
class adxl_uart_data_message:
    x_acc: np.uint16 = np.uint16(0)
    y_acc: np.int16 = np.uint16(0)
    z_acc: np.int16 = np.uint16(0)

    def __init__(self):
        pass


class Decoder(QObject):
    def __init__(self):
        super(Decoder, self).__init__()

    def encode_message(self, data):
        list_convert = self.convert_into_8bit_list(data)
        bytes_raw = bytearray(list_convert)
        bytes_encoded = cobs.encode(bytes_raw)
        bytes_encoded += bytes([0])
        return bytes_encoded

    @staticmethod
    def convert_into_8bit_list(data):
        data_convert = [0] * (len(data) + 3)
        j = 0
        ########################################
        for i in range(len(data)):
            if type(data[i]) == np.uint16:
                if data[i] <= 255:
                    data_convert[j] = 0
                    data_convert[j + 1] = np.uint8(data[i])
                    j += 1
            else:
                data_convert[j] = np.uint8(data[i])
            j += 1
        ########################################
        return data_convert

    def decode_message(self, length, enc_message):
        print("decode message")
        dec_message = cobs.decode(b''.join(enc_message))

        message_id = dec_message[length - 2 - 1] & 15
        sensor_id = (dec_message[length - 2 - 1] & 240) >> 4

        if length - 2 == TIMING_NUM_BYTES:
            pass
        if length - 2 == ADXL_NUM_BYTES and 0 <= sensor_id <= 8:
            if message_id == BUS_DATA_PAYLOAD_AD_NEW:
                pass
            elif message_id == BUS_DATA_PAYLOAD_IMU:
                pass
            elif message_id == BUS_DATA_PAYLOAD_DIN:
                pass
            elif message_id == BUS_DATA_PAYLOAD_AD:
                pass
            elif message_id == BUS_DATA_PAYLOAD_355 or message_id == BUS_DATA_PAYLOAD_357:
                return self.decode_payload_data(dec_message, ADXL_NUM_BYTES)
            elif message_id == BUS_INFO_PAYLOAD:
                pass
            elif message_id == BUS_TIMING_PAYLOAD:
                pass

    #region Decode Payload
    def decode_payload_data(self, input_message, data_len):
        if data_len >= ADXL_NUM_BYTES:
            new_data = adxl_uart_data_message()

            #region set data
            new_data.timestamp = self.decode_uint32(input_message, ADXL_NUM_BYTES, 0)

            x_acc = self.decode_int16(input_message, ADXL_NUM_BYTES, 4)
            new_data.x_acc = float(x_acc) / imu_acc_sensitivity * (-1.0)
            y_acc = self.decode_int16(input_message, ADXL_NUM_BYTES, 6)
            new_data.y_acc = float(y_acc) / imu_acc_sensitivity * (-1.0)
            z_acc = self.decode_int16(input_message, ADXL_NUM_BYTES, 8)
            new_data.z_acc = float(z_acc) / imu_acc_sensitivity * (-1.0)
            # endregion
            return new_data
    #endregion

    #region Decode unsigned Integer
    @staticmethod
    def decode_int16(input, data_len, startBit):
        temp = 0

        if startBit <= data_len and (startBit + 1) <= data_len:
            temp = int.from_bytes(input[startBit:startBit + 2], byteorder='little')
            temp = np.int16(temp)

        return temp

    @staticmethod
    def decode_uint32(input, data_len, startBit):
        temp = 0

        if startBit <= data_len and (startBit + 3) <= data_len:
            temp = int.from_bytes(input[startBit: startBit + 4], byteorder='little')
            temp = np.uint32(temp)

        return temp
    #endregion


class Connection(QObject):
    #region Signals
    connection_established = Signal()
    debug_message = Signal(str)
    #endregion

    def __init__(self, window_graph_ref):
        super(Connection, self).__init__()
        self.connected = False
        self.finish_reading = False
        self.buffer = Queue()
        self.decoder = Decoder()
        self.graph = window_graph_ref

    #region Setup Port
    def create_port(self):
        self.ser = Serial('COM3')
        self.debug_message.emit("Port created")
        self.setup_port_settings()

    def setup_port_settings(self):
        self.ser.port = 'COM3'
        self.ser.baudrate = 115200
        self.ser.bytesize = EIGHTBITS
        self.ser.parity = PARITY_NONE
        self.ser.stopbits = STOPBITS_ONE

        self.open_port()

    def open_port(self):
        if self.ser.is_open:
            self.debug_message.emit("Port open")
            if not self.connected:
                self.open_connection()
    #endregion

    def open_connection(self):
        if not self.ser.is_open:
            self.open_port()

        self.debug_message.emit("Open Connection")

        #region send config message
        self.send_message(setRFChannel)
        sleep(0.2)
        self.send_message(settings_node_tab_1)
        sleep(0.2)
        self.send_message(settings_node_tab_2)
        sleep(0.2)
        self.send_message(settings_node_tab_3)
        sleep(0.2)
        self.send_message(standard_base_settings)
        #endregion

        self.connected = True
        self.connection_established.emit()
        self.debug_message.emit("Connection Established")
        self.reading_data_into_buffer()

    def close_connection(self):
        self.debug_message.emit("Closing Connection")
        self.send_message(reset_USB_MCU)

        if self.ser.is_open:
            self.ser.close()

        sleep(0.1)
        self.debug_message.emit("Connection closed")
        self.connected = False

    def send_message(self, data):
        if self.ser.isOpen():
            data_encode = self.decoder.encode_message(data)
            self.ser.write(data_encode)

    def create_data(self):
        tmp = adxl_uart_data_message()
        tmp.x_acc = random.randint(-4, 4)
        tmp.z_acc = random.randint(-4, 4)
        tmp.y_acc = random.randint(-4, 4)
        self.graph.add_element_to_adxl_data(tmp)

    def reading_data_into_buffer(self):
        counter = 0
        message = []
        while not self.finish_reading:
            all_bytes = self.ser.read(100)
            for i in range(100):
                byte_got = all_bytes[i]
                if byte_got == 0:
                    self.buffer.put(message, block=True, timeout=0.1)
                    self.read_message()
                    message = []
                    counter = 0
                else:
                    counter += 1
                    message.append(byte_got)
        self.close_connection()

    def read_message(self):
        if not self.buffer.empty():
            message = self.buffer.get(block=True, timeout=0.01)
            if len(message) == ADXL_NUM_BYTES + 1:
                self.create_data()


class Graph(QWidget):
    def __init__(self):
        super(Graph, self).__init__()
        self.current_time = 0
        self.is_plotting = False
        self.adxl_data_queue = Queue()
        self.adxl_data_list = [adxl_uart_data_message]
        self.test_bool = True

        #region setup graph widget
        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setBackground('w')
        self.graph_widget.addLegend()
        self.graph_widget.showGrid(x=True, y=True)
        self.graph_widget.setMouseEnabled(x=False, y=False)
        self.graph_widget.setYRange(-4, 4)
        self.graph_widget.setLabel('left', "<span style=\"color:black;font-size:20px\">Voltage [mV/V]</span>")
        self.graph_widget.setLabel('bottom', "<span style=\"color:black;font-size:20px\">Time [s]</span>")
        #endregion

        self.list_of_y_values = [
            [0] * 8000,
            [0] * 8000,
            [0] * 8000
        ]

        self.plot_liste = [
            self.create_plot((0, 255, 0), 'X-ACC'),
            self.create_plot((0, 0, 255), 'Y-ACC'),
            self.create_plot((255, 0, 0), 'Z-ACC')
        ]

        #region setup layout
        layout = QHBoxLayout()
        layout.addWidget(self.graph_widget)
        self.setLayout(layout)
        #endregion

    def create_plot(self, color, name: str):
        pen = pg.mkPen(color=color, width=0.6)

        return self.graph_widget.plot(list(np.linspace(0, (0 + 5), num=500)), ([0] * 500), name=name, pen=pen)

    @staticmethod
    def update_plot(plot, x_values, y_values):
        plot.setData(x_values, y_values)

    def start_plotting(self):
        #region setup timer
        self.update_plot_timer = QTimer()
        self.update_plot_timer.setInterval(10)
        self.update_plot_timer.timeout.connect(self.add_value_to_y_values)
        #endregion
        self.update_plot_timer.start()

    def stop_plotting(self):
        self.update_plot_timer.stop()

    def plot(self):
        tmp_current_time = self.current_time
        for i in range(3):
            self.update_plot(self.plot_liste[i], list(np.linspace(tmp_current_time, (tmp_current_time + 5), num=8000)),
                             self.list_of_y_values[i])
        self.current_time += 0.01

    def add_element_to_adxl_data(self, message: adxl_uart_data_message):
        self.adxl_data_list.append(message)

    def add_value_to_y_values(self):
        if self.test_bool:
            self.test_bool = False
            adxl_data_list_tmp = self.adxl_data_list.copy()
            adxl_data_list_tmp_len = len(adxl_data_list_tmp)

            if adxl_data_list_tmp_len > 1:
                if adxl_data_list_tmp_len > 8000:
                    adxl_data_list_tmp_len = 8000
                del self.adxl_data_list[0:len(adxl_data_list_tmp)]
                for i in range(3):
                    del self.list_of_y_values[i][0:adxl_data_list_tmp_len]

                [self.list_of_y_values[0].append(element.x_acc) for element in
                 adxl_data_list_tmp[0: adxl_data_list_tmp_len]]
                [self.list_of_y_values[1].append(element.y_acc) for element in
                 adxl_data_list_tmp[0: adxl_data_list_tmp_len]]
                [self.list_of_y_values[2].append(element.z_acc) for element in
                 adxl_data_list_tmp[0: adxl_data_list_tmp_len]]

            self.plot()
            self.test_bool = True


class MainWindow(QWidget):
    def __init__(self, start_connection_fn, stop_connection_fn):
        super(MainWindow, self).__init__()

        self.start_connection = start_connection_fn
        self.stop_connection = stop_connection_fn

        self.textfield_debugger = QPlainTextEdit()
        self.textfield_debugger.setReadOnly(True)

        #region setup Buttons
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.start_connection)

        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.clicked.connect(self.stop_connection)
        #endregion

        #region setup graph
        self.graph = Graph()
        self.graph_thread = QThread()
        self.graph.moveToThread(self.graph_thread)
        self.graph_thread.started.connect(self.graph.start_plotting)
        #endregion

        #region setup layout
        button_layout = QHBoxLayout()

        button_layout.addWidget(self.connect_button)
        button_layout.addWidget(self.disconnect_button)

        widget_layout = QVBoxLayout()
        widget_layout.addLayout(button_layout)
        widget_layout.addWidget(self.graph)
        widget_layout.addWidget(self.textfield_debugger)

        self.setLayout(widget_layout)
        #endregion

    def start_plotting(self):
        self.graph_thread.start()

    def stop_plotting(self):
        self.graph.stop_plotting()
        self.graph_thread.terminate()
        self.graph_thread.wait()


class Controller:
    def __init__(self):
        self.window = MainWindow(self.start_connection, self.stop_connection)
        self.connection = Connection(self.window.graph)

        self.connection_thread = QThread()
        self.connection.moveToThread(self.connection_thread)
        self.connection_thread.started.connect(self.connection.create_port)

        self.connection.connection_established.connect(self.start_reading_data)
        self.connection.debug_message.connect(self.send_debug_message)

    def send_debug_message(self, message: str):
        self.window.textfield_debugger.appendPlainText(message)

    def show_window(self):
        self.window.show()

    def start_reading_data(self):
        self.window.graph.start_plotting()

    def start_connection(self):
        self.connection.finish_reading = False
        self.connection_thread.start()

    def stop_connection(self):
        self.connection.finish_reading = True
        self.window.stop_plotting()
        while self.connection.connected:
            pass
        self.terminate_threads()

    def terminate_threads(self):
        self.connection_thread.terminate()
        self.connection_thread.wait()


def main():
    app = QApplication(sys.argv)

    controller = Controller()
    controller.show_window()

    app.exec()


if __name__ == '__main__':
    main()

