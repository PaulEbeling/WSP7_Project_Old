import numpy as np
from cobs import cobs
from PySide6.QtSerialPort import QSerialPort
from PySide6.QtCore import QIODevice, QObject, Signal
from queue import Queue
from time import sleep
from dataclasses import dataclass

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

imu_acc_sensitivity = 16393.44
imu_gyr_sensitivity = 114.2857
imu_mag_sensitivity = 6842.0
#endregion

#region dataclass

@dataclass
class adxl_uart_info_message:
    timestamp: np.uint32
    info = [np.uint8] * 246
    rssi: np.uint8
    id_byte: np.uint8

@dataclass
class adxl_uart_data_message:
    x_gyr: np.int16
    y_gyr: np.int16
    z_gyr: np.int16

    x_mag: np.int16
    y_mag: np.int16
    z_mag: np.int16

    temperature: np.int16

    adxl_data = [np.uint8] * 225

    fuel: np.uint8
    rssi: np.uint8
    id_byte: np.uint8

    x_acc: np.uint16 =  np.uint16(0)
    y_acc: np.int16 = np.uint16(0)
    z_acc: np.int16 = np.uint16(0)

    def __init__(self):
        pass
#endregion


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
        dec_message = cobs.decode(b''.join(enc_message))
        #byte_list_to_int_array = [int.from_bytes(element, "big") for element in enc_message]
        #dec_message = cobs.decode(bytearray(byte_list_to_int_array))

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
                self.decode_payload_info(dec_message, ADXL_NUM_BYTES)
            elif message_id == BUS_TIMING_PAYLOAD:
                pass

    #region Decode Payload
    def decode_payload_data(self, input_message, data_len):
        if data_len >= ADXL_NUM_BYTES:
            new_data = adxl_uart_data_message()

            #region set data
            new_data.timestamp = self.decode_uint32(input_message, ADXL_NUM_BYTES, 0)

            x_acc = self.decode_int16(input_message, ADXL_NUM_BYTES, 4)
            new_data.x_acc = float(x_acc)/imu_acc_sensitivity * (-1.0)
            y_acc = self.decode_int16(input_message, ADXL_NUM_BYTES, 6)
            new_data.y_acc = float(y_acc) / imu_acc_sensitivity * (-1.0)
            z_acc = self.decode_int16(input_message, ADXL_NUM_BYTES, 8)
            new_data.z_acc = float(z_acc) / imu_acc_sensitivity * (-1.0)

            new_data.x_gyr = self.decode_int16(input_message, ADXL_NUM_BYTES, 10)
            new_data.y_gyr = self.decode_int16(input_message, ADXL_NUM_BYTES, 12)
            new_data.z_gyr = self.decode_int16(input_message, ADXL_NUM_BYTES, 14)

            new_data.x_mag = self.decode_int16(input_message, ADXL_NUM_BYTES, 16)
            new_data.y_mag = self.decode_int16(input_message, ADXL_NUM_BYTES, 18)
            new_data.z_mag = self.decode_int16(input_message, ADXL_NUM_BYTES, 20)

            new_data.temperature = self.decode_int16(input_message, ADXL_NUM_BYTES, 22)

            new_data.fuel = input_message[ADXL_NUM_BYTES - 3]
            new_data.rssi = input_message[ADXL_NUM_BYTES - 2]
            new_data.id_byte = input_message[ADXL_NUM_BYTES - 1]

            #############################################
            new_data.adxl_data.extend(input_message[24: 249])
            #############################################
            # endregion
            return new_data

    def decode_payload_info(self, input, data_len):
        if data_len >= ADXL_NUM_BYTES:
            new_info = adxl_uart_info_message
            new_info.timestamp = self.decode_uint32(input, data_len, 0)
            new_info.rssi = input[data_len - 2]
            new_info.id_byte = input[data_len - 1]
            ########################################
            new_info.info = input[4:250]
            ########################################
            return new_info
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
        self.ser = QSerialPort()
        self.debug_message.emit("Port created")
        self.setup_port_settings()

    def setup_port_settings(self):
        self.ser.setPortName("COM3")
        self.ser.setBaudRate(QSerialPort.BaudRate.Baud115200)
        self.ser.setDataBits(QSerialPort.DataBits.Data8)
        self.ser.setParity(QSerialPort.Parity.NoParity)
        self.ser.setStopBits(QSerialPort.StopBits.OneStop)
        self.ser.setFlowControl(QSerialPort.FlowControl.NoFlowControl)

        self.open_port()

    def open_port(self):
        if self.ser.open(QIODevice.OpenModeFlag.ReadWrite):
            self.ser.setDataTerminalReady(True)
            self.debug_message.emit("Port open")
            if not self.connected:
                self.open_connection()
    #endregion

    def open_connection(self):
        if not self.ser.isOpen():
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

        if self.ser.waitForBytesWritten(100):
            pass

        self.connected = True
        self.connection_established.emit()
        self.debug_message.emit("Connection Established")
        self.reading_data_into_buffer()

    def close_connection(self):
        self.debug_message.emit("Closing Connection")
        self.send_message(reset_USB_MCU)

        if self.ser.waitForBytesWritten(100):
            pass

        if self.ser.isOpen():
            self.ser.close()

        sleep(0.1)
        self.debug_message.emit("Connection closed")
        self.connected = False

    def send_message(self, data):
        if self.ser.isOpen():
            data_encode = self.decoder.encode_message(data)
            self.ser.write(data_encode)

    def reading_data_into_buffer(self):
        while not self.finish_reading:
            message = []
            if self.ser.waitForReadyRead(100):
                all_bytes = self.ser.readAll()
                for byte in all_bytes:
                    if byte == bytes(1):
                        self.buffer.put(message, block=True, timeout=0.1)
                        message = []
                    else:
                        message.append(byte)
                self.read_message()
        self.close_connection()

    def read_message(self):
        buffer_len = self.buffer.qsize()
        for i in range(buffer_len):
            message = self.buffer.get(block=True, timeout=0.01)
            if len(message) == ADXL_NUM_BYTES + 1:
                tmp = self.decoder.decode_message((ADXL_NUM_BYTES + 2), message)
                if not tmp is None:
                    self.graph.add_element_to_adxl_data(tmp)
            elif len(message) == BASE_NUM_BYTES + 1:
                self.decoder.decode_message((BASE_NUM_BYTES + 2), message)
            elif len(message) == BASE_NUM_BYTES_NEW + 1:
                self.decoder.decode_message((BASE_NUM_BYTES_NEW + 2), message)
            elif len(message) == GPS_NUM_BYTES + 1:
                self.decoder.decode_message((GPS_NUM_BYTES + 2), message)
            elif len(message) == TIMING_NUM_BYTES + 1:
                self.decoder.decode_message((TIMING_NUM_BYTES + 2), message)
            else:
                print("USB Byte Size unknown " + str(len(message) + 1))
            return

        """""
        message = []
        byte_counter = 0
        #################################################
        for i in range(self.buffer.qsize()):
            byte = self.buffer.get(block=True, timeout=0.01)
            if byte_counter < NUM_BUF:
                byte_counter += 1

                if byte == 0:
                    if byte_counter == ADXL_NUM_BYTES + 2:
                        tmp = self.decoder.decode_message((ADXL_NUM_BYTES + 2), message)
                        if not tmp is None:
                            self.graph.add_element_to_adxl_data(tmp)
                    elif byte_counter == BASE_NUM_BYTES + 2:
                        self.decoder.decode_message((BASE_NUM_BYTES + 2), message)
                    elif byte_counter == BASE_NUM_BYTES_NEW + 2:
                        self.decoder.decode_message((BASE_NUM_BYTES_NEW + 2), message)
                    elif byte_counter == GPS_NUM_BYTES + 2:
                        self.decoder.decode_message((GPS_NUM_BYTES + 2), message)
                    elif byte_counter == TIMING_NUM_BYTES + 2:
                        self.decoder.decode_message((TIMING_NUM_BYTES + 2), message)
                    else:
                        print("USB Byte Size unknown " + str(byte_counter))
                    return
                else:
                    message.append(byte)
        ####################################################
        """

"""""
class DataReader(QObject):
    def __init__(self, connect_buffer_ref, window_graph_ref):
        super(DataReader, self).__init__()
        self.decoder = Decoder()
        self.buffer = connect_buffer_ref
        self.message_in_buffer = False
        self.finish_reading = False
        self.graph = window_graph_ref

    def read_from_buffer(self):
        while not self.finish_reading:
            if self.message_in_buffer:
                self.read_message()
                self.message_in_buffer = False

    def read_message(self):
        message = []
        byte_counter = 0
        for i in range(self.buffer.qsize()):
            byte = self.buffer.get(block=True, timeout=0.01)
            if byte_counter < NUM_BUF:
                byte_counter += 1

                if byte == 0:
                    if byte_counter == ADXL_NUM_BYTES + 2:
                        tmp = self.decoder.decode_message((ADXL_NUM_BYTES + 2), message)
                        if not tmp is None:
                            self.graph.add_element_to_adxl_data(tmp)
                    elif byte_counter == BASE_NUM_BYTES + 2:
                        self.decoder.decode_message((BASE_NUM_BYTES + 2), message)
                    elif byte_counter == BASE_NUM_BYTES_NEW + 2:
                        self.decoder.decode_message((BASE_NUM_BYTES_NEW + 2), message)
                    elif byte_counter == GPS_NUM_BYTES + 2:
                        self.decoder.decode_message((GPS_NUM_BYTES + 2), message)
                    elif byte_counter == TIMING_NUM_BYTES + 2:
                        self.decoder.decode_message((TIMING_NUM_BYTES + 2), message)
                    else:
                        print("USB Byte Size unknown " + str(byte_counter))
                    return
                else:
                    message.append(byte)
"""""