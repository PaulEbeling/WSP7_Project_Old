import numpy as np
from PySide6.QtCore import QThread, QTimer, Signal
from PySide6.QtWidgets import QPushButton, QVBoxLayout, QHBoxLayout, QPlainTextEdit, QWidget
import pyqtgraph as pg
from Performance_Test_Pyserial.Input import adxl_uart_data_message
import time

OVERRUN_DETECTION_TOLERANCE = 10000000
ACC_PLOTTEN = 0
GYR_PLOTTEN = 1
MAG_PLOTTEN = 2
MEMS_ACC_PLOTTEN = 3


"""""
import numpy as np

liste = [0,5,10,15,20,25,30]
liste2 = []

for i in range(len(liste) - 1):
    liste2.extend(np.linspace(liste[i], liste[i + 1],6))
    if i != (len(liste) - 2):
        del liste2[-1]
    
print(liste2)
"""""


class Graph(QWidget):
    def __init__(self, graph_type):
        super(Graph, self).__init__()
        self.is_plotting = False
        self.adxl_data_list = [adxl_uart_data_message]
        self.timestamps = []
        self.overrun_counter = 0
        self.graph_type = graph_type

        #region setup graph widget
        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setBackground('w')
        self.graph_widget.addLegend()
        self.graph_widget.showGrid(x=True, y=True)
        self.graph_widget.setMouseEnabled(x=False, y=False)
        self.graph_widget.setYRange(-4, 4)
        self.graph_widget.setLabel('left', "<span style=\"color:black;font-size:20px\">Voltage [mV/V]</span>")
        self.graph_widget.setLabel('bottom', "<span style=\"color:black;font-size:20px\">Time [s]</span>")
        self.time_start = 0
        #endregion

        self.list_of_y_values = [
            [],
            [],
            []
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

        self.update_plot_timer = QTimer()
        #endregion

    def create_plot(self, color, name: str):
        pen = pg.mkPen(color=color, width=0.6)

        return self.graph_widget.plot(list(np.linspace(0, (0 + 10), num=1600)), ([0] * 1600), name=name, pen=pen)

    @staticmethod
    def update_plot(plot, x_values, y_values):
        plot.setData(x_values, y_values, autoDownsample=True)

    def start_plotting(self):
        #region setup timer
        self.update_plot_timer = QTimer()
        self.update_plot_timer.setInterval(0.01667)
        if self.graph_type == MAG_PLOTTEN:
            self.update_plot_timer.timeout.connect(self.read_mag_data_from_adxl_data)
        elif self.graph_type == ACC_PLOTTEN:
            self.update_plot_timer.timeout.connect(self.read_acc_data_from_adxl_data)
        elif self.graph_type == GYR_PLOTTEN:
            self.update_plot_timer.timeout.connect(self.read_gyr_data_from_adxl_data)
        elif self.graph_type == MEMS_ACC_PLOTTEN:
            self.update_plot_timer.timeout.connect(self.read_fast_acc_data_from_adxl_data)
        #endregion
        self.update_plot_timer.start()

    def stop_plotting(self):
        self.update_plot_timer.stop()
        self.reset_values()

    def reset_values(self):
        self.timestamps = []
        self.list_of_y_values = [
            [],
            [],
            []
        ]
        self.graph_widget.clear()

        self.plot_liste = [
            self.create_plot((0, 255, 0), 'X-ACC'),
            self.create_plot((0, 0, 255), 'Y-ACC'),
            self.create_plot((255, 0, 0), 'Z-ACC')
        ]

    def plot(self):
        if self.timestamps[-1] > 10:
            self.graph_widget.setXRange(self.timestamps[-1] - 10, self.timestamps[-1])
        for i in range(3):
            self.update_plot(self.plot_liste[i], self.timestamps, self.list_of_y_values[i])

    def add_element_to_adxl_data(self, message: adxl_uart_data_message):
        self.adxl_data_list.append(message)

    def read_acc_data_from_adxl_data(self):
        adxl_data_list_tmp = self.adxl_data_list.copy()
        adxl_data_list_tmp_len = len(adxl_data_list_tmp)
        start = 0

        if adxl_data_list_tmp_len > 1:
            del self.adxl_data_list[0:len(adxl_data_list_tmp)]

            for i in range(len(adxl_data_list_tmp[0: adxl_data_list_tmp_len])):
                current_timestamp = adxl_data_list_tmp[i].timestamp / 1000000
                real_time = (adxl_data_list_tmp[i].timestamp / 1000000) + (4294967296 * self.overrun_counter)
                try:
                    last_timestamp = self.timestamps[-1]

                    if current_timestamp < OVERRUN_DETECTION_TOLERANCE and last_timestamp > (4294967296 - OVERRUN_DETECTION_TOLERANCE):
                        print("Timer overrun")
                        self.overrun_counter += 1
                    elif current_timestamp < last_timestamp:
                        print("new Timestamp smaller than last")
                        self.reset_values()
                        start = i
                except IndexError:
                    pass
                self.timestamps.append(real_time)

            [self.list_of_y_values[0].append(element.x_acc) for element in
             adxl_data_list_tmp[start: adxl_data_list_tmp_len]]
            [self.list_of_y_values[1].append(element.y_acc) for element in
             adxl_data_list_tmp[start: adxl_data_list_tmp_len]]
            [self.list_of_y_values[2].append(element.z_acc) for element in
             adxl_data_list_tmp[start: adxl_data_list_tmp_len]]
            #print(self.timestamps)

            self.plot()

    def read_gyr_data_from_adxl_data(self):
        adxl_data_list_tmp = self.adxl_data_list.copy()
        adxl_data_list_tmp_len = len(adxl_data_list_tmp)
        start = 0

        if adxl_data_list_tmp_len > 1:
            del self.adxl_data_list[0:len(adxl_data_list_tmp)]

            for i in range(len(adxl_data_list_tmp[0: adxl_data_list_tmp_len])):
                current_timestamp = adxl_data_list_tmp[i].timestamp / 1000000
                real_time = (adxl_data_list_tmp[i].timestamp / 1000000) + (4294967296 * self.overrun_counter)
                try:
                    last_timestamp = self.timestamps[-1]

                    if current_timestamp < OVERRUN_DETECTION_TOLERANCE and last_timestamp > (4294967296 - OVERRUN_DETECTION_TOLERANCE):
                        print("Timer overrun")
                        self.overrun_counter += 1
                    elif current_timestamp < last_timestamp:
                        print("new Timestamp smaller than last")
                        self.reset_values()
                        start = i
                except IndexError:
                    pass
                self.timestamps.append(real_time)

            [self.list_of_y_values[0].append(element.x_gyr) for element in
             adxl_data_list_tmp[start: adxl_data_list_tmp_len]]
            [self.list_of_y_values[1].append(element.y_gyr) for element in
             adxl_data_list_tmp[start: adxl_data_list_tmp_len]]
            [self.list_of_y_values[2].append(element.z_gyr) for element in
             adxl_data_list_tmp[start: adxl_data_list_tmp_len]]
            #print(self.timestamps)

            self.plot()

    def read_mag_data_from_adxl_data(self):
        adxl_data_list_tmp = self.adxl_data_list.copy()
        adxl_data_list_tmp_len = len(adxl_data_list_tmp)
        start = 0

        if adxl_data_list_tmp_len > 1:
            del self.adxl_data_list[0:len(adxl_data_list_tmp)]

            for i in range(len(adxl_data_list_tmp[0: adxl_data_list_tmp_len])):
                current_timestamp = adxl_data_list_tmp[i].timestamp / 1000000
                real_time = (adxl_data_list_tmp[i].timestamp / 1000000) + (4294967296 * self.overrun_counter)
                try:
                    last_timestamp = self.timestamps[-1]

                    if current_timestamp < OVERRUN_DETECTION_TOLERANCE and last_timestamp > (4294967296 - OVERRUN_DETECTION_TOLERANCE):
                        print("Timer overrun")
                        self.overrun_counter += 1
                    elif current_timestamp < last_timestamp:
                        print("new Timestamp smaller than last")
                        self.reset_values()
                        start = i
                except IndexError:
                    pass
                self.timestamps.append(real_time)

            [self.list_of_y_values[0].append(element.x_mag) for element in
             adxl_data_list_tmp[start: adxl_data_list_tmp_len]]
            [self.list_of_y_values[1].append(element.y_mag) for element in
             adxl_data_list_tmp[start: adxl_data_list_tmp_len]]
            [self.list_of_y_values[2].append(element.z_mag) for element in
             adxl_data_list_tmp[start: adxl_data_list_tmp_len]]
            #print(self.timestamps)

            self.plot()

    def read_fast_acc_data_from_adxl_data(self):
        adxl_data_list_tmp = self.adxl_data_list.copy()
        adxl_data_list_tmp_len = len(adxl_data_list_tmp)
        start = 0

        if adxl_data_list_tmp_len > 1:
            del self.adxl_data_list[0:len(adxl_data_list_tmp)]

            for i in range(len(adxl_data_list_tmp[0: adxl_data_list_tmp_len])):
                try:
                    last_timestamp = self.timestamps[-1]
                    current_timestamp = adxl_data_list_tmp[i].timestamp / 1000000

                    if current_timestamp < OVERRUN_DETECTION_TOLERANCE and last_timestamp > (
                            4294967296 - OVERRUN_DETECTION_TOLERANCE):
                        print("Timer overrun")
                        self.overrun_counter += 1
                    elif current_timestamp < last_timestamp:
                        print("new Timestamp smaller than last")
                        self.reset_values()
                        start = i

                    real_timestamp = (adxl_data_list_tmp[i].timestamp / 1000000) + (
                            4294967296 * self.overrun_counter)
                    interpolierte_timestamps = (np.linspace(last_timestamp, real_timestamp, 25))
                    self.timestamps.extend(interpolierte_timestamps)
                except IndexError:
                    current_timestamp = adxl_data_list_tmp[i].timestamp / 1000000
                    real_timestamp = current_timestamp + (4294967296 * self.overrun_counter)
                    interpolierte_timestamps = (np.linspace(0, real_timestamp, 25))
                    self.timestamps.extend(interpolierte_timestamps)

            for data in adxl_data_list_tmp[start:adxl_data_list_tmp_len]:
                for i in range(25):
                    try:
                        self.list_of_y_values[0].append(data.mems_x_acc[i])
                    except IndexError:
                        print("Mems X ACC values: " + str(len(data.mems_x_acc)) + " i: " + str(i))
                    self.list_of_y_values[1].append(data.mems_y_acc[i])
                    self.list_of_y_values[2].append(data.mems_z_acc[i])

            #print(len(self.list_of_y_values[0]))
            #print(len(self.timestamps))
            # print(self.timestamps)

            self.plot()


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
        self.graphs = [
            Graph(0),
            Graph(MEMS_ACC_PLOTTEN)
        ]

        self.threads = [
            QThread(),
            QThread()
        ]

        for i in range(2):
            self.threads[i] = QThread()
            self.graphs[i].moveToThread(self.threads[i])
            self.threads[i].started.connect(self.graphs[i].start_plotting)
        #endregion

        #region setup layout
        button_layout = QHBoxLayout()

        button_layout.addWidget(self.connect_button)
        button_layout.addWidget(self.disconnect_button)

        widget_layout = QVBoxLayout()
        widget_layout.addLayout(button_layout)
        widget_layout.addWidget(self.graphs[0])
        widget_layout.addWidget(self.graphs[1])
        widget_layout.addWidget(self.textfield_debugger)

        self.setLayout(widget_layout)
        #endregion

    def start_plotting(self):
        for thread in self.threads:
            thread.start()

    def stop_plotting(self):
        for graph in self.graphs:
            graph.stop_plotting()
        for thread in self.threads:
            thread.terminate()
            thread.wait()

