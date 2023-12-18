import numpy as np
from PySide6.QtWidgets import QPushButton, QVBoxLayout, QHBoxLayout, QPlainTextEdit, QWidget
import pyqtgraph as pg
from Static_Test.Input import adxl_uart_data_message

OVERRUN_DETECTION_TOLERANCE = 10000000
ACC_PLOTTEN = 0
GYR_PLOTTEN = 1
MAG_PLOTTEN = 2
MEMS_ACC_PLOTTEN = 3


class Graph(QWidget):
    def __init__(self):
        super(Graph, self).__init__()
        self.current_time = 0
        self.is_plotting = False
        self.adxl_data_list = [adxl_uart_data_message]
        self.timestamps = []
        self.overrun_counter = 0

        #region setup graph widget
        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setBackground('w')
        self.graph_widget.addLegend()
        self.graph_widget.showGrid(x=True, y=True)
        self.graph_widget.setMouseEnabled(x=False, y=False)
        self.graph_widget.setYRange(-10, 10)
        self.graph_widget.setLabel('left', "<span style=\"color:black;font-size:20px\">Voltage [mV/V]</span>")
        self.graph_widget.setLabel('bottom', "<span style=\"color:black;font-size:20px\">Time [s]</span>")
        self.test_counter = 0
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
        #endregion

    def create_plot(self, color, name: str):
        pen = pg.mkPen(color=color)

        return self.graph_widget.plot(list(np.linspace(0, (0 + 5), num=500)), ([0] * 500), name=name, pen=pen)

    def reset_values(self):
        self.timestamps = []
        self.list_of_y_values = [
            [],
            [],
            []
        ]
        self.graph_widget.clear()

    @staticmethod
    def update_plot(plot, x_values, y_values):
        plot.setData(x_values, y_values)

    def plot(self):
        self.add_value_to_y_values()
        for i in range(3):
            self.update_plot(self.plot_liste[i], self.timestamps, self.list_of_y_values[i])

    def add_element_to_adxl_data(self, message: adxl_uart_data_message):
        self.adxl_data_list.append(message)

    def add_value_to_y_values(self):
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
                self.list_of_y_values[0].extend(data.mems_x_acc[0:25])
                self.list_of_y_values[1].extend(data.mems_y_acc[0:25])
                self.list_of_y_values[2].extend(data.mems_z_acc[0:25])

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
        self.graph = Graph()
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


