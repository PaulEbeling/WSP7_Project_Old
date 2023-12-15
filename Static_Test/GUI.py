import numpy as np
from PySide6.QtCore import QThread, QTimer, Signal
from PySide6.QtWidgets import QPushButton, QVBoxLayout, QHBoxLayout, QPlainTextEdit, QWidget
import pyqtgraph as pg
from queue import Queue
from Static_Test.Input import adxl_uart_data_message
from time import sleep


class Graph(QWidget):
    def __init__(self):
        super(Graph, self).__init__()
        self.current_time = 0
        self.is_plotting = False
        self.adxl_data_queue = Queue()
        self.adxl_data_list = [adxl_uart_data_message]

        #region setup graph widget
        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setBackground('w')
        self.graph_widget.addLegend()
        self.graph_widget.showGrid(x=True, y=True)
        self.graph_widget.setMouseEnabled(x=False, y=False)
        self.graph_widget.setYRange(-1, 1)
        self.graph_widget.setLabel('left', "<span style=\"color:black;font-size:20px\">Voltage [mV/V]</span>")
        self.graph_widget.setLabel('bottom', "<span style=\"color:black;font-size:20px\">Time [s]</span>")
        self.test_counter = 0
        #endregion

        self.list_of_y_values = [
            [0] * 500,
            [0] * 500,
            [0] * 500
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

    @staticmethod
    def update_plot(plot, x_values, y_values):
        plot.setData(x_values, y_values)

    def plot(self):
        self.add_value_to_y_values()
        for i in range(3):
            self.update_plot(self.plot_liste[i], list(np.linspace(5,  10, num=500)), self.list_of_y_values[i])

    def add_element_to_adxl_data(self, message: adxl_uart_data_message):
        self.adxl_data_list.append(message)

    def add_value_to_y_values(self):
        adxl_data_list_tmp = self.adxl_data_list.copy()
        adxl_data_list_tmp_len = len(adxl_data_list_tmp)

        if adxl_data_list_tmp_len > 1:
            if adxl_data_list_tmp_len > 500:
                adxl_data_list_tmp_len = 500

            del self.adxl_data_list[0:len(adxl_data_list_tmp)]
            for i in range(3):
                del self.list_of_y_values[i][0:adxl_data_list_tmp_len]

            [self.list_of_y_values[0].append(element.x_acc) for element in adxl_data_list_tmp[0: adxl_data_list_tmp_len]]
            [self.list_of_y_values[1].append(element.y_acc) for element in adxl_data_list_tmp[0: adxl_data_list_tmp_len]]
            [self.list_of_y_values[2].append(element.z_acc) for element in adxl_data_list_tmp[0: adxl_data_list_tmp_len]]


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


