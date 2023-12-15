from Static_Test.GUI import MainWindow
from Static_Test.Input import Connection
from PySide6.QtCore import QThread
from PySide6.QtWidgets import QApplication
import sys
from time import sleep


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

    def start_connection(self):
        self.connection.finish_reading = False
        self.connection_thread.start()

    def start_reading_data(self):
        sleep(8)
        self.window.graph.plot()

    def stop_connection(self):
        self.connection.finish_reading = True
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