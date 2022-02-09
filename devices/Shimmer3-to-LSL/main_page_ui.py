from PyQt5 import QtWidgets, uic, QtCore
import sys
import random
from edit_device_ui import EditDeviceWindow

from shimmer_lsl_device import ShimmerLSL


class DummyDevice(object):
    def __init__(self, port=''):
        if port:
            self.name = 'Device_{}'.format(port.replace('Port_', ''))
        else:
            self.name = 'Device_{}'.format(random.randint(0, 100))

    def __str__(self):
        return self.name

    @staticmethod
    def find_connected_ports():
        return ['Port_' + str(_id) for _id in range(3)]

    def get_device_name(self):
        return self.name

    def start_streaming(self):
        print(f"{self.name} started streaming!")

    def stop_streaming(self):
        print(f"{self.name} stopped streaming")


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()  # Call the inherited classes __init__ method
        uic.loadUi('main_window.ui', self)  # Load the .ui file

        self.setWindowTitle(f'PyShimmer LSL Interface')

        # Instance variables
        self.bt_ports = []
        self.devices = []

        # Connect Bluetooth button
        self.connect_to_bluetooth = self.findChild(QtWidgets.QPushButton, 'connect_to_bluetooth')
        self.connect_to_bluetooth.setCheckable(True)
        self.connect_to_bluetooth.pressed.connect(self.bt_button_clicked)
        self.connect_to_bluetooth.toggled.connect(self.bt_button_toggle)

        # Box for displaying info to user
        self.display_text = self.findChild(QtWidgets.QLabel, 'display_text')

        # List of devices
        self.device_list = self.findChild(QtWidgets.QListWidget, 'device_list')
        self.device_list.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.device_list.itemClicked.connect(self._display_num_items_selected)


        # Device operation buttons
        self.start_streaming_button = self.findChild(QtWidgets.QPushButton, 'start_streaming_button')
        self.start_streaming_button.clicked.connect(self.start_streaming)
        self.stop_streaming_button = self.findChild(QtWidgets.QPushButton, 'stop_streaming_button')
        self.stop_streaming_button.setEnabled(False)
        self.stop_streaming_button.clicked.connect(self.stop_streaming)
        self.edit_device_button = self.findChild(QtWidgets.QPushButton, 'edit_device_button')
        self.edit_device_button.clicked.connect(self.edit_devices)

        # Grey out these buttons, not in use
        self.stop_plotting_button = self.findChild(QtWidgets.QPushButton, 'stop_plotting_button')
        self.stop_plotting_button.setEnabled(False)
        self.start_recording_button = self.findChild(QtWidgets.QPushButton, 'start_recording_button')
        self.start_recording_button.setEnabled(False)
        self.stop_recording_button = self.findChild(QtWidgets.QPushButton, 'stop_recording_button')
        self.stop_recording_button.setEnabled(False)

        self.show()

    def bt_button_clicked(self):
        """
        On button click, disable and change check state. This triggers bt_button_toggle
        :return:
        """
        self.connect_to_bluetooth.setEnabled(False)
        self.connect_to_bluetooth.setText('Connecting to devices...')
        # self.bt_ports = ['Port1', 'Port2']
        self.connect_to_bluetooth.setChecked(True)
        self.connect_to_bluetooth.repaint()

    def bt_button_toggle(self):
        """
        Handle toggling of button. If it is disabled, we are connecting bluetooth.
        After connecting to bluetooth, update device list
        We reset checked state after bluetooth is connected.
        :return: None
        """
        if self.connect_to_bluetooth.isChecked():
            self.connect_to_bluetooth.repaint()
            # self.bt_ports = DummyDevice.find_connected_ports()
            self.bt_ports = ShimmerLSL.find_connected_ports()
            self.display_text.setText(f'Connected to {len(self.bt_ports)} devices')
            # self.bt_ports = ShimmerLSL.find_connected_ports()

            # Remake device list
            self._make_device_list()

            # Uncheck and return
            self.connect_to_bluetooth.setChecked(False)
        else:
            # Re-enable button
            self.connect_to_bluetooth.setText('Connect to Bluetooth')
            self.connect_to_bluetooth.setEnabled(True)

    def _make_device_list(self):
        """Setup devices from self.bt_ports and populate self.device_list"""
        self.devices = {}
        for port in self.bt_ports:
            # device = DummyDevice(port)
            device = ShimmerLSL(port)
            name = device.get_device_name()
            self.devices[name] = device
        self.device_list.clear()
        self.device_list.addItems(self.devices.keys())

        for item in self._iter_list_items():
            item.setCheckState(0)
            item.setSelected(0)

    def _iter_list_items(self):
        """Iterate through device_list items"""
        for i in range(self.device_list.count()):
            yield self.device_list.item(i)

    def start_streaming(self):
        """Look at device list for checked items and send start streaming command"""
        active_lsl_streams = []
        for item in self._iter_list_items():
            if item.checkState():
                key = item.text()
                self.devices[key].start_streaming()
                active_lsl_streams.append(self.devices[key].get_lsl_name())
            flags = item.flags()
            # Toggle off with AND
            flags &= ~QtCore.Qt.ItemIsEnabled
            item.setFlags(flags)
        # Disable button until we stop streaming
        self.start_streaming_button.setEnabled(False)
        self.stop_streaming_button.setEnabled(True)
        # Display text
        self.display_text.setText(f"Started streaming on LSL streams:\n{', '.join(name for name in active_lsl_streams)}")

    def stop_streaming(self):
        """Send stop streaming signals to the checked items in the device list and re-enable buttons"""
        for item in self._iter_list_items():
            if item.checkState():
                key = item.text()
                self.devices[key].stop_streaming()
            flags = item.flags()
            # Toggle on with OR
            flags |= QtCore.Qt.ItemIsEnabled
            item.setFlags(flags)
        # Enable button
        self.start_streaming_button.setEnabled(True)
        self.stop_streaming_button.setEnabled(False)
        # Display text
        self.display_text.setText('Stopped streaming')

    def edit_devices(self):
        """Make pop-up windows to edit devices"""
        self.edit_windows = []
        for item in self._iter_list_items():
            if item.checkState():
                key = item.text()
                device = self.devices[key]
                print('Editing device')
                window = EditDeviceWindow(device)
                window.show()
                self.edit_windows.append(window)

    def _display_num_items_selected(self):
        """Display number of items checked in display text"""
        num_checked = len([item for item in self._iter_list_items() if item.checkState()])
        self.display_text.setText(f"{num_checked} item(s) checked")


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)  # Create an instance of QtWidgets.QApplication
    window = MainWindow()  # Create an instance of our class
    app.exec_()  # Start the application
