from PyQt5 import QtWidgets, uic
import sys
from pyshimmer.device import ESensorGroup

from shimmer_lsl_device import ShimmerLSL


class DummyDevice(ShimmerLSL):
    """Dummy class for debugging"""
    def __init__(self):
        super().__init__('COM8')  # B0B1
        self.name = self.get_device_name()

    def write_out(self):
        """
        Write changes to device upon exit of UI
        Returns:
        --------
            None
        """
        if getattr(self, 'streams', None):
            print(self.streams)
            # Map streams to ESensorGroup
            stream_map = {
                'Gyroscope (IMU)': ESensorGroup.GYRO,
                'Mangetometer (IMU)': ESensorGroup.MAG,
                'Accelerometer Wide Range (IMU)': ESensorGroup.ACCEL_WR,
                'Accelerometer Low Noise (IMU)': ESensorGroup.ACCEL_LN,
                'Galvanic Skin Reponse (GSR)': ESensorGroup.GSR,
                'Photoplethysmography (PPG)': ESensorGroup.CH_A13
            }
            sensor_groups = [stream_map[s] for s in self.streams if stream_map.get(s, None) is not None]
            self.set_streams(sensor_groups)

        if getattr(self, 'lsl_stream_name', None):
            # self.set_lsl_name(self.lsl_name)
            pass

        if getattr(self, 'experiment_name', None):
            self.set_experiment_id(self.experiment_name)
        print('Changes written to device')

    def __str__(self):
        rep = ''
        dev_name = self.get_device_name()
        exp_name = self.get_experiment_id()
        _, _, streams = self.get_inquiry()
        rep = f"{dev_name}, {exp_name}, {streams}"
        return rep


# Create base class
class EditDeviceWindow(QtWidgets.QDialog):
    def __init__(self, device):
        super(EditDeviceWindow, self).__init__() # Call the inherited classes __init__ method
        uic.loadUi('edit_device.ui', self) # Load the .ui file

        # self.device = DummyDevice()
        self.device = device
        self.update_info = {
            'device_name': self.device.get_device_name(),
            'lsl_name': self.device.get_lsl_name(),
            'streams': [],
            'experiment_id': self.device.get_experiment_id()
        }
        if not self.update_info['lsl_name']:
            self.update_info['lsl_name'] = self.device.get_device_name()
        if not self.update_info['experiment_id']:
            self.update_info['experiment_id'] = 'Experiment_0'

        self.setWindowTitle(f'Settings for {self.update_info["device_name"]}')

        self.buttonbox = self.findChild(QtWidgets.QDialogButtonBox)
        self.buttonbox.accepted.connect(self.write_changes_to_device)
        self.buttonbox.rejected.connect(self.no_changes_to_device)

        # Experiment name input
        self.experiment_display_name = self.findChild(QtWidgets.QLabel, 'display_current_experiment')
        self.experiment_display_name.setText(self.update_info['experiment_id'])
        self.input_exp_name = self.findChild(QtWidgets.QLineEdit, 'input_experiment_name')
        self.input_exp_name.editingFinished.connect(self.update_exp_name)

        # LSL Name input
        self.lsl_display_name = self.findChild(QtWidgets.QLabel, 'display_current_lsl_name')
        self.lsl_display_name.setText(self.update_info['lsl_name'])

        self.input_lsl_name = self.findChild(QtWidgets.QLineEdit, 'input_lsl_stream')
        self.input_lsl_name.editingFinished.connect(self.update_lsl_stream)

        # Selecting sensor streams
        self.stream_list = self.findChild(QtWidgets.QListWidget, 'sensor_list_select')
        # Set default unchecked
        for item in self._iter_list_items():
            item.setCheckState(0)
            item.setSelected(0)
        self.stream_list.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.stream_list.itemClicked.connect(self.update_check_state)

    @staticmethod
    def update_check_state(item):
        curr_state = item.checkState()
        new_state = 0 if curr_state else 2
        item.setCheckState(new_state)
        item.setSelected(bool(new_state))

    def update_exp_name(self):
        exp_name = self.input_exp_name.text()
        self.experiment_display_name.setText(exp_name)
        self.update_info['experiment_id'] = exp_name

    def update_lsl_stream(self):
        lsl_name = self.update_info['device_name'] + '_' + self.input_lsl_name.text()
        self.lsl_display_name.setText(lsl_name)
        self.update_info['lsl_name'] = lsl_name

    def _iter_list_items(self):
        for i in range(self.stream_list.count()):
            yield self.stream_list.item(i)

    def update_streams(self):
        stream_names = [item.text() for item in self._iter_list_items() if item.checkState()]
        self.update_info['streams'] = stream_names

    def write_changes_to_device(self):
        self.update_streams()
        if self.update_info.get('streams', None):
            # Map stream names on UI to ESensorGroup
            stream_map = {
                'Gyroscope (IMU)': ESensorGroup.GYRO,
                'Mangetometer (IMU)': ESensorGroup.MAG,
                'Accelerometer Wide Range (IMU)': ESensorGroup.ACCEL_WR,
                'Accelerometer Low Noise (IMU)': ESensorGroup.ACCEL_LN,
                'Galvanic Skin Reponse (GSR)': ESensorGroup.GSR,
                'Photoplethysmography (PPG)': ESensorGroup.CH_A13
            }
            sensor_groups = [stream_map[s] for s in self.update_info['streams'] if stream_map.get(s, None) is not None]
            self.device.set_streams(sensor_groups)

        if self.update_info.get('lsl_name', None):
            self.device.set_lsl_name(self.update_info['lsl_name'])

        if self.update_info.get('experiment_id', None):
            self.device.set_experiment_id(self.update_info['experiment_id'])

        if self.update_info.get('device_name', None):
            self.device.set_device_name(self.update_info['device_name'])

        print(self.device)

    def no_changes_to_device(self):
        print(self.device)
        print('Changes not saved')


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)  # Create an instance of QtWidgets.QApplication
    window = EditDeviceWindow(DummyDevice())  # Create an instance of our class
    window.show()
    app.exec_()  # Start the application
