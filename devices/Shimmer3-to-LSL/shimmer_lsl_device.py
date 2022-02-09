import time

from serial import Serial
from serial.tools import list_ports
from pyshimmer import ShimmerBluetooth, DEFAULT_BAUDRATE
from serial.serialutil import SerialTimeoutException
from pyshimmer.device import *

from functools import partial
import itertools
from pylsl import StreamInfo, StreamOutlet


# Info dictionaries for the streams
units = {'GSR_KOHM': 'kOhms',
             'PPG_MV': 'mV'}
stream_types = {'GSR_KOHM': 'EDA',
                'PPG_MV': 'EDA'}

for k in itertools.product(['ACCEL_LN_', 'ACCEL_LSM303DLHC', 'GYRO_MPU9150_', 'MAG_LSM303DLHC'], ['X', 'Y', 'Z']):
    k = ''.join(k)
    units.update({k: 'None'})
    stream_types.update({k: 'IMU'})


class ShimmerLSL(ShimmerBluetooth):
    """
    Extended class of the Shimmer Bluetooth API
    Useful methods of the base class include:
        (get|set)_device_name
        (get|set)_experiment_id
        get_sampling_rate
        (start|stop)_streaming
        (add|remove)_stream_callback

    This class implements the following public methods for use to connect to LSL
        (get|set)_streams --> Actually writes changes to device
        (get|set)_lsl_name
        get_lsl_streams
    """

    def __init__(self, port, device_name='', lsl_name='', exp_name=''):
        """
        Connect ShimmerBluetooth device over serial
        Set any names if given
        Parameters:
        -----------
            port: str
            device_name: str
            lsl_name: str
                Name of LSL stream
            exp_name: str
        Returns:
        --------
            None
        """
        serial = Serial(port, DEFAULT_BAUDRATE)
        super(ShimmerLSL, self).__init__(serial)
        self.initialize()
        print(f'Synching clocks for {self.get_device_name()}')
        self.set_rtc(time.time())
        # Setup current streams EChannelTypes
        self.ch_types = self.get_data_types()
        self.lsl_outlet = None
        self.lsl_name = None

        if device_name:
            self.set_device_name(device_name)
        if lsl_name:
            self.set_lsl_name(lsl_name)
        if exp_name:
            self.set_experiment_id(exp_name)

    def set_lsl_name(self, lsl_name):
        self.lsl_name = lsl_name

    def get_lsl_name(self):
        if self.lsl_name:
            return self.lsl_name
        else:
            return self.get_device_name()

    def set_streams(self, sensor_groups):
        """
        Set the streams from a list of ESensorGroup
        The pyshimmer API stores the types as Enum members that do different parts of setup
        Certain functions require EChannelType, ESensorGroup, and ChannelDataType
        Reset the lsl stream names after reassignment

        Parameters:
        -----------
            streams: list of ESensorGroup
        Returns:
        --------
            None
        """
        # Get the channel types from all the groups
        channel_types = []
        for group in sensor_groups:
            channel_types.extend(SensorChannelAssignment[group])

        # Bluetooth write command requires a tuple of (EChannelType, ChannelDataType) to write to device
        stream_types = [(t, ChDataTypeAssignment[t]) for t in channel_types]
        self._bluetooth.set_stream_types(stream_types)

        # Set sensors command requires just the list of groups
        self.set_sensors(sensor_groups)

    def get_lsl_streams(self):
        """
        Convert Shimmer stream names into LSL stream names

        Returns:
        --------
            lsl_streams: list
        """
        lsl_streams = []
        for s in self.get_streams():
            if s == 'INTERNAL_ADC_13':
                lsl_streams.append('PPG_MV')
            elif s == 'GSR_RAW':
                lsl_streams.append('GSR_KOHM')
            else:
                lsl_streams.append(s)
        return lsl_streams

    def get_streams(self):
        """
        Get string names of streams from EChannelTypes
        Parameters:
        -----------
            None
        Returns:
        --------
            streams: list of string
        """
        fs, _, channels = self.get_inquiry()
        return [ch.name for ch in channels]

    def _streaming_callback(self, pkt):
        """
        Streaming callback that intakes packet and writes to LSL
        Does conversion of PPG and GSR data.
        Currently not setup to convert raw IMU data to physical values

        Parameters:
        -----------
            pkt: LSL DataPacket
        Returns:
        --------
            None
        """
        sample = []
        for ch_type in self.ch_types:
            if ch_type.name == 'TIMESTAMP':
                # Use LSL timestamp
                continue
            elif ch_type.name == 'GSR_RAW':
                sample.append(ShimmerLSL._convert_gsr(pkt[ch_type]))
            elif ch_type.name == 'INTERNAL_ADC_13':
                sample.append(ShimmerLSL._convert_ppg(pkt[ch_type]))
            else:
                sample.append(pkt[ch_type])
        self.lsl_outlet.push_sample(sample)

    def start_streaming(self):
        """
        Setup LSL outlet
        Invoke Shimmer Bluetooth API start streaming command
        """
        self.setup_lsl_outlet()
        self.add_stream_callback(self._streaming_callback)
        super().start_streaming()

    def stop_streaming(self):
        """
        Remove LSL streaming callback and stop streaming
        """
        super().stop_streaming()
        self.remove_stream_callback(self._streaming_callback)

    @staticmethod
    def _convert_ppg(PPG_raw):
        """Convert PPG to mV"""
        PPG_mv = PPG_raw * (3000.0 / 4095.0)
        return PPG_mv

    @staticmethod
    def _convert_gsr(GSR_raw):
        """Convert GSR to kOhm with encoded resistor information"""
        # get current GSR range resistor value
        # May be able to set the constant range and skip this checking.
        Range = ((GSR_raw >> 14) & 0xff)  # upper two bits
        if (Range == 0):
            Rf = 40.2  # kohm
        elif (Range == 1):
            Rf = 287.0  # kohm
        elif (Range == 2):
            Rf = 1000.0  # kohm
        elif (Range == 3):
            Rf = 3300.0  # kohm

        # convert GSR to kohm value
        gsr_to_volts = (GSR_raw & 0x3fff) * (3.0 / 4095.0)
        GSR_ohm = Rf / ((gsr_to_volts / 0.5) - 1.0)
        return GSR_ohm

    def setup_lsl_outlet(self):
        """
        Creates LSL outlet based on current device info and attaches to self.lsl_outlet

        Parameters:
        -----------
            None
        Returns:
        --------
            None
        """
        channel_names = self.get_lsl_streams()

        n_channels = len(channel_names)
        sample_rate = self.get_sampling_rate()
        stream_type = 'Shimmer3'
        name = self.get_lsl_name()

        # Setup metadata
        info = StreamInfo(name, stream_type, n_channels, sample_rate, 'float32', self.get_device_name())
        info.desc().append_child_value("manufacturer", "Shimmer Sensing")
        channels = info.desc().append_child("channels")

        for i, label in enumerate(channel_names):
            ch = channels.append_child("channel")
            ch.append_child_value("label", label)
            ch.append_child_value("unit", units.get(label, ''))
            ch.append_child_value("type", stream_types.get(label, ''))

        # Setup LSL Stream
        self.lsl_outlet = StreamOutlet(info)

    @staticmethod
    def find_connected_ports():
        """
        Check active COM ports (on Windows) for a shimmer device.

        Returns: list
            list of (port, device_name) tuples
        """
        print('Finding active ports with shimmer devices')
        comlist = list_ports.comports()
        ports = []
        for port, desc, hwid in sorted(comlist):
            ports.append(port)
        active_ports = []
        for port in ports:
            # if port == 'COM20':
            #     continue
            try:
                with Serial(port, baudrate=DEFAULT_BAUDRATE, write_timeout=2.0) as ser:
                    print(f'Attempting connection on {port}...')
                    shim_dev = ShimmerBluetooth(ser)
                    shim_dev.initialize()
                    try:
                        shim_dev.send_ping()  # If this device is connected, then it will not exceed the timeout
                        dev_name = shim_dev.get_device_name()
                        print(f'Device {dev_name} connected on {port}')
                        # active_ports.append((port, dev_name))
                        active_ports.append(port)
                    # If no device, ping write will timeout and raise SerialTimeoutException
                    except SerialTimeoutException:
                        print(f'No device on {port}')
                    shim_dev.shutdown()
                    # time.sleep(0.1)
            except OSError as e:  # Ports that were connected but have no active device on Windows return OS Error
                print(f'Could not connect on {port} due to OSError {e}')
                continue
        return active_ports


