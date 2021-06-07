
import digitalio
import board
from io import BufferedReader
from threading import Thread
from time import sleep

from pyubx2 import UBXReader, UBXMessage, VALCKSUM, POLL, SET, UBX_MSGIDS
from serial import Serial, SerialException, SerialTimeoutException

import pyubx2.exceptions as ube

class GPSReader:
    """
    GPSReader class
    """
    def __init__(self, port, baudrate, timeout=5, ubx_only=False):
        """
        Constructor.
        """

        self._serial_object = None
        self._serial_thread = None
        self._ubxreader = None
        self._connected = False
        self._reading = False
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._ubx_only = ubx_only

    def __del__(self):
        """
        Destructor.
        """

        self.disconnect()

    def connect(self):
        """
        Open serial connection.
        """

        self._connected = False
        try:
            self._serial_object = Serial(
                self._port, self._baudrate, timeout=self._timeout
            )
            self._ubxreader = UBXReader(BufferedReader(self._serial_object), ubxonly=self._ubx_only)
            self._connected = True
        except (SerialException, SerialTimeoutException) as err:
            print(f"Error connecting to serial port {err}")

        return self._connected

    def disconnect(self):
        """
        Close serial connection.
        """

        if self._connected and self._serial_object:
            try:
                self._serial_object.close()
            except (SerialException, SerialTimeoutException) as err:
                print(f"Error disconnecting from serial port {err}")
        self._connected = False

        return self._connected

    def read(self):
        """
        """
        if self._serial_object:
            if self._serial_object.in_waiting:
                try: 
                    (raw_data, parsed_data) = self._ubxreader.read()
                    if parsed_data:
                        return parsed_data
                except (
                    ube.UBXStreamError,
                    ube.UBXMessageError,
                    ube.UBXTypeError,
                    ube.UBXParseError,
                ) as err:
                    print(f"Something went wrong {err}")
                    return False
        return False

    '''
    def start_read_thread(self):
        """
        Start the serial reader thread.
        """

        if self._connected:
            self._reading = True
            self._serial_thread = Thread(target=self._read_thread, daemon=True)
            self._serial_thread.start()

    def stop_read_thread(self):
        """
        Stop the serial reader thread.
        """

        if self._serial_thread is not None:
            self._reading = False

    '''
    def _send(self, data):
        """
        Send data to serial connection.
        """

        self._serial_object.write(data)
        
    def config_msg(self, config):
        """
        Creates a series of CFG-MSG configuration messages and
        sends them to the receiver.
        """

        try:

            msgs = []

            # compile all the UBX-NAV config message types
            for key, val in UBX_MSGIDS.items():
                if val == "NAV-PVT":
                # if val[0:3] == "NAV":
                    msgs.append(key)

            # send each UBX-NAV config message in turn
            for msgtype in msgs:
                payload = msgtype + config
                msg = UBXMessage("CFG", "CFG-MSG", SET, payload=payload)
                print(f"Sending {msg}")
                self._send(msg.serialize())
                sleep(1)

        except (ube.UBXMessageError, ube.UBXTypeError, ube.UBXParseError) as err:
            print(f"Something went wrong {err}")

    def config_timepulse(self):
        '''
        Not working.
        '''
        try: 
            # {'tpIdx': 'U001', 'reserved0': 'U001', 'reserved1': 'U002', 'antCableDelay': 'I002', 'rfGroupDelay': 'I002', 
            # 'freqPeriod': 'U004', 'freqPeriodLock': 'U004', 'pulseLenRatio': 'U004', 'pulseLenRatioLock': 'U004', 
            # 'userConfigDelay': 'I004', 'flags': 'X004'}
            payload = {
                "tpIdx": 0, # b"\x00", # timepulse selection
                # 0x00, # message version
                "reserved0": int.from_bytes(b"\x00", "little"), # reserved
                "reserved1": int.from_bytes(b"\x00", "little"), # reserved
                "antCableDelay": int.from_bytes(b"\x00\x00", "little"), # Antenna delay
                "rfGroupDelay": int.from_bytes(b"\x00\x00", "little"), # RF delay
                "freqPeriod": int.from_bytes(b"\x01\x00\x00\x00", "little"), # frequency in Hz
                "freqPeriodLock": int.from_bytes(b"\x01\x00\x00\x00", "little"), # frequency in Hz when GPS time is locked in
                "pulseLenRatio": int.from_bytes(b"\xff\xff\xff\x7f", "little"), # 50% pulse width
                "pulseLenRatioLock": int.from_bytes(b"\xff\xff\xff\x7f", "little"), # 50% pulse width when locked to GPS time
                "userConfigDelay": int.from_bytes(b"\x00\x00\x00\x00", "little"), # user configurable delay
                "flags": b"\x00\x00\x00\x69"  # flags
            }
            msg = UBXMessage("CFG", "CFG-TP5", SET, **payload)
            print(f"Sending {msg.serialize()}")
            self._send(msg.serialize())
            sleep(1)
        except (ube.UBXMessageError, ube.UBXTypeError, ube.UBXParseError) as err:
            print(f"Something went wrong {err}")

    def config_timepulse_badly(self):
        custom_string = b"\xb5\x62\x06\x31\x20\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x00\x01\x00\x00\x00\xff\xff\xff\x7f\xff\xff\xff\x7f\x00\x00\x00\x00\x69\x00\x00\x00\xba\x37"
        print(f"Sending {custom_string}")
        self._send(custom_string)
        sleep(0.1)

    def flush(self):
        """
        Flush input buffer
        """

        self._serial_object.reset_input_buffer()

    def waiting(self):
        """
        Check if any messages remaining in the input buffer
        """

        return self._serial_object.in_waiting

    '''
    def _read_thread(self):
        """
        THREADED PROCESS
        Reads and parses UBX message data from stream
        """

        while self._reading and self._serial_object:
            if self._serial_object.in_waiting:
                try:
                    (raw_data, parsed_data) = self._ubxreader.read()
                    #                     if raw_data:
                    #                         print(raw_data)
                    if parsed_data:
                        print(parsed_data)
                except (
                    ube.UBXStreamError,
                    ube.UBXMessageError,
                    ube.UBXTypeError,
                    ube.UBXParseError,
                ) as err:
                    print(f"Something went wrong {err}")
                    continue
    '''
if __name__ == "__main__":
    timepulse = digitalio.DigitalInOut(board.D26)
    timepulse.direction = digitalio.Direction.INPUT 

    PORT = "/dev/ttyACM0"
    BAUDRATE = 9600
    TIMEOUT = 1
    UBXONLY = False
    ON = b"\x00\x01\x01\x01\x00\x00"
    OFF = b"\x00\x00\x00\x00\x00\x00"
    ubp = GPSReader(PORT, BAUDRATE, TIMEOUT, UBXONLY)

    if ubp.connect():
        # import pdb;pdb.set_trace()
        ubp.config_timepulse_badly()
        ubp.config_msg(ON)
        while True:
            # if timepulse.value:
            nav_msg = ubp.read()
            print(nav_msg)
            print(timepulse.value)
            sleep(0.5)
