"""
Example implementation of a threaded UBXMessage streamer

Connects to the receiver's serial port and sets up a
threaded UBXReader process. With the reader process running
in the background, it polls the current PRT, USB, NMEA and MSG
configuration.

You should see the poll responses in the input stream,
or an ACK-NAK (Not Acknowledged) message if that
particular CFG-MSG type is not supported by the receiver.

Created on 2 Oct 2020

@author: semuadmin
"""

from io import BufferedReader
from threading import Thread
from time import sleep

from pyubx2 import UBXReader, UBXMessage, VALCKSUM, POLL, SET, UBX_MSGIDS
from serial import Serial, SerialException, SerialTimeoutException

import pyubx2.exceptions as ube


class UBXStreamer:
    """
    UBXStreamer class.
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

        self.stop_read_thread()
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

    def send(self, data):
        """
        Send data to serial connection.
        """

        self._serial_object.write(data)

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


if __name__ == "__main__":

    YES = ("Y", "y", "YES,", "yes", "True")
    NO = ("N", "n", "NO,", "no", "False")
    PAUSE = 1

    print("Enter port: ", end="")
    val = input().strip('"')
    prt = val
    print("Enter baud rate (9600): ", end="")
    val = input().strip('"') or '9600'
    baud = int(val)
    print("Enter timeout (0.1): ", end="")
    val = input().strip('"') or '0.1'
    timout = float(val)
    print("Do you want to ignore any non-UBX data (y/n)? (y) ", end="")
    val = input() or "y"
    ubxonly = val in NO

    print("Instantiating UBXStreamer class...")
    ubp = UBXStreamer(prt, baud, timout, ubxonly)
    print(f"Connecting to serial port {prt} at {baud} baud...")
    if ubp.connect():
        print("Starting reader thread...")
        ubp.start_read_thread()

        print("\nPolling receiver configuration...\n")
        # poll the receiver configuration
        for prt in (0, 1, 2, 3, 4):  # I2C, UART1, UART2, USB, SPI
            msg = UBXMessage("CFG", "CFG-PRT", POLL, portID=prt)
            ubp.send(msg.serialize())
            sleep(PAUSE)
        for msgtype in ("CFG-USB", "CFG-NMEA", "CFG-NAV5"):
            msg = UBXMessage("CFG", msgtype, POLL)
            ubp.send(msg.serialize())
            sleep(PAUSE)

        # poll a selection of current navigation message rates using CFG-MSG
        print("\nPolling navigation message rates...\n")
        for msgid in UBX_MSGIDS.keys():
            if msgid[0] in (1, 240, 241):  # NAV, NMEA-Standard, NMEA-Proprietary
                msg = UBXMessage("CFG", "CFG-MSG", POLL, payload=msgid)
                ubp.send(msg.serialize())
                sleep(1)
        print("\n\nPolling complete, waiting for final responses...\n\n")

        sleep(PAUSE)

        print("\n\nStopping reader thread...")
        ubp.stop_read_thread()
        print("Disconnecting from serial port...")
        ubp.disconnect()
        print("Test Complete")
