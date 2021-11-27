# NB: NOT circuitpython

import re
import serial.tools.list_ports
import serial
from every.every import Every


class ArduinoPort:
    """Finds the 1st obvious arduino-port, and implements readline and write.
        supports disconnect (close), reconnect.
        Auto connnect.
        If we can't find a port, then readline/write are noops.
    """

    def __init__(self, pattern=None, baud=115200):
        """Provide a pattern, or we'll default to "arduino port name heuristics" """
        self.pattern = "^(/dev/ttyACM|/dev/cu\.usbmodem|COM)" if pattern == None else pattern
        self.port = None
        self.should_reconnect = True
        self.reconnect_time = Every(0.5, True)
        self.baud = baud
        self.reconnect()

    def disconnect(self, auto_reconnect=True):
        if self.port and self.port.is_open:
            self.port.close()
            self.port = None
            self.should_reconnect = auto_reconnect
        
    def reconnect(self):
        if not self.should_reconnect:
            return

        if not self.reconnect_time():
            return

        found_port = None
        for comport in serial.tools.list_ports.comports():
            print(comport.device)

            if re.search( self.pattern, comport.device):
                found_port = comport
                self.port = serial.Serial(comport.device, self.baud)
                print("! {}".format(comport.device))

        if found_port == None:
            print("No arduino port found yet")

    def write(self, one_char, flush=True):
        """arduino is probably only expecting a single char"""

        if not self.port:
            self.reconnect()

        if self.port and self.port.is_open:
            self.port.write(one_char.encode('utf-8'))
            if flush:
                self.port.flush()

    def readline(self):
        """arduino probably sends whole lines.
        THIS IS BLOCKING
        """

        if not self.port:
            self.reconnect()

        if self.port and self.port.is_open:
            return self.port.readline()

