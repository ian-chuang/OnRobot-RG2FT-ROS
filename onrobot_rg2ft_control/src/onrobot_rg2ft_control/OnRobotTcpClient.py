#!/usr/bin/env python3
"""
Module comModbusTcp: defines a class which communicates with
OnRobot Grippers using the Modbus/TCP protocol.
"""

import sys
import threading
from pymodbus.client.sync import ModbusTcpClient

class OnRobotTcpClient:
    """ communication sends commands and receives the status of RG gripper.

        Attributes:
            client (pymodbus.client.sync.ModbusTcpClient):
                instance of ModbusTcpClient to establish modbus connection
            lock (threading.Lock):
                instance of the threading.Lock to achieve exclusive control

            connectToDevice: Connects to the client device (gripper).
            disconnectFromDevice: Closes connection.
            sendCommand: Sends a command to the Gripper.
            getStatus: Sends a request to read and returns the gripper status.
    """

    def __init__(self):
        self.client = None
        self.lock = threading.Lock()

    def connect(self, ip, port, changer_addr=65):
        """ Connects to the client device (gripper).

            Args:
                ip (str): IP address (e.g. '192.168.1.1')
                port (str): port number (e.g. '502')
                changer_addr (int): quick tool changer address
        """

        self.client = ModbusTcpClient(
            ip,
            port=port,
            stopbits=1,
            bytesize=8,
            parity='E',
            baudrate=115200,
            timeout=1)
        self.changer_addr = changer_addr
        self.client.connect()

    def disconnect(self):
        """ Closes connection. """
        self.client.close()

    def write(self, address, message):
        """ Sends a command to the Gripper.

            Args:
                message (list[int]): message to be sent
        """

        # Sending a command to the device (address 0 ~ 2)
        if len(message) > 0:
            with self.lock:
                self.client.write_registers(
                    address=address, values=message, unit=self.changer_addr)

    def read(self, address, count):
        """ Sends a request to read and returns the gripper status. """

        # Getting status from the device (address 257 ~ 282)
        with self.lock:
            response = self.client.read_holding_registers(
                address=address, count=count, unit=self.changer_addr).registers

        # Output the result
        return response

    def restartPowerCycle(self):
        """ Restarts the power cycle of Compute Box.

            Necessary is Safety Switch of the grippers are pressed
            Writing 2 to this field powers the tool off
            for a short amount of time and then powers them back
        """

        message = 2
        restart_address = 63

        # Sending 2 to address 0x0 resets compute box (address 63) power cycle
        with self.lock:
            self.client.write_registers(
                address=0, values=message, unit=restart_address)
