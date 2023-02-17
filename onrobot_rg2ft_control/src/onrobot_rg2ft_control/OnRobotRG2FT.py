#!/usr/bin/env python3

import rospy
from onrobot_rg2ft_control.OnRobotTcpClient import OnRobotTcpClient
from onrobot_rg2ft_msgs.msg import RG2FTCommand, RG2FTState
from onrobot_rg2ft_msgs.srv import SetProximityOffsetsRequest
import struct

RG2FT_MIN_WIDTH = 0
RG2FT_MAX_WIDTH = 1000
RG2FT_MIN_FORCE = 0
RG2FT_MAX_FORCE = 400
RG2FT_DEVICE_ADDRESS = 65

def s16(val):
    return struct.unpack('h', struct.pack('H', val))[0]

def u16(val):
    return struct.unpack('H', struct.pack('h', val))[0]

class OnRobotRG2FT:

    def __init__(self, ip, port):
        self.client = OnRobotTcpClient()
        self.client.connect(ip, port, RG2FT_DEVICE_ADDRESS)

    def writeCommand(self, cmd: RG2FTCommand):
        if cmd.TargetWidth < RG2FT_MIN_WIDTH or cmd.TargetWidth > RG2FT_MAX_WIDTH:
           rospy.logerr("[OnRobotRG2FT] Target width out of range")
           return 
           
        if cmd.TargetForce < RG2FT_MIN_FORCE or cmd.TargetForce > RG2FT_MAX_FORCE:
           rospy.logerr("[OnRobotRG2FT] Target force out of range")
           return 

        if cmd.Control not in [0, 1]:
            rospy.logerr("[OnRobotRG2FT] Control is not 0 or 1")
            return 

        message = [0] * 3

        message[0] = cmd.TargetForce
        message[1] = cmd.TargetWidth
        message[2] = cmd.Control

        self.client.write(address=2, message=message)

    def readState(self) -> RG2FTState:
        resp = self.client.read(address=257, count=26)

        msg = RG2FTState()

        msg.StatusL = resp[0] # Reads low (0x0000) when there is no error with the left finger sensor.
        msg.FxL = s16(resp[2]) # Left finger sensor's force value along the X axis (in the sensor coordinate system) in 1/10N.
        msg.FyL = s16(resp[3]) # Left finger sensor's force value along the Y axis (in the sensor coordinate system) in 1/10N.
        msg.FzL = s16(resp[4]) # Left finger sensor's force value along the Z axis (in the sensor coordinate system) in 1/10N.
        msg.TxL = s16(resp[5]) # Left finger sensor's torque value about the X axis (in the sensor coordinate system) in 1/100 Nm.
        msg.TyL = s16(resp[6]) # Left finger sensor's torque value about the Y axis (in the sensor coordinate system) in 1/100 Nm.
        msg.TzL = s16(resp[7]) # Left finger sensor's torque value about the Z axis (in the sensor coordinate system) in 1/100 Nm.
        msg.StatusR = resp[9] # Same as the left above.
        msg.FxR = s16(resp[11]) # Same as the left above.
        msg.FyR = s16(resp[12]) # Same as the left above.
        msg.FzR = s16(resp[13]) # Same as the left above.
        msg.TxR = s16(resp[14]) # Same as the left above.
        msg.TyR = s16(resp[15]) # Same as the left above.
        msg.TzR = s16(resp[16]) # Same as the left above.
        msg.ProximityStatusL = resp[17] # Reads low (0x0000) when there is no error with the left proximity sensor.
        msg.ProximityValueL = s16(resp[18]) # Reads the current distance from the left proximity sensor in 1/10 mm.
        msg.ProximityStatusR = resp[20] # Same as the left above.
        msg.ProximityValueR = s16(resp[21]) # Same as the left above.
        msg.ActualGripperWidth = s16(resp[23]) # Indicates the current width between the gripper fingers in 1/10 millimeters.
        msg.GripperBusy = resp[24] # High (1) when a motion is ongoing, low (0) when not. The gripper will only accept new commands when this flag is low.
        msg.GripDetected = resp[25] # High (1) when an internal or external grip is detected.

        return msg

    def setProximityOffsets(self, left_offset, right_offset):
        message = [0] * 2

        message[0] = left_offset # This field sets the offset of the left proximity sensor that is subtracted from the raw signal. It must be provided in 1/10 millimeters. 
        message[1] = right_offset # Same as the left above.

        print(message)

        self.client.write(address=5, message=message)

    def zeroForceTorque(self, val):
        message = [0] * 1

        message[0] = val # Zero the force and torque values to cancel any offset.

        self.client.write(address=0, message=message)

    def restartPowerCycle(self):
        self.client.restartPowerCycle()
