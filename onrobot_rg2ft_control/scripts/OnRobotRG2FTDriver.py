#!/usr/bin/env python3

import rospy
from onrobot_rg2ft_control.OnRobotRG2FT import OnRobotRG2FT
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from onrobot_rg2ft_msgs.srv import SetProximityOffsets, SetProximityOffsetsResponse
from onrobot_rg2ft_msgs.msg import RG2FTCommand, RG2FTState
from geometry_msgs.msg import Wrench

class OnRobotRG2FTDriver:

    def __init__(self, ip, port):
        self.gripper = OnRobotRG2FT(ip, port)

        self.state_pub = rospy.Publisher('state', RG2FTState, queue_size=1)
        self.left_wrench_pub = rospy.Publisher('left_wrench', Wrench, queue_size=1)
        self.right_wrench_pub = rospy.Publisher('right_wrench', Wrench, queue_size=1)

        self.cmd_sub = rospy.Subscriber('command', RG2FTCommand, self.gripper.writeCommand)

        # The restarting service
        self.restart_srv = rospy.Service("restart", Trigger, self.restart_cb)
        self.zero_srv = rospy.Service("zero_force_torque", SetBool, self.zero_force_torque_cb)
        self.set_prox_offset_srv = rospy.Service("set_proximity_offsets", SetProximityOffsets, self.prox_offsets_cb)

    def mainLoop(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            # Getting and publish the Gripper status
            state = self.gripper.readState()
            self.state_pub.publish(state)
            self.publish_wrenches(state)

            rate.sleep()

    def restart_cb(self, req):
        rospy.loginfo("Restarting the power cycle of all grippers connected.")
        self.gripper.restartPowerCycle()
        rospy.sleep(1)
        return TriggerResponse(success=None, message=None)  # TODO: implement

    def zero_force_torque_cb(self, req):
        rospy.loginfo("Zeroing force and torque values to cancel any offset")
        self.gripper.zeroForceTorque(req.data)
        return SetBoolResponse(success=None, message=None)  # TODO: implement

    def prox_offsets_cb(self, req):
        rospy.loginfo("Setting proximity offsets.")
        self.gripper.setProximityOffsets(req.ProximityOffsetL, req.ProximityOffsetR)
        return SetProximityOffsetsResponse(success=None, message=None)  # TODO: implement
    
    def publish_wrenches(self, state):
        left_wrench = Wrench()
        left_wrench.force.x = state.FxL / 10
        left_wrench.force.y = state.FyL / 10
        left_wrench.force.z = state.FzL / 10
        left_wrench.torque.x = state.TxL / 100
        left_wrench.torque.y = state.TyL / 100
        left_wrench.torque.z = state.TzL / 100

        right_wrench = Wrench()
        right_wrench.force.x = state.FxR / 10
        right_wrench.force.y = state.FyR / 10
        right_wrench.force.z = state.FzR / 10
        right_wrench.torque.x = state.TxR / 100
        right_wrench.torque.y = state.TyR / 100
        right_wrench.torque.z = state.TzR / 100

        self.left_wrench_pub.publish(left_wrench)
        self.right_wrench_pub.publish(right_wrench)

if __name__ == '__main__':
    rospy.init_node('onrobot_rg2ft_driver', anonymous=True)

    ip = rospy.get_param('~ip', '192.168.1.1')
    port = rospy.get_param('~port', '502')

    driver = OnRobotRG2FTDriver(ip, port)

    driver.mainLoop()


