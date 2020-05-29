#!/usr/bin/env python

from pysabertooth import Sabertooth
import time
import serial.tools.list_ports as port
import rospy
from geometry_msgs.msg import Twist
from sabertooth_ctrl.msg import TwoFloats

global w_l, w_r


class cmd_vel_to_motors:
    """
    This class has 4 methods:
    1. __init__ :   - creates a subscriber for cmd_vel (published from teleop_twist_joy).
                    - creates a publisher node to send motor commands to sabertooth MC (motor controller).
                    - sets constants values for wheel speed calculations.
    2. cmd_vel_cb:  - callback method for cmd_vel, executed every time a new callback message is received.
                    - calculates wheel velocities using differential drive kinematics 
                    - Ref: https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
                    - publishes motor commands as separate wheel velocities

    3. map_val:     - linear interpolation of a given range of input values between an allowed max and min

    4. main:        - prevents the Python Main thread from exiting.

    I've created a custom message TwoFloats for this purpose instead of creating two publishers
    NOTE the math for forward kinematics is a work in progress
    """

    def __init__(self):
        # create subscriber for cmd_vel
        self.cmd_vel_sub = rospy.Subscriber(
            "cmd_vel", Twist, self.cmd_vel_cb, queue_size=1
        )

        # create publisher for process motor commands
        self.pub_motor_cmd = rospy.Publisher(
            "motor_cmd", TwoFloats, queue_size=1)
        self.cmd_msg = TwoFloats()  # object for custom message

        # constants (physical properties of the robot)
        self.WHEEL_RADIUS = 0.06096  # radius of wheels (meters)
        self.WHEEL_SEPARATION = 0.31  # width of the robot (meters)

    def cmd_vel_cb(self, vel):
        # read linear and angular velocities from joy_node
        self.lin_speed = vel.linear.x
        self.ang_speed = vel.angular.z

        # convert linear and angular inputs to left and right wheel velocities
        w_l = ((2 * self.lin_speed) - (self.ang_speed * self.WHEEL_SEPARATION)) / (
            2.0 * self.WHEEL_RADIUS
        )

        w_r = ((2 * self.lin_speed) + (self.ang_speed * self.WHEEL_SEPARATION)) / (
            2.0 * self.WHEEL_RADIUS
        )

        # map values obtained above between [-70 , 70] out of [-127,127]
        w_l = self.map_val(w_l, -15, 15, -70, 70)
        w_r = self.map_val(w_r, -15, 15, -70, 70)

        # add values to cmd_msg and publish
        self.cmd_msg.left = w_l
        self.cmd_msg.right = w_r
        self.pub_motor_cmd.publish(self.cmd_msg)

    def main(self):
        rospy.spin()

    def map_val(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class sabertooth_control:
    """

    """

    def __init__(self, saber_baudrate, saber_addr, saber_timeout):
        self.BAUDRATE = saber_baudrate
        self.SABERTOOTH_ADDRESS = saber_addr
        self.SABERTOOTH_TIMEOUT = saber_timeout
        self.motor_cmd_sub = rospy.Subscriber(
            "motor_cmd", TwoFloats, self.motor_cmd_cb, queue_size=1
        )
        self.vel = TwoFloats()

    def initialize_sabertooth(self):
        rospy.loginfo("\nInit sabertooth....\n")
        rospy.loginfo("\nDetecting sabertooth....\n")
        portlist = list(port.comports())

        # print portlist

        serial_port = ""

        for p in portlist:
            print p
            if "Sabertooth" in str(p):
                serial_port = str(p).split(" ")

        # print "\nAddress found @"

        # print address[0]

        self.saber = Sabertooth(
            serial_port[0],
            self.BAUDRATE,
            self.SABERTOOTH_ADDRESS,
            self.SABERTOOTH_TIMEOUT,
        )

        rospy.loginfo(
            "Sabertooth %s connected at : %s", str(
                serial_port[2]), str(serial_port[0])
        )

    def motor_cmd_cb(self, data):
        self.v_l = data.left
        self.v_r = data.right
        self.vl = w_l
        self.vr = w_r
        rospy.loginfo("received commands: vl: %d, vr:%d", self.v_l, self.v_r)
        self.saber.drive(1, self.v_l)
        self.saber.drive(2, self.v_r)

    def main(self):
        rospy.spin()


# def drive_sabertooth(self):
#     pass
# rospy.init_node('motor_cmd_listener')
# subscribe to
# # drive(number, speed)
# # number: 1-2
# # speed: -100 - 100
# saber.drive(1, -50)

# saber.drive(2, -50)
# time.sleep(1)
# saber.stop()


if __name__ == "__main__":
    rospy.init_node("cmd_vel_to_motors_convertor")
    cmdvel = cmd_vel_to_motors()
    cmdvel.main()

    rospy.init_node("sabertooth_driver")
    sc = sabertooth_control(saber_baudrate=9600,
                            saber_addr=128, saber_timeout=0.1)

    sc.initialize_sabertooth()
    time.sleep(2)
    sc.main()
