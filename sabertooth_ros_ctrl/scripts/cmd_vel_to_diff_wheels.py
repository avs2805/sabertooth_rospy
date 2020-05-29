#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sabertooth_ctrl.msg import TwoFloats


class cmd_vel_to_motors:
    """
    This class has 4 methods:
    1. __init__ :   - creates a subscriber for cmd_vel (published from teleop_twist_joy).
                    - creates a publisher node to send motor commands to sabertooth MC (motor controller).
                    - sets constants values for wheel speed calculations.
    2. cmd_vel_cb:  - callback method for cmd_vel, executed every time a new callback message is received.
                    - calculates wheel velocities using differential drive kinematics Ref: https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
                    - publishes motor commands as separate wheel velocities

    3. map_val:     - linear interpolation of a given range of input values between an allowed max and min

    4. main:        - prevents the Python Main thread from exiting.

    I've created a custom message TwoFloats for this purpose instead of creating two publishers
    NOTE the speed to the sabertooth was tuned using the config params in teleop_twist_joy
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
        self.w_l = ((2 * self.lin_speed) - (self.ang_speed * self.WHEEL_SEPARATION)) / (
            2.0 * self.WHEEL_RADIUS
        )

        self.w_r = ((2 * self.lin_speed) + (self.ang_speed * self.WHEEL_SEPARATION)) / (
            2.0 * self.WHEEL_RADIUS
        )
        rospy.loginfo("calc commands: w_l: %d, w_r:%d", self.w_l, self.w_r)

        # map values obtained above between [-70 , 70] out of [-127,127]
        self.w_l = self.map_val(self.w_l, -3, 3, -20, 20)
        self.w_r = self.map_val(self.w_r, -3, 3, -20, 20)
        rospy.loginfo("mapped commands: w_l: %d, w_r:%d", self.w_l, self.w_r)

        # add values to cmd_msg and publish
        self.cmd_msg.left = self.w_l
        self.cmd_msg.right = self.w_r
        self.pub_motor_cmd.publish(self.cmd_msg)

    def main(self):
        rospy.spin()

    def map_val(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


if __name__ == "__main__":
    rospy.init_node("cmd_vel_to_motors_convertor")
    cmdvel = cmd_vel_to_motors()
    cmdvel.main()
