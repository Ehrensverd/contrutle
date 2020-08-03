#!/usr/bin/env python
import sys
import time
import numpy as np

import rospy
import moveit_commander
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint
import moveit_msgs.msg
from geometry_msgs.msg import Point, Vector3
import geometry_msgs.msg
from moveit_commander import MoveItCommanderException

from sensor_msgs.msg import Joy
from math import pi
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class TurtleController(object):

    def __init__(self):
        super(TurtleController, self).__init__()

        rospy.init_node("conturtler", anonymous=True)
        self.moving = False
        self.ik_getting_ready = True
        self.x_pos = 0.05
        self.y_pos = 0
        self.z_pos = 0.35
        #  Subscribers
        self.controller_input = rospy.Subscriber("/joy", Joy, self.joystick_callback)

        # Joy setup
        # rospy.set_param('autorepeat_rate', 0.3)
        self.joystick_output = Joy()
        self.joy_sending = False
        self.buttons = (-0.0,) * 8
        self.axes = self.axes_default_pos = (-0.0, -0.0, -0.0, -0.0, 1.0, 1.0, -0.0, -0.0)
        # self.autorate = rospy.get_param('~autorepeat_rate', 2)

        # Publishers
        self.gripper_pub = rospy.Publisher("/gripper_position", Float64MultiArray, queue_size=1)
        self.turtle_drive_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # gripper
        self.gripper_closed = False
        self.gripper_busy = False

        # Setup moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        group_name = "arm"
        self.arm_group = moveit_commander.MoveGroupCommander(group_name)
        group_name2 = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(group_name2)
        self.rate = rospy.Rate(200)
        self.arm_group.set_planning_time(0.1)
        self.gripper_group.set_planning_time(0.1)

    def gripper_control(self):
        gripper = self.gripper_group
        gripper_message = Float64MultiArray()
        gripper_position = gripper.get_current_joint_values()

        # Button not engaged, in open close mode
        if self.axes[4] == 1.0:
            # do open close
            if self.buttons[5]:
                self.gripper_closed = (self.gripper_closed == False)

                if self.gripper_closed:
                    # open
                    print
                    "open gripper"
                    gripper_position[0] = 0.01
                    gripper_message.data = gripper_position
                    self.gripper_pub.publish(gripper_message)
                else:
                    print
                    "close gripper"
                    gripper_position[0] = -0.01
                    gripper_message.data = gripper_position
                    self.gripper_pub.publish(gripper_message)

                print
                "sleep grip"
                time.sleep(0.5)
        else:
            self.gripper_closed = False
            print
            round(self.axes[4] / 100, 6)
            gripper_position[0] = round(self.axes[4] / 100, 8)
            gripper_message.data = gripper_position
            self.gripper_pub.publish(gripper_message)

        time.sleep(0.1)
        self.gripper_busy = False

    def turtle_drive(self):

        x = self.axes[1]
        turn = self.axes[0]
        if self.buttons[3]:
            self.turtle_drive_pub.publish(Twist(linear=Vector3(x=x * 0.5), angular=Vector3(z=turn * 2)))
        elif self.buttons[1]:
            self.turtle_drive_pub.publish(Twist(linear=Vector3(x=x * 0.04), angular=Vector3(z=turn * 0.3)))
        else:
            self.turtle_drive_pub.publish(Twist(linear=Vector3(x=x * 0.1), angular=Vector3(z=turn * 0.8)))

    def xyz_desired_increase(self):
        if self.axes[3] == -0.0 and self.axes[2] == -0.0 and self.axes[5] == 1.0:
            return False

        if self.axes[3] != -0.0:
            self.x_pos = self.x_pos + (self.axes[3] / 40)

        if self.axes[2] != -0.0:
            self.y_pos = self.y_pos + (self.axes[2] / 40)

        if self.axes[5] != 1.0:
            z_increase = (self.axes[5] - 1) / 40
            if self.buttons[4]:
                z_increase *= -1
            else:
                z_increase *= 1
            self.z_pos = self.z_pos + z_increase

        return [self.x_pos, self.y_pos, self.z_pos]

    def get_current_arm(self):
        return self.arm_group.get_current_pose(
            end_effector_link="end_effector_link"), self.arm_group.get_current_joint_values

    def joystick_callback(self, joystick):
        if not joystick.buttons or not joystick.axes:
            print
            "no input"
            return

        # check if controller is non-default and update changed values
        self.buttons = joystick.buttons
        self.axes = joystick.axes
        self.joy_sending = True

    def move_ik(self, xyz):
        move_group = self.arm_group

        tilt_constraint = OrientationConstraint()
        # The link that must be oriented upwards
        tilt_constraint.link_name = "end_effector_link"
        # 'base_link' is equal to the world link
        tilt_constraint.header.frame_id = "link1"

        tilt_constraint.orientation.w = 1.0
        # Allow rotation of 45 degrees around the x and y axis
        tilt_constraint.absolute_x_axis_tolerance = 0.1  # Allow max rotation of 45 degrees
        tilt_constraint.absolute_y_axis_tolerance = 0.1  # Allow max rotation of 360 degrees
        tilt_constraint.absolute_z_axis_tolerance = 3.14  # Allow max rotation of 45 degrees
        # The tilt constraint is the only constraint
        tilt_constraint.weight = 1.0

        constraint = Constraints()
        constraint.name = "tilt constraint"
        constraint.orientation_constraints = [tilt_constraint]
        print
        "move seting stuff"
        move_group.set_path_constraints(constraint)

        move_group.set_position_target(xyz)
        print
        "move set stuff done"
        try:
            move_group.go(wait=True)
            print
            "move go"
        except MoveItCommanderException:
            print
            "sorry cant move here!"
        # move_group.stop()
        self.moving = False

    def move_joints(self, joints):
        move_group = self.arm_group
        try:
            move_group.go(list(joints), wait=True)
        except MoveItCommanderException:
            print
            "sorry cant move here!"
        move_group.clear_pose_targets()
        move_group.stop()
        self.moving = False

    def manipulator(self):
        # override go to home position
        if self.buttons[0]:
            self.arm_group.stop()
            self.move_home()
            time.sleep(4)
            self.moving = False
            return
        if self.moving:
            return
        self.moving = True
        new_xyz = self.xyz_desired_increase()
        if not new_xyz:
            self.moving = False
            return

        self.move_ik(new_xyz)

    def move_home(self):
        move_group = self.arm_group
        try:
            move_group.go([0, -1, 0.319, 0.71], wait=True)
        except MoveItCommanderException:
            print
            "sorry cant move here!"
        move_group.stop()
        self.x_pos = 0.05
        self.y_pos = 0
        self.z_pos = 0.35
        self.moving = False

    def run(self):
        # give time for nodes to initialise
        time.sleep(3)

        print
        "Turtle home posistion"
        time.sleep(1)
        print
        "Turtle ready!"
        while not rospy.is_shutdown():

            self.turtle_drive()

            if not self.gripper_busy:
                self.gripper_busy = True
                self.gripper_control()

            self.manipulator()

            self.rate.sleep()


if __name__ == "__main__":
    marvin = TurtleController()
    marvin.run()
