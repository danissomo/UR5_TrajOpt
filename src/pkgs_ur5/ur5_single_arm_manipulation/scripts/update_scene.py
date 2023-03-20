#!/usr/bin/env python
import rospy
import moveit_commander
import tf
import rospkg
import os

rospy.init_node('update_scene', anonymous=True)

robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("manipulator")
grp_group = moveit_commander.MoveGroupCommander("gripper")

arm_group.set_named_target('up')
arm_group.go(wait=True)
print("UR5 stand up")

# Open
grp_group.set_joint_value_target([9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05, 9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05])
grp_group.go(wait=True)
print("Gripper open")
