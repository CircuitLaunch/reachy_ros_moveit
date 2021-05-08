#!/usr/bin/env python

import sys
import copy
import rospy
import time
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
#from moveit_commander.conversations import pose_to_list
from reachy_ros_moveit.msg import move_to_goal

class MoveToGoal:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_to_goal")

        self.cmd = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.rightArmGroupName = "right_arm"
        self.leftArmGroupName = "left_arm"

        self.rightArmCmd = moveit_commander.MoveGroupCommander(self.rightArmGroupName)
        self.leftArmCmd = moveit_commander.MoveGroupCommander(self.leftArmGroupName)

        self.rvisPublisher = rospy.Publisher(
            "/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size = 20
        )

        self.rightPlanningFrame = self.rightArmCmd.get_planning_frame()
        self.leftPlanningFrame = self.leftArmCmd.get_planning_frame()

        self.rightEndEffector = self.rightArmCmd.get_end_effector_link()
        self.leftEndEffector = self.leftArmCmd.get_end_effector_link()

        self.groupNames = self.cmd.get_group_names()

        rightArmCurrentPose = self.rightArmCmd.get_current_pose()
        leftArmCurrentPose = self.leftArmCmd.get_current_pose()

        #rospy.loginfo("\nCurrent right arm pose:\n%s\n" % rightArmCurrentPose)
        #rospy.loginfo("\nCurrent left arm pose:\n%s\n" % leftArmCurrentPose)

        r_px = rightArmCurrentPose.pose.position.x
        r_py = rightArmCurrentPose.pose.position.y
        r_pz = rightArmCurrentPose.pose.position.z

        r_ox, r_oy, r_oz, r_oa = self._quaternionToOrientation(
            rightArmCurrentPose.pose.orientation.x,
            rightArmCurrentPose.pose.orientation.y,
            rightArmCurrentPose.pose.orientation.z,
            rightArmCurrentPose.pose.orientation.w
        )

        l_px = leftArmCurrentPose.pose.position.x
        l_py = leftArmCurrentPose.pose.position.y
        l_pz = leftArmCurrentPose.pose.position.z

        l_ox, l_oy, l_oz, l_oa = self._quaternionToOrientation(
            leftArmCurrentPose.pose.orientation.x,
            leftArmCurrentPose.pose.orientation.y,
            leftArmCurrentPose.pose.orientation.z,
            leftArmCurrentPose.pose.orientation.w
        )

        rospy.loginfo("\nCurrent right arm pose: pos(%f, %f, %f), ori(%f, %f, %f, %f)\n" % (r_px, r_py, r_pz, r_ox, r_oy, r_oz, r_oa))
        rospy.loginfo("\nCurrent left arm pose: pos(%f, %f, %f), ori(%f, %f, %f, %f)\n" % (l_px, l_py, l_pz, l_ox, l_oy, l_oz, l_oa))

        rospy.Subscriber("move_to_goal_topic", move_to_goal, self._handleCallback)

        rospy.spin()

    def _quaternionToOrientation(self, qx: float, qy: float, qz: float, qw: float):
        if qw == 1.0:
            x = 1
            y = 0
            z = 0
            a = 0
        else:
            half_a = math.acos(qw)
            factor = 1.0 / math.sin(half_a)
            x = qx * factor
            y = qy * factor
            z = qz * factor
            a = math.degrees(half_a * 2.0)
        return x, y, z, a

    def _orientationToQuaternion(self, ox: float, oy: float, oz: float, oa: float):
        qx = ox
        qy = oy
        qz = oz
        qw = oa
        mag = math.sqrt(qx * qx + qy * qy + qz * qz)
        half_a = math.radians(qw) * 0.5
        factor = math.sin(half_a) / mag
        qx *= factor
        qy *= factor
        qz *= factor
        qw = math.cos(half_a)
        return qx, qy, qz, qw

    def _handleCallback(self, msg):
        side = msg.side
        x = msg.pos_x
        y = msg.pos_y
        z = msg.pos_z
        qx, qy, qz, qw = self._orientationToQuaternion(msg.ori_x, msg.ori_y, msg.ori_z, msg.ori_w)
        rospy.loginfo("\nRecieved command to move %s arm to new pose\n%s\n" % (side, msg))
        self._moveToGoal(side, x, y, z, qx, qy, qz, qw)

    def _moveToGoal(self, side: String, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float):
        poseGoal = geometry_msgs.msg.PoseStamped()
        frac, secs = math.modf(time.time())
        poseGoal.header.stamp.secs = secs
        poseGoal.header.stamp.nsecs = frac * pow(10, 9)
        poseGoal.pose.position.x = x
        poseGoal.pose.position.y = y
        poseGoal.pose.position.z = z
        poseGoal.pose.orientation.x = qx
        poseGoal.pose.orientation.y = qy
        poseGoal.pose.orientation.z = qz
        poseGoal.pose.orientation.w = qw

        if side == 'right':
            poseGoal.header.frame_id = self.rightPlanningFrame
            moveGroup = self.rightArmCmd
        else:
            poseGoal.header.frame_id = self.leftPlanningFrame
            moveGroup = self.leftArmCmd

        moveGroup.set_pose_target(poseGoal)

        plan = moveGroup.go(wait = True)

        moveGroup.stop()

        moveGroup.clear_pose_targets()

    def _addBoxToScene(self, name: String, frameId: String, x: float = 0, y: float = 0, z: float = 0, w: float = 0.05, h: float = 0.05, d: float = 0.05, qx: float = 0, qy: float = 0, qz: float = 0, qw: float = 1):
        objectPose = geometry_msgs.msg.PoseStamped()
        objectPose.header.frame_id = frameId
        objectPose.pose.orientation.x = qx
        objectPose.pose.orientation.y = qy
        objectPose.pose.orientation.z = qz
        objectPose.pose.orientation.w = qw
        objectPose.pose.position.x = x
        objectPose.pose.position.y = y
        objectPose.pose.position.z = z
        self.scene.add_box(name, objectPose, size=(w, h, d))

if __name__ == "__main__":
    MoveToGoal()
