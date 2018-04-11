#!/usr/bin/env python

import rospy
import sys

import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

import math
import actionlib
from moveit_python import PlanningSceneInterface
from grasping_msgs.msg import *

GROUP_NAME_ARM = "arm"
GROUP_NAME_GRIPPER = "gripper"
REFERENCE_FRAME = "base_link"

class MoveItDemo:
    def __init__(self):
        rospy.init_node('find_object',anonymous=False)
        self.target_pose_pub=rospy.Publisher('target_pose',PoseStamped,queue_size=5)
        scene = PlanningSceneInterface("base_link")
        # Connect to the simple_grasping find_objects action server
        rospy.loginfo("Connecting to basic_grasping_perception/find_objects...")
        find_objects = actionlib.SimpleActionClient("basic_grasping_perception/find_objects", FindGraspableObjectsAction)
        find_objects.wait_for_server()
        rospy.loginfo("...connected")

        rospy.sleep(1)

        while not rospy.is_shutdown():
            # Initialize the grasping goal
            goal = FindGraspableObjectsGoal()

            goal.plan_grasps = False

            find_objects.send_goal(goal)

            find_objects.wait_for_result(rospy.Duration(5.0))

            find_result = find_objects.get_result()

            rospy.loginfo("Found %d objects" %len(find_result.objects))

            for name in scene.getKnownCollisionObjects():
                scene.removeCollisionObject(name, False)
            for name in scene.getKnownAttachedObjects():
                scene.removeAttachedObject(name, False)

            scene.waitForSync()

            scene._colors = dict()

            # use the nearest object on the table as the target
            target_pose = PoseStamped()
            target_pose.header.frame_id = REFERENCE_FRAME
            count = -1

            for obj in find_result.objects:
                count +=1
                scene.addSolidPrimitive("object %d"%count,
                                        obj.object.primitives[0],
                                        obj.object.primitive_poses[0],
                                        wait = False)

                
                target_size = obj.object.primitives[0].dimensions

                target_pose.pose = obj.object.primitive_poses[0]

                target_pose.pose.orientation.x = 0.0
                target_pose.pose.orientation.y = 0.0
                target_pose.pose.orientation.z = 0.0
                target_pose.pose.orientation.w = 0.0

                rospy.loginfo("Target")
                self.target_pose_pub.publish(target_pose)
                rospy.sleep(1)


if __name__ == "__main__":
    MoveItDemo()
