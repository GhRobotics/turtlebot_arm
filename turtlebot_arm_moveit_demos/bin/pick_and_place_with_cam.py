#!/usr/bin/env python

import sys
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import moveit_commander
import actionlib

from math import atan2
from copy import deepcopy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_msgs.msg import Grasp, GripperTranslation
from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import MoveGroupCommander
from moveit_python import PlanningSceneInterface
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
from grasping_msgs.msg import *

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'

GRIPPER_FRAME = 'gripper_link'
GRIPPER_JOINT_NAMES = ['gripper_joint']
GRIPPER_EFFORT = [1.0]
GRIPPER_PARAM = '/gripper_controller'

REFERENCE_FRAME = 'base_link'
ARM_BASE_FRAME = 'arm_base_link'

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('moveit_demo')

        # We need a tf2 listener to convert poses into arm reference base
        try:
            self._tf2_buff = tf2_ros.Buffer()
            self._tf2_list = tf2_ros.TransformListener(self._tf2_buff)
        except rospy.ROSException as err:
            rospy.logerr("MoveItDemo: could not start tf buffer client: " + str(err))
            raise err

        self.gripper_opened = [rospy.get_param(GRIPPER_PARAM + "/max_opening") - 0.01]
        self.gripper_closed = [rospy.get_param(GRIPPER_PARAM + "/min_opening") + 0.01]
        self.gripper_neutral = [rospy.get_param(GRIPPER_PARAM + "/neutral",
                                                (self.gripper_opened[0] + self.gripper_closed[0])/2.0) ]
        
        self.gripper_tighten = rospy.get_param(GRIPPER_PARAM + "/tighten", 0.0) 

        # Use the planning scene object to add or remove objects
        self.scene = PlanningSceneInterface(REFERENCE_FRAME)

        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)

        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('target_pose', PoseStamped, queue_size=10)

        # Create a dictionary to hold object colors
        self.colors = dict()

        # Initialize the move group for the right arm
        arm = MoveGroupCommander(GROUP_NAME_ARM)

        # Initialize the move group for the right gripper
        gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)

        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()

        # Allow some leeway in position (meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.05)
        arm.set_goal_orientation_tolerance(0.1)

        # Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)

        # Set the right arm reference frame
        arm.set_pose_reference_frame(REFERENCE_FRAME)

        # Allow 5 seconds per planning attempt
        arm.set_planning_time(15)

        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 5

        # Set a limit on the number of place attempts
        max_place_attempts = 3

        # Give the scene a chance to catch up
        rospy.sleep(2)

        rospy.loginfo("Connecting to basic_grasping_perception/find_objects...")
        find_objects = actionlib.SimpleActionClient("basic_grasping_perception/find_objects", FindGraspableObjectsAction)
        find_objects.wait_for_server()
        rospy.loginfo("...connected")

        rospy.sleep(1)
 
        arm.set_named_target('right_up')
        if arm.go() != True:
            rospy.logwarn("  Go failed")

        gripper.set_joint_value_target(self.gripper_opened)
        if gripper.go() != True:
            rospy.logwarn("  Go failed")

        rospy.sleep(1)

        while not rospy.is_shutdown():
            # Initialize the grasping goal
            goal = FindGraspableObjectsGoal()

            goal.plan_grasps = False

            find_objects.send_goal(goal)

            find_objects.wait_for_result(rospy.Duration(5.0))

            find_result = find_objects.get_result()

            rospy.loginfo("Found %d objects" %len(find_result.objects))

            for name in self.scene.getKnownCollisionObjects():
                self.scene.removeCollisionObject(name, False)
            for name in self.scene.getKnownAttachedObjects():
                self.scene.removeAttachedObject(name, False)

            self.scene.waitForSync()

            self.scene._colors = dict()

            # Use the nearest object on the table as the target
            target_pose = PoseStamped()
            target_pose.header.frame_id = REFERENCE_FRAME
            target_size = None
            the_object = None
            the_object_dist_xmin = 0.23
            the_object_dist_xmax = 0.30
            the_object_dist_ymin = -0.09
            the_object_dist_ymax = 0.09
            count = -1

            for obj in find_result.objects:
                count +=1
                self.scene.addSolidPrimitive("object %d"%count,
                                        obj.object.primitives[0],
                                        obj.object.primitive_poses[0],
                                        wait = False)

    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, joint_positions):
        # Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()

        # Set the joint names to the gripper joint names
        t.header.stamp = rospy.get_rostime()
        t.joint_names = GRIPPER_JOINT_NAMES

        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()

        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions

        # Set the gripper effort
        tp.effort = GRIPPER_EFFORT

        tp.time_from_start = rospy.Duration(0.0)

        # Append the goal point to the trajectory points
        t.points.append(tp)

        # Return the joint trajectory
        return t

    # Generate a gripper translation in the direction given by vector
    def make_gripper_translation(self, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()

        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]

        # The vector is relative to the gripper frame
        g.direction.header.frame_id = GRIPPER_FRAME

        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired

        return g

    # Generate a list of possible grasps
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects, grasp_opening=[0]):
        # Initialize the grasp object
        g = Grasp()

        # Set the pre-grasp and grasp postures appropriately;
        # grasp_opening should be a bit smaller than target width
        g.pre_grasp_posture = self.make_gripper_posture(self.gripper_opened)
        g.grasp_posture = self.make_gripper_posture(grasp_opening)

        # Set the approach and retreat parameters as desired
        g.pre_grasp_approach = self.make_gripper_translation(0.01, 0.1, [1.0, 0.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, [0.0, -1.0, 1.0])

        # Set the first grasp pose to the input pose
        g.grasp_pose = initial_pose_stamped

        # Pitch angles to try
        pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.4, -0.4, 0.5, -0.5, 0.6, -0.6, 0.7, -0.7, 0.8, -0.8, 0.9, -0.9, 1.0, -1.0, 1.1, -1.1, 1.2, -1.2, 1.3, -1.3, 1.4, -1.4, 1.5, -1.5]

        # Yaw angles to try; given the limited dofs of turtlebot_arm, we must calculate the heading
        # from arm base to the object to pick (first we must transform its pose to arm base frame)
        target_pose_arm_ref = self._tf2_buff.transform(initial_pose_stamped, ARM_BASE_FRAME)
        x = target_pose_arm_ref.pose.position.x
        y = target_pose_arm_ref.pose.position.y
        yaw_vals = [atan2(y, x) + inc for inc in [0, 0.1,-0.1]]

        # A list to hold the grasps
        grasps = []

        # Generate a grasp for each pitch and yaw angle
        for yaw in yaw_vals:
            for pitch in pitch_vals:
                # Create a quaternion from the Euler angles
                q = quaternion_from_euler(0, pitch, yaw)

                # Set the grasp pose orientation accordingly
                g.grasp_pose.pose.orientation.x = q[0]
                g.grasp_pose.pose.orientation.y = q[1]
                g.grasp_pose.pose.orientation.z = q[2]
                g.grasp_pose.pose.orientation.w = q[3]

                # Set and id for this grasp (simply needs to be unique)
                g.id = str(len(grasps))

                # Set the allowed touch objects to the input list
                g.allowed_touch_objects = allowed_touch_objects

                # Don't restrict contact force
                g.max_contact_force = 0

                # Degrade grasp quality for increasing pitch angles
                g.grasp_quality = 1.0 - abs(pitch)

                # Append the grasp to the list
                grasps.append(deepcopy(g))

        # Return the list
        return grasps

    # Generate a list of possible place poses
    def make_places(self, target_id, init_pose):
        # Initialize the place location as a PoseStamped message
        place = PoseStamped()

        # Start with the input place pose
        place = init_pose

        # A list of x shifts (meters) to try
        x_vals = [0, 0.005, -0.005] #, 0.01, -0.01, 0.015, -0.015]

        # A list of y shifts (meters) to try
        y_vals = [0, 0.005, -0.005, 0.01, -0.01] #, 0.015, -0.015]

        # A list of pitch angles to try
        pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.4, -0.4, 0.5, -0.5, 0.6, -0.6, 0.7, -0.7, 0.8, -0.8, 0.9, -0.9, 1.0, -1.0, 1.1, -1.1, 1.2, -1.2, 1.3, -1.3, 1.4, -1.4, 1.5, -1.5] #, 0.005, -0.005, 0.01, -0.01, 0.02, -0.02]

        # A list to hold the places
        places = []

        # Generate a place pose for each angle and translation
        for pitch in pitch_vals:
            for dy in y_vals:
                for dx in x_vals:
                    place.pose.position.x = init_pose.pose.position.x + dx
                    place.pose.position.y = init_pose.pose.position.y + dy
    
                    # Yaw angle: given the limited dofs of turtlebot_arm, we must calculate the heading from
                    # arm base to the place location (first we must transform its pose to arm base frame)
                    target_pose_arm_ref = self._tf2_buff.transform(place, ARM_BASE_FRAME)
                    x = target_pose_arm_ref.pose.position.x
                    y = target_pose_arm_ref.pose.position.y
                    yaw = atan2(y, x)
    
                    # Create a quaternion from the Euler angles
                    q = quaternion_from_euler(0, pitch, yaw)
    
                    # Set the place pose orientation accordingly
                    place.pose.orientation.x = q[0]
                    place.pose.orientation.y = q[1]
                    place.pose.orientation.z = q[2]
                    place.pose.orientation.w = q[3]

                    # MoveGroup::place will transform the provided place pose with the attached body pose, so the object retains
                    # the orientation it had when picked. However, with our 4-dofs arm this is infeasible (nor we care about the
                    # objects orientation!), so we cancel this transformation. It is applied here:
                    # https://github.com/ros-planning/moveit_ros/blob/jade-devel/manipulation/pick_place/src/place.cpp#L64
                    # More details on this issue: https://github.com/ros-planning/moveit_ros/issues/577
                    acobjs = self.scene.get_attached_objects([target_id])
                    aco_pose = self.get_pose(acobjs[target_id])
                    if aco_pose is None:
                        rospy.logerr("Attached collision object '%s' not found" % target_id)
                        return None

                    aco_tf = self.pose_to_mat(aco_pose)
                    place_tf = self.pose_to_mat(place.pose)
                    place.pose = self.mat_to_pose(place_tf, aco_tf)
                    rospy.logdebug("Compensate place pose with the attached object pose [%s]. Results: [%s]" \
                                   % (aco_pose, place.pose))

                    # Append this place pose to the list
                    places.append(deepcopy(place))

        # Return the list
        return places

    # Set the color of an object
    def setColor(self, name, r, g, b, a=0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()

        # Set the id to the name given as an argument
        color.id = name

        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a

        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff
        p.is_diff = True

        # Append the colors from the global color dictionary
        for color in self.colors.values():
            p.object_colors.append(color)

        # Publish the scene diff
        self.scene_pub.publish(p)

    def get_pose(self, co):
        # We get object's pose from the mesh/primitive poses; try first with the meshes
        if isinstance(co, CollisionObject):
            if co.mesh_poses:
                return co.mesh_poses[0]
            elif co.primitive_poses:
                return co.primitive_poses[0]
            else:
                rospy.logerr("Collision object '%s' has no mesh/primitive poses" % co.id)
                return None
        elif isinstance(co, AttachedCollisionObject):
            if co.object.mesh_poses:
                return co.object.mesh_poses[0]
            elif co.object.primitive_poses:
                return co.object.primitive_poses[0]
            else:
                rospy.logerr("Attached collision object '%s' has no mesh/primitive poses" % co.id)
                return None
        else:
            rospy.logerr("Input parameter is not a collision object")
            return None

    def pose_to_mat(self, pose):
        '''Convert a pose message to a 4x4 numpy matrix.

        Args:
            pose (geometry_msgs.msg.Pose): Pose rospy message class.

        Returns:
            mat (numpy.matrix): 4x4 numpy matrix
        '''
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        pos = np.matrix([pose.position.x, pose.position.y, pose.position.z]).T
        mat = np.matrix(quaternion_matrix(quat))
        mat[0:3, 3] = pos
        return mat

    def mat_to_pose(self, mat, transform=None):
        '''Convert a homogeneous matrix to a Pose message, optionally premultiply by a transform.

        Args:
            mat (numpy.ndarray): 4x4 array (or matrix) representing a homogenous transform.
            transform (numpy.ndarray): Optional 4x4 array representing additional transform

        Returns:
            pose (geometry_msgs.msg.Pose): Pose message representing transform.
        '''
        if transform != None:
            mat = np.dot(mat, transform)
        pose = Pose()
        pose.position.x = mat[0,3]
        pose.position.y = mat[1,3]
        pose.position.z = mat[2,3]
        quat = quaternion_from_matrix(mat)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

if __name__ == "__main__":
    MoveItDemo()
