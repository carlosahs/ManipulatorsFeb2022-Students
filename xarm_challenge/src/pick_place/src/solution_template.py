#!/usr/bin/env python

import rospy
import sys
import tf_conversions
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import numpy as np
from moveit_commander.conversions import pose_to_list
import geometry_msgs.msg
from path_planner.srv import *
from tf.transformations import *
from moveit_msgs.msg import Grasp
from math import pi


def translation_quaternion_matrix(trans_mat, quat_mat):
    return np.dot(trans_mat, quat_mat)

def get_translation_matrix(trans):
    return translation_matrix((trans.x, trans.y, trans.z))

def get_quaternion_matrix(quat_rot):
    return quaternion_matrix((quat_rot.x, quat_rot.y, quat_rot.z, quat_rot.w))

class Planner():
  def __init__(self):
    #TO DO: Initialise move it interface
    # Initialize moveit_commander and a rospy node
    moveit_commander.roscpp_initialize(sys.argv)

    # Instantiate a RobotCommander object
    robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object
    scene = moveit_commander.PlanningSceneInterface()
    
    # Instantiate a MoveGroupCommander object
    xarm_group = moveit_commander.MoveGroupCommander("xarm6")
    xgripper = moveit_commander.MoveGroupCommander("xarm_gripper")

    # Create DisplayTrajectory ROS publisher
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20
    )

    # Display basic information
    planning_frame = xarm_group.get_planning_frame()
    print "====== Planning frame: %s" % planning_frame

    eef_link = xarm_group.get_end_effector_link()
    print "====== End effector link is : %s" % eef_link

    group_names = robot.get_group_names()
    print "====== Available Planning Groups: %s" % group_names

    print "====== Printing robot state"
    print robot.get_current_state()
    print ''

    # Instantiate class variables
    self.robot = robot
    self.scene = scene
    self.xarm_group = xarm_group
    self.xgripper = xgripper
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def wait_for_state_update(self,box_name, box_is_known=False, box_is_attached=False, timeout=0.5):
    #TO DO: Whenever we change something in moveit we need to make sure that the interface has been updated properly
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()

    while (seconds - start < timeout) and not rospy.is_shutdown():
        attached_objects = scene.get_attached_objects([box_name])

        is_attached = len(attached_objects.keys()) > 0
        is_known = box_name in scene.get_known_object_names()

        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        rospy.sleep(0.1)
        seconds = rospy.get_time()

    return False



  def addObstacles(self):
    #TO DO: Add obstables in the world
    """ 
    **************************
    Adding Objects to the Planning Scene
    **************************
    """ 
    scene = self.scene

    #Cargo names
    targets = [
        "RedBox",
        "BlueBox",
        "GreenBox"
    ]

    targets_state = True
  
    # Red box
    rbox_pose = geometry_msgs.msg.PoseStamped()
    rbox_pose.header.frame_id = targets[0]

    rbox_pose.pose.orientation.w = 1.0
    rbox_pose.pose.position.z = 0
    rbox_name = targets[0]

    scene.add_box(rbox_name, rbox_pose, size=(0.06, 0.06, 0.06))

    targets_state = targets_state and self.wait_for_state_update(targets[0], box_is_known=True)

    # Blue box
    bbox_pose = geometry_msgs.msg.PoseStamped()
    bbox_pose.header.frame_id = targets[1]

    bbox_pose.pose.orientation.w = 2.0
    bbox_pose.pose.position.z = 0
    bbox_name = targets[1]

    scene.add_box(bbox_name, bbox_pose, size=(0.06, 0.06, 0.06))

    targets_state = targets_state and self.wait_for_state_update(targets[1], box_is_known=True)

    # Green box
    gbox_pose = geometry_msgs.msg.PoseStamped()
    gbox_pose.header.frame_id = targets[2]

    gbox_pose.pose.orientation.w = 3.0
    gbox_pose.pose.position.z = 0
    gbox_name = targets[2]

    scene.add_box(gbox_name, gbox_pose, size=(0.06, 0.06, 0.06))

    targets_state = targets_state and self.wait_for_state_update(targets[2], box_is_known=True)

    #goal names
    boxes = [
        "DepositBoxGreen",
        "DepositBoxRed",
        "DepositBoxBlue"
    ]

    return targets_state

  def goToPose(self,pose_goal):
    #TO DO: Code used to move to a given position using move it
    xarm_group = self.xarm_group

    xarm_group.set_pose_target(pose_goal)
    xarm_group.go(wait=True)

    xarm_group.stop()
    xarm_group.clear_pose_targets()


  def detachBox(self,box_name):
    #TO DO: Open the gripper and call the service that releases the box
    rospy.wait_fro_service("link_attacher_node/attach")

    xgripper = self.xgripper

    xgripper.set_named_target("open")
    xgripper.go(wait = True)

    xgripper.stop()
    xgripper.clear_pose_targets()

  def attachBox(self,box_name):
    #TO DO: Close the gripper and call the service that releases the box
    xgripper = self.xgripper

    xgripper.set_named_target("close")
    xgripper.go(wait = True)

    xgripper.stop()
    xgripper.clear_pose_targets()

######################################################################################

class myNode():
  def __init__(self):
    #TO DO: Initialise ROS and create the service calls
    rospy.init_node('solution', anonymous=True)

    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

    # Good practice trick, wait until the required services are online before continuing with the aplication
    rospy.wait_for_service('RequestGoal')
    rospy.wait_for_service('AttachObject')

  def move2goal(self, transform):
      trans = translation_from_matrix(transform)
      quat_rot = quaternion_from_matrix(transform)

      goal = geometry_msgs.msg.Pose()

      goal.position.x = trans[0]
      goal.position.y = trans[1]
      goal.position.z = trans[2]

      goal.orientation.x = 1.0 # quat_rot[0]
      goal.orientation.y = 0.0 # quat_rot[1]
      goal.orientation.z = 0.0 # quat_rot[2]
      goal.orientation.w = 0.0 # quat_rot[3]

      self.planner.goToPose(goal)

  def getGoal(self,action):
    #TO DO: Call the service that will provide you with a suitable target for the movement

    pass


  def tf_goal(self, goal):
    #TO DO:Use tf2 to retrieve the position of the target with respect to the proper reference frame
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
      try:
        trans = self.tfBuffer.lookup_transform('link_tcp', goal, rospy.Time())
        return trans
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        continue


  def main(self):
    #TO DO: Main code that contains the aplication
    self.planner = Planner()
    self.planner.addObstacles()

    # Plan a motion for this group to a desired pose for the end-effector
    # Get Box Position
    gbox = geometry_msgs.msg.Pose()

    # gbox.position.x = 0.214284
    # gbox.position.y = 0.187592
    # gbox.position.z = 0.001827

    xarm_pos_inv = self.tf_goal("link_base")
    gbox_tf_pos = self.tf_goal("GreenBox")

    xarm_trans_inv = get_translation_matrix(xarm_pos_inv.transform.translation)
    xarm_rot_inv = get_quaternion_matrix(xarm_pos_inv.transform.rotation)

    xarm_pos = inverse_matrix(translation_quaternion_matrix(xarm_trans_inv, xarm_rot_inv))

    print(xarm_pos)

    gbox_trans = get_translation_matrix(gbox_tf_pos.transform.translation)
    gbox_rot = get_quaternion_matrix(gbox_tf_pos.transform.rotation)

    gbox_pos = translation_quaternion_matrix(gbox_trans, gbox_rot)

    xarm2gbox = np.dot(xarm_pos, gbox_pos)

    print(xarm2gbox)

    self.move2goal(xarm2gbox)

    # gbox.position.x = current_pos.pose.position.x
    # gbox.position.y = current_pos.pose.position.y
    # gbox.position.z = current_pos.pose.position.z
    # 
    # gbox.orientation = current_pos.pose.orientation

    # self.planner.goToPose(gbox)

    # Get green box deposit position
    # deposit_gbox = geometry_msgs.msg.Pose()

    # deposit_gbox.position.x = 0.041
    # deposit_gbox.position.y = -0.464
    # deposit_gbox.position.z = 0.049

    # current_pos = self.planner.xarm_group.get_current_pose("link_eef")

    # deposit_gbox.orientation = current_pos.pose.orientation

    # self.planner.goToPose(deposit_gbox)

    rospy.signal_shutdown("Task Completed")


if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass
