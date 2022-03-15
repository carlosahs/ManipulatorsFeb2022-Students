#!/usr/bin/env python

import rospy
import sys
import tf_conversions
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import numpy as np
import geometry_msgs.msg
from path_planner.srv import *
from tf.transformations import *
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Grasp
from math import pi

OPERATIONAL_HEIGHT = 0.2

BOX_LENGTH = 0.06

BOXES = [
    "GreenBox",
    "RedBox",
    "BlueBox"
]

DEPOSITS = [
    "DepositBoxGreen",
    "DepositBoxRed",
    "DepositBoxBlue"
]

def translation_quaternion_matrix(trans_mat, quat_mat):
    return np.dot(trans_mat, quat_mat)

def get_target_position(target):
    trans = translation_matrix((
        target.transform.translation.x,
        target.transform.translation.y,
        target.transform.translation.z
    ))

    quat_rot = quaternion_matrix((
        target.transform.rotation.x,
        target.transform.rotation.y,
        target.transform.rotation.z,
        target.transform.rotation.w
    ))

    return translation_quaternion_matrix(trans, quat_rot)

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
    print("====== Planning frame: %s" % planning_frame)
    planning_frame = xgripper.get_planning_frame()
    print("====== Planning frame: %s" % planning_frame)
    eef_link = xarm_group.get_end_effector_link()
    print("====== End effector link is : %s" % eef_link)
    eef_link = xgripper.get_end_effector_link()
    print("====== End effector link is : %s" % eef_link)
    group_names = robot.get_group_names()
    print("====== Available Planning Groups: %s" % group_names)
    print("====== Printing robot state")
    print(robot.get_current_state())
    print('')
   
    # Instantiate class variables
    self.robot = robot
    self.scene = scene
    self.xarm_group = xarm_group
    self.xgripper = xgripper
    self.display_trajectory_publisher = display_trajectory_publisher
    # self.planning_frame = planning_frame
    # self.eef_link = eef_link
    # self.group_names = group_names

  def _pick(self, box):
    frame_id = self.xarm_group.get_planning_frame()
    grasp = Grasp()

    # Grasp pose setup
    grasp.grasp_pose.header.frame_id = frame_id

    quat_rot = quaternion_from_euler(-pi / 2, -pi / 4, -pi / 2)
    quat_msg = geometry_msgs.msg.Quaternion(
        quat_rot[0],
        quat_rot[1],
        quat_rot[2],
        quat_rot[3]
    )

    grasp.grasp_pose.pose.orientation = quat_msg

    grasp.grasp_pose.pose.position.x = 0.415
    grasp.grasp_pose.pose.position.y = 0.0
    grasp.grasp_pose.pose.position.z = 0.5

    # Pre-grasp approach
    grasp.pre_grasp_approach.direction.header.frame_id = frame_id

    grasp.pre_grasp_approach.direction.vector.x = 1.0
    grasp.pre_grasp_approach.min_distance = 0.1
    grasp.pre_grasp_approach.desired_distance = 0.115

    # Post-grasp retreat
    grasp.post_grasp_retreat.direction.header.frame_id = frame_id

    grasp.post_grasp_retreat.direction.vector.z = 1.0
    grasp.post_grasp_retreat.min_distance = 0.1
    grasp.post_grasp_retreat.desired_distance = 0.25

    self._open_gripper(grasp.pre_grasp_posture)
    self._close_gripper(grasp.grasp_posture)

    self.xarm_group.pick(box, [grasp])

  def _open_gripper(self, posture):
    offset = 0.01

    # Add both finger joints of xarm6 robot
    posture.joint_names.append("left_finger_joint")
    posture.joint_names.append("right_finger_joint")

    # Set fingers open
    posture.points.append(JointTrajectoryPoint())

    posture.points[0].positions.append(BOX_LENGTH + offset)
    posture.points[0].positions.append(BOX_LENGTH + offset)

    posture.points[0].time_from_start = rospy.rostime.Duration(0.5)

  def _close_gripper(self, posture):
    # Add both finger joints of xarm6 robot
    posture.joint_names.append("left_finger_joint")
    posture.joint_names.append("right_finger_joint")

    # Set fingers open
    posture.points.append(JointTrajectoryPoint())

    posture.points[0].positions.append(0.0)
    posture.points[0].positions.append(0.0)

    posture.points[0].time_from_start = rospy.rostime.Duration(0.5)

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

    targets_state = True
    # Red box
    rbox_pose = geometry_msgs.msg.PoseStamped()
    rbox_pose.header.frame_id = BOXES[0]
    rbox_pose.pose.orientation.w = 1.0
    rbox_pose.pose.position.z = 0
    rbox_name = BOXES[0]
    scene.add_box(rbox_name, rbox_pose, size=(BOX_LENGTH, BOX_LENGTH, BOX_LENGTH))
    targets_state = targets_state and self.wait_for_state_update(BOXES[0], box_is_known=True)
    # Blue box
    bbox_pose = geometry_msgs.msg.PoseStamped()
    bbox_pose.header.frame_id = BOXES[1]
    bbox_pose.pose.orientation.w = 2.0
    bbox_pose.pose.position.z = 0
    bbox_name = BOXES[1]
    scene.add_box(bbox_name, bbox_pose, size=(BOX_LENGTH, BOX_LENGTH, BOX_LENGTH))
    targets_state = targets_state and self.wait_for_state_update(BOXES[1], box_is_known=True)
    # Green box
    gbox_pose = geometry_msgs.msg.PoseStamped()
    gbox_pose.header.frame_id = BOXES[2]
    gbox_pose.pose.orientation.w = 3.0
    gbox_pose.pose.position.z = 0
    gbox_name = BOXES[2]
    scene.add_box(gbox_name, gbox_pose, size=(BOX_LENGTH, BOX_LENGTH, BOX_LENGTH))
    targets_state = targets_state and self.wait_for_state_update(BOXES[2], box_is_known=True)

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
    try:
        attach = rospy.ServiceProxy('AttachObject', AttachObject)
        attach(0, box_name)
        # self.xgripper.set_named_target("open")
        # self.xgripper.go(wait = True)
        # self.xgripper.stop()
        # self.xgripper4clear_pose_targets()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

  def attachBox(self,box_name):
    #TO DO: Close the gripper and call the service that releases the box
    try:
        attach = rospy.ServiceProxy('AttachObject', AttachObject)
        attach(1, box_name)

        self._pick(box_name)
        # self.xgripper.pick(box_name, grasp)
        # self.xgripper.set_named_target("close")
        # self.xgripper.go(wait = True)
        # self.xgripper.stop()
        # self.xgripper.clear_pose_targets()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

######################################################################################

class myNode():
  def __init__(self):
    #TO DO: Initialise ROS and create the service calls
    rospy.init_node('solution', anonymous=True)
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

    self.box_name = ''
    self.box_is_picked = False
    # Good practice trick, wait until the required services are online before continuing with the aplication
    rospy.wait_for_service('RequestGoal')
    rospy.wait_for_service('AttachObject')

  def _move2goal(self, reference, goal):
      transform = np.dot(reference, goal)

      trans = translation_from_matrix(transform)
      quat_rot = quaternion_from_matrix(transform)

      self._go_to_pose(trans, quat_rot)

  def _go_to_pose(self, trans, quat_rot):
      goal = geometry_msgs.msg.Pose()

      # Positions
      goal.position.x = trans[0]
      goal.position.y = trans[1]
      goal.position.z = trans[2]

      # Orientations
      goal.orientation.x = 1.0
      goal.orientation.y = 0.0
      goal.orientation.z = 0.0
      goal.orientation.w = 0.0

      self.planner.goToPose(goal)

  def _move_to_box(self, box):
      xarm_pose = self._get_xarm_pose()
      box_pose = self._get_goal_pose(box)

      base2box_pose = np.dot(xarm_pose, box_pose)
      base2box_pose_up = translation_matrix((0, 0, OPERATIONAL_HEIGHT))

      # Move above box
      self._move2goal(base2box_pose, base2box_pose_up)

      xarm_pose = self._get_xarm_pose()
      box_pose = self._get_goal_pose(box)

      # Move down to box
      self._move2goal(xarm_pose, box_pose)

      xarm_pose = self._get_xarm_pose()
      base2box_up_pose = np.dot(base2box_pose, base2box_pose_up)

      # Pick box
      self.planner.attachBox(box)

      self.box_name = box
      self.box_is_picked = True

      # Move up with box
      self._move2goal(xarm_pose, np.dot(inverse_matrix(xarm_pose), base2box_up_pose))

  def _move_to_deposit(self, deposit):
      xarm_pose = self._get_xarm_pose()
      deposit_pose = self._get_goal_pose(deposit)

      base2deposit_pose = np.dot(xarm_pose, deposit_pose)
      base2deposit_pose_up = translation_matrix((0, 0, OPERATIONAL_HEIGHT))

      # Move above deposit
      self._move2goal(base2deposit_pose, base2deposit_pose_up)

      # Place cargo
      self.planner.detachBox(self.box_name)

      self.box_name = ''
      self.box_is_picked = False

  def _get_xarm_pose(self):
      return inverse_matrix(get_target_position(self.tf_goal("link_base")))

  def _get_goal_pose(self, goal):
      return get_target_position(self.tf_goal(goal))

  def getGoal(self,action):
    #TO DO: Call the service that will provide you with a suitable target for the movement
    try:
        getObj = rospy.ServiceProxy('RequestGoal', RequestGoal)
        return getObj(action)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

  def tf_goal(self, goal):
    #TO DO:Use tf2 to retrieve the position of the target with respect to the proper reference frame
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
      try:
        trans = self.tfBuffer.lookup_transform('link_tcp', goal, rospy.Time())
        # Does it always return?
        return trans
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        continue

  def main(self):
    #TO DO: Main code that contains the aplication
    self.planner = Planner()
    self.planner.addObstacles()

    # Move to green box
    self._move_to_box(BOXES[0])
    # self._move_to_deposit(DEPOSITS[0])

    rospy.signal_shutdown("Task Completed")

if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass
