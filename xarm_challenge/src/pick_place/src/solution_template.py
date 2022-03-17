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
# from moveit_msgs.msg import Grasp
from math import pi

OPERATIONAL_HEIGHT = 0.125
BOX_LENGTH = 0.06
GRIPPER_CLOSURE_VALUE = (pi / 11 + pi / 12) / 2

ARM_GROUP = "xarm6"
GRASPING_GROUP = "xarm_gripper"

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

    return np.dot(trans, quat_rot)

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
    xarm_group = moveit_commander.MoveGroupCommander(ARM_GROUP)
    xgripper = moveit_commander.MoveGroupCommander(GRASPING_GROUP)
    # Create DisplayTrajectory ROS publisher
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20
    )

    # Display basic information
    planning_frame = xarm_group.get_planning_frame()
    print("====== Planning frame: %s" % planning_frame)
    eef_link = xarm_group.get_end_effector_link()
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
    self.eef_link = eef_link
    # self.group_names = group_names

  def _close_grip(self):
    xgripper = self.xgripper
    xgripper_joint_values = xgripper.get_current_joint_values()

    close_joint_value = GRIPPER_CLOSURE_VALUE

    for i in range(len(xgripper_joint_values)):
        xgripper_joint_values[i] = close_joint_value

    xgripper.go(xgripper_joint_values, wait=True)
    xgripper.stop()

  def _open_grip(self):
    xgripper = self.xgripper
    xgripper_joint_values = xgripper.get_current_joint_values()

    open_joint_value = 0.0

    for i in range(len(xgripper_joint_values)):
        xgripper_joint_values[i] = open_joint_value

    xgripper.go(xgripper_joint_values, wait=True)
    xgripper.stop()

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
        self._open_grip()

        self.scene.remove_attached_object(self.eef_link, name=box_name)
        self.scene.remove_world_object(box_name)

        attach = rospy.ServiceProxy('AttachObject', AttachObject)
        attach(False, box_name)

        return self.wait_for_state_update(box_name, box_is_known=False, box_is_attached=False)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

        return False

  def attachBox(self,box_name):
    #TO DO: Close the gripper and call the service that releases the box
    try:
        touch_links = self.robot.get_link_names(group=GRASPING_GROUP)
        self.scene.attach_box(self.eef_link, box_name, touch_links=touch_links)

        self._close_grip()

        attach = rospy.ServiceProxy('AttachObject', AttachObject)
        attach(True, box_name)

        return self.wait_for_state_update(box_name, box_is_known=True, box_is_attached=True)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

        return False

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

      xyz_rpy = list(euler_from_quaternion([1.0, 0.0, 0.0, 0.0]))
      xyz_rpy[2] = euler_from_quaternion(quat_rot)[2]

      xyz_quat = quaternion_from_euler(xyz_rpy[0], xyz_rpy[1], xyz_rpy[2])

      # Orientations
      goal.orientation.x = xyz_quat[0]
      goal.orientation.y = xyz_quat[1]
      goal.orientation.z = xyz_quat[2]
      goal.orientation.w = xyz_quat[3]

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

    # Move according to RequestGoal service
    while True:
        pick = self.getGoal("pick")

        if pick.status == True and pick.goal != "End":
            # Get the box
            self._move_to_box(pick.goal)
            place = self.getGoal("place")

            if (place.status == True and place.goal != "End") or place != None:
                # Go to deposit
                self._move_to_deposit(place.goal)
            else:
                break
        else:
            break

    rospy.signal_shutdown("Task Completed")

if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass
