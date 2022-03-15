#!/usr/bin/env python3
# Python 2/3 compatibility imports
import rospy
import sys
import copy
import tf_conversions
import tf2_ros
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import geometry_msgs.msg
from path_planner.srv import *
from tf.transformations import *
from moveit_msgs.msg import Grasp
import math

# Ignore warnings
import warnings
warnings.filterwarnings("ignore")

class Planner():
  def __init__(self):
    #TO DO: Initialise move it interface
    # Initialize moveit_commander and a rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    # Instantiate a RobotCommander object
    robot = moveit_commander.RobotCommander()
    # Instantiate a PlanningSceneInterface object
    scene = moveit_commander.PlanningSceneInterface()
    # Instantiate a MoveGroupCommander objects
    xarm_group = moveit_commander.MoveGroupCommander("xarm6")
    xgripper = moveit_commander.MoveGroupCommander("xarm_gripper")
    # Create DisplayTrajectory ROS publisher
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20
    )
    # Instantiate class variables
    self.robot = robot
    self.scene = scene
    self.xarm_group = xarm_group
    self.xgripper = xgripper
    self.display_trajectory_publisher = display_trajectory_publisher


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
    #Cargo names
    targets = ["RedBox",
               "BlueBox",
               "GreenBox"]
    targets_state = True
    # Red box
    rbox_pose = geometry_msgs.msg.PoseStamped()
    rbox_pose.header.frame_id = targets[0]
    rbox_pose.pose.orientation.w = 1.0
    rbox_pose.pose.position.z = 0
    rbox_name = targets[0]
    self.scene.add_box(rbox_name, rbox_pose, size=(0.06, 0.06, 0.06))
    targets_state = targets_state and self.wait_for_state_update(targets[0], box_is_known=True)
    # Blue box
    bbox_pose = geometry_msgs.msg.PoseStamped()
    bbox_pose.header.frame_id = targets[1]
    bbox_pose.pose.orientation.w = 2.0
    bbox_pose.pose.position.z = 0
    bbox_name = targets[1]
    self.scene.add_box(bbox_name, bbox_pose, size=(0.06, 0.06, 0.06))
    targets_state = targets_state and self.wait_for_state_update(targets[1], box_is_known=True)
    # Green box
    gbox_pose = geometry_msgs.msg.PoseStamped()
    gbox_pose.header.frame_id = targets[2]
    gbox_pose.pose.orientation.w = 3.0
    gbox_pose.pose.position.z = 0
    gbox_name = targets[2]
    self.scene.add_box(gbox_name, gbox_pose, size=(0.06, 0.06, 0.06))
    targets_state = targets_state and self.wait_for_state_update(targets[2], box_is_known=True)
    
    #goal names
    boxes = ["DepositBoxGreen",
              "DepositBoxRed",
              "DepositBoxBlue"]
    
    
    return targets_state

  def goToPose(self, pose_goal): 
    #TO DO: Code used to move to a given position using move it
    self.xarm_group.set_pose_target(pose_goal)
    self.xarm_group.go()
    self.xarm_group.stop()
    self.xarm_group.clear_pose_targets()

  def dontDie(self):
    joint_goal = self.xarm_group.get_current_joint_values()
    joint_goal[0] = math.radians(-0)
    joint_goal[1] = math.radians(-18)
    joint_goal[2] = math.radians(-73)
    joint_goal[3] = math.radians(0)
    joint_goal[4] = math.radians(91)
    joint_goal[5] = math.radians(-0)
    self.xarm_group.go(joint_goal, wait = True)
    self.xarm_group.stop()
    self.xarm_group.clear_pose_targets()

  def detachBox(self, box_name):
    #TO DO: Open the gripper and call the service that releases the box
    """
    try:
      request_goal = rospy.ServiceProxy('RequestGoal', RequestGoal)
      resp = request_goal(box_name)
      return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    """
    self.xgripper.set_named_target("open")
    self.xgripper.go(wait = True)

  def attachBox(self, box_name):
    #TO DO: Close the gripper and call the service that releases the box
    pass
    """
    try:
        attachObject = rospy.ServiceProxy('AttachObject', AttachObject)
        resp = attachObject(box_name)
        self.xgripper.set_named_target("close")
        self.xgripper.go(wait = True)
        self.xgripper.stop()cl
        self.xgripper.clear_pose_targets()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    """

######################################################################################

class myNode():
  def __init__(self):
    #TO DO: Initialise ROS and create the service calls
    rospy.init_node('solution', anonymous=True)
    # Good practice trick, wait until the required services are online before continuing with the aplication
    rospy.wait_for_service('RequestGoal')
    rospy.wait_for_service('AttachObject')
    # Instantiate class variables
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    
  def getGoal(self, action):
    #TO DO: Call the service that will provide you with a suitable target for the movement
    """
    try:
       = rospy.ServiceProxy('', )
      resp = 
      return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    """
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

  def move2goal(self, position):
    obj = geometry_msgs.msg.Pose()
    obj.position.x = position[0]
    obj.position.y = position[1]
    obj.position.z = position[2]
    obj.orientation.x = position[3]
    obj.orientation.y = position[4]
    obj.orientation.z = position[5]
    obj.orientation.w = position[6]  
    return obj

  def main(self):
    #TO DO: Main code that contains the aplication
    self.planner = Planner()
    self.planner.addObstacles()
    # print(self.planner.addObstacles())

    ################################### GREEN BOX ###################################
    # Get the Green Box
    current_pos = self.planner.xarm_group.get_current_pose("link_tcp")   
    gpos = self.tf_goal("GreenBox")
    gbox = [current_pos.pose.position.x + gpos.transform.translation.x,
            current_pos.pose.position.y - gpos.transform.translation.y,
            current_pos.pose.position.z - gpos.transform.translation.z,
            current_pos.pose.orientation.x,
            current_pos.pose.orientation.y,
            current_pos.pose.orientation.z,
            current_pos.pose.orientation.w]
    fgbox = self.move2goal(gbox)
    self.planner.goToPose(fgbox)
    # self.planner.attachBox("GreenBox")

    # self.planner.dontDie()

    # Go to the Green Deposit
    gdpos = self.tf_goal("DepositBoxGreen")
    gdep = [current_pos.pose.position.x + gdpos.transform.translation.x,
            current_pos.pose.position.y - gdpos.transform.translation.y,
            current_pos.pose.position.z,
            current_pos.pose.orientation.x,
            current_pos.pose.orientation.y,
            current_pos.pose.orientation.z,
            current_pos.pose.orientation.w]
    # fgdep = self.move2goal(gdep)
    # self.planner.goToPose(fgdep)
    # self.planner.detachBox("GreenBox")
    
    ##################################### RED BOX #####################################
    # Get the Red Box
    rpos = self.tf_goal("RedBox")
    rbox = [current_pos.pose.position.x + rpos.transform.translation.x,
            current_pos.pose.position.y - rpos.transform.translation.y,
            current_pos.pose.position.z - rpos.transform.translation.z,
            current_pos.pose.orientation.x,
            current_pos.pose.orientation.y,
            current_pos.pose.orientation.z,
            current_pos.pose.orientation.w]
    frbox = self.move2goal(rbox)
    self.planner.goToPose(frbox)    

    # self.planner.dontDie()
    
    #  Go to the Red Deposit
    rdpos = self.tf_goal("DepositBoxRed")
    rdep = [rdpos.transform.translation.x,
            rdpos.transform.translation.y,
            rdpos.transform.translation.z,
            rdpos.transform.rotation.x,
            rdpos.transform.rotation.y,
            rdpos.transform.rotation.z,
            rdpos.transform.rotation.w]
    # frdep = self.move2goal(rdep)
    # self.planner.goToPose(frdep)

    #################################### BLUE BOX ####################################
    # Get the Blue Box
    bpos = self.tf_goal("BlueBox")
    bbox = [current_pos.pose.position.x + bpos.transform.translation.x,
            current_pos.pose.position.y - bpos.transform.translation.y,
            current_pos.pose.position.z - bpos.transform.translation.z,
            current_pos.pose.orientation.x,
            current_pos.pose.orientation.y,
            current_pos.pose.orientation.z,
            current_pos.pose.orientation.w]
    fbbox = self.move2goal(bbox)
    self.planner.goToPose(fbbox)

    # self.planner.dontDie()

    #  Go to the Blue Deposit
    bdpos = self.tf_goal("DepositBoxBlue")
    bdep = [bdpos.transform.translation.x,
            bdpos.transform.translation.y,
            bdpos.transform.translation.z,
            bdpos.transform.rotation.x,
            bdpos.transform.rotation.y,
            bdpos.transform.rotation.z,
            bdpos.transform.rotation.w]
    # fbdep = self.move2goal(bdep)
    # self.planner.goToPose(fbdep)

    rospy.signal_shutdown("Task Completed")


if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass
