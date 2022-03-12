#!/usr/bin/env python

import rospy
import sys
import tf_conversions
import tf2_ros
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import geometry_msgs.msg
from path_planner.srv import *
from tf.transformations import *
from moveit_msgs.msg import Grasp
from math import pi

class Planner():
  def __init__(self):
    #TO DO: Initialise move it interface
    # Initialize moveit_commander and a rospy node
    moveit_commander.roscpp_initialize(sys.argv)

    # Instantiate a RobotCommander object
    self.robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object
    self.scene = moveit_commander.PlanningSceneInterface()
    
    # Instantiate a MoveGroupCommander object
    group_name = "xarm6"
    self.move_group = moveit_commander.MoveGroupCommander(group_name)

    # Create DisplayTrajectory ROS publisher
    self.display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_trajectory_publisher",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20
    )

  def wait_for_state_update(self,box_name, box_is_known=False, box_is_attached=False, timeout=0.5):
    #TO DO: Whenever we change something in moveit we need to make sure that the interface has been updated properly

    pass

  def addObstacles(self):
    #TO DO: Add obstables in the world
    """ 
    **************************
    Adding Objects to the Planning Scene
    **************************
    """ 
    #Cargo names
    targets = ["RedBox",
               "BlueBox",
               "GreenBox"]
  
    Rbox_pose = geometry_msgs.msg.PoseStamped()
    Bbox_pose = geometry_msgs.msg.PoseStamped()
    Gbox_pose = geometry_msgs.msg.PoseStamped()

    Rbox_pose.header.frame_id = "%s1" % targets[0]
    Bbox_pose.header.frame_id = "%s1" % targets[1]
    Gbox_pose.header.frame_id = "%s1" % targets[2]

    # RED
    Rbox_pose.pose.orientation.w = 1.0
    Rbox_pose.pose.position.z = 0
    Rbox_name = targets[0]

    # BLUE
    Bbox_pose.pose.orientation.w = 2.0
    Bbox_pose.pose.position.z = 0
    Bbox_name = targets[1]

    # GREEN
    Gbox_pose.pose.orientation.w = 3.0
    Gbox_pose.pose.position.z = 0
    Gbox_name = targets[2]

    self.scene.add_box(Rbox_name, Rbox_pose, size=(0.06, 0.06, 0.06))
    self.scene.add_box(Bbox_name, Bbox_pose, size=(0.06, 0.06, 0.06))
    self.scene.add_box(Gbox_name, Gbox_pose, size=(0.06, 0.06, 0.06))

    #goal names
    boxes = ["DepositBoxGreen",
              "DepositBoxRed",
              "DepositBoxBlue"]

    

  def goToPose(self,pose_goal):
    #TO DO: Code used to move to a given position using move it
    
    pass


  def detachBox(self,box_name):
    #TO DO: Open the gripper and call the service that releases the box
    
    pass


  def attachBox(self,box_name):
    #TO DO: Close the gripper and call the service that releases the box
    
    pass

######################################################################################

class myNode():
  def __init__(self):
    #TO DO: Initialise ROS and create the service calls
    rospy.init_node('solution', anonymous=True)

    # Good practice trick, wait until the required services are online before continuing with the aplication
    rospy.wait_for_service('RequestGoal')
    rospy

  def getGoal(self,action):
    #TO DO: Call the service that will provide you with a suitable target for the movement

    pass


  def tf_goal(self, goal):
    #TO DO:Use tf2 to retrieve the position of the target with respect to the proper reference frame
    
    pass


  def main(self):
    #TO DO: Main code that contains the aplication
    self.planner = Planner()
    self.planner.addObstacles()

    rospy.signal_shutdown("Task Completed")


if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass
