#!/usr/bin/env python3
# Python 2/3 compatibility imports
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
import math

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
    self.xarm_group = xarm_group
    xgripper = moveit_commander.MoveGroupCommander("xarm_gripper")
    self.xgripper = xgripper

  def wait_for_state_update(self,box_name, box_is_known=False, box_is_attached=False, timeout=0.5):
    #TO DO: Whenever we change something in moveit we need to make sure that the interface has been updated properly

    pass

  def addObstacles(self):
    #TO DO: Add obstables in the world
    # Instantiate a PlanningSceneInterface object
    scene = moveit_commander.PlanningSceneInterface()
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
    Rbox_pose.header.frame_id = targets[0]
    Bbox_pose.header.frame_id = targets[1]
    Gbox_pose.header.frame_id = targets[2]
    # RED
    Rbox_pose.pose.orientation.w = 1.0
    Rbox_pose.pose.position.z = 0
    Rbox_name = "RedBox"
    # BLUE
    Bbox_pose.pose.orientation.w = 2.0
    Bbox_pose.pose.position.z = 0
    Bbox_name = "BlueBox"
    # GREEN
    Gbox_pose.pose.orientation.w = 3.0
    Gbox_pose.pose.position.z = 0
    Gbox_name = "GreenBox"
    scene.add_box(Rbox_name, Rbox_pose, size=(0.06, 0.06, 0.06))
    scene.add_box(Bbox_name, Bbox_pose, size=(0.06, 0.06, 0.06))
    scene.add_box(Gbox_name, Gbox_pose, size=(0.06, 0.06, 0.06))
    #goal names
    boxes = ["DepositBoxGreen",
              "DepositBoxRed",
              "DepositBoxBlue"]
  

    
  def goToPose(self, pose_goal): 
    #TO DO: Code used to move to a given position using move it
    self.xarm_group.set_pose_target(pose_goal)
    self.xarm_group.go()
    self.xarm_group.stop()
    self.xarm_group.clear_pose_targets()


  def detachBox(self,box_name):
    #TO DO: Open the gripper and call the service that releases the box
    self.xgripper.set_named_target("open")
    self.xgripper.go(wait = True)


  def attachBox(self,box_name):
    #TO DO: Close the gripper and call the service that releases the box
    self.xgripper.set_named_target("close")
    selfxgripper.go(wait = True)

######################################################################################

class myNode():
  def __init__(self):
    #TO DO: Initialise ROS and create the service calls
    rospy.init_node('solution', anonymous=True)

    # Good practice trick, wait until the required services are online before continuing with the aplication
    rospy.wait_for_service('RequestGoal')
    rospy.wait_for_service('AttachObject')
    

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

    # Get Box Position
    box = geometry_msgs.msg.Pose()
    box.position.x = 0.214284
    box.position.y = 0.187592
    box.position.z = 0.001827
    box.orientation.x = 0.999998
    box.orientation.y = -0
    box.orientation.z = -0.00190996
    box.orientation.w = -0.000413786
    self.planner.goToPose(box)

    # Get Deposit Position
    deposit = geometry_msgs.msg.Pose()
    deposit.position.x = 0.22664
    deposit.position.y = -0.411781
    deposit.position.z = 0.29226
    deposit.orientation.x = 0.999766
    deposit.orientation.y = 0.02116288
    deposit.orientation.z = -0
    deposit.orientation.w = 0
    self.planner.goToPose(deposit)

    rospy.signal_shutdown("Task Completed")


if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass
