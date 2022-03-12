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
    robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object
    scene = moveit_commander.PlanningSceneInterface()
    
    # Instantiate a MoveGroupCommander object
    group_name = "xarm6"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Create a DisplayTrajectory ROS publisher
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

    """ 
    **************************
    Getting Basic Information
    **************************
    """ 
    # We can get the name of the reference frame for this robot
    planning_frame = move_group.get_planning_frame()
    print("Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group
    eef_link = move_group.get_end_effector_link()
    print("End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot
    group_names = robot.get_group_names()
    print("Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the robot
    print("Printing robot state")
    print(robot.get_current_state())
    print("")

    
    """ 
    **************************
    Planning to a Joint Goal
    **************************
    """ 
    # We get the joint values from the group and change some of the values
    tau = 2*pi
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -tau / 8
    joint_goal[2] = 0
    joint_goal[3] = -tau / 4
    joint_goal[4] = 0
    joint_goal[5] = tau / 6

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()


    """ 
    **************************
    Planning to a Pose Goal
    **************************
    """ 
    # Plan a motion for this group to a desired pose for the end-effector
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    move_group.set_pose_target(pose_goal)

    # call the planner to compute the plan and execute it
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    move_group.clear_pose_targets()


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
    Rbox_pose.header.frame_id = "%s" % (targets[0], '1')
    Bbox_pose.header.frame_id = "%s" % (targets[1], '1')
    Gbox_pose.header.frame_id = "%s" % (targets[2], '1')
    # RED
    Rbox_pose.pose.orientation.w = 1.0
    Rbox_pose.pose.position.z = 0
    Rbox_name = "%s" % (targets[0], '1')
    # BLUE
    Bbox_pose.pose.orientation.w = 2.0
    Bbox_pose.pose.position.z = 0
    Bbox_name = "%s" % (targets[1], '1')
    # GREEN
    Gbox_pose.pose.orientation.w = 3.0
    Gbox_pose.pose.position.z = 0
    Gbox_name = "%s" % (targets[2], '1')

    scene.add_box(Rbox_name, Rbox_pose, size=(0.06, 0.06, 0.06))
    scene.add_box(Bbox_name, Bbox_pose, size=(0.06, 0.06, 0.06))
    scene.add_box(Gbox_name, Gbox_pose, size=(0.06, 0.06, 0.06))

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