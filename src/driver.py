#!/usr/bin/env python

import roslib; roslib.load_manifest('project_rrt')
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees


class driver(object):
    
    def __init__(self):
        rospy.init_node("rrt_driver")

    def create_nav_goal(self,x, y, yaw):
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.frame_id = '/map' 
        mb_goal.target_pose.pose.position.x = x
        mb_goal.target_pose.pose.position.y = y
        mb_goal.target_pose.pose.position.z = 0.0 
    
        # Orientation of the robot is expressed in the yaw value of euler angles
        angle = radians(yaw) # angles are expressed in radians
        quat = quaternion_from_euler(0.0, 0.0, angle) # roll, pitch, yaw
        mb_goal.target_pose.pose.orientation = Quaternion(*quat.tolist())
    
        return mb_goal


    def drive(self):        
        # Connect to the navigation action server
        nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Connecting to /move_base AS...")
        nav_as.wait_for_server()
        rospy.loginfo("Connected.")
    
        rospy.loginfo("Creating navigation goal...")
        nav_goal = self.create_nav_goal(1.0, 0.0, 0.0)
        rospy.loginfo("Sending goal to x=1.0 y=0.0 yaw=0...")
        nav_as.send_goal(nav_goal)
        rospy.loginfo("Waiting for result...")
        nav_as.wait_for_result()
        nav_res = nav_as.get_result()
        nav_state = nav_as.get_state()
        rospy.loginfo("Done!")
        print "Result: ", str(nav_res) 
        print "Nav state: ", str(nav_state) 
