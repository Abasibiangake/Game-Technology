#!/usr/bin/env python

import rospy
import numpy as np
#from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
#import yaml
#room1_entrance_x = -1.43
#room1_entrance_y = 1.15

class MoveToXandY():
    def __init__(self):
        self.sent_goal = False

        #what to do if shut down (e.g. ctrl + C or failure)
        #rospy.on_shutdown(self.shutdown)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up...")

    	# Allow up to 5 seconds for the action server to come up
    	self.move_base.wait_for_server(rospy.Duration(5))

    def movetoposition(self, pose):
        #print(type(pose))
        #quat = {'r1' : 0.000, 'r2' : 0.000, 'r3' : -0.5, 'r4' : 0.1}

        reachedGoal = False
        self.sent_goal = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose

        #print(goal)

        #Make robot start moving
        self.move_base.send_goal(goal) # goal is published properly (goal can be seen in rostopic echo move_base/goal) but robot does not move
        rospy.loginfo("Robot is moving to the goal set.")

        #Complete motion in 60secs ie 1min
        self.motion_success = self.move_base.wait_for_result(rospy.Duration(60))

        #print(self.motion_success)

        if not self.motion_success:
            self.move_base.cancel_goal()
            rospy.loginfo("The Turtlrbot has failed to reach the set goal position")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                reachedGoal = True
                rospy.loginfo("We made it! reached position")

        rospy.sleep(1)
        return reachedGoal

    # def shutdown(self):
    #     if self.sent_goal:
    #         self.move_base.cancel_goal()
    #     rospy.loginfo("Stop")
    #     rospy.sleep(1)
