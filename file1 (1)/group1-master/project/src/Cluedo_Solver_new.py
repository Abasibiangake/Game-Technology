#!/usr/bin/env python
from __future__ import division
import cv2
import cv2.cv as cv
import numpy as np
import rospy
import sys
import yaml
import os

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from enum import Enum
from geometry_msgs.msg import Pose, Point, Quaternion
import yaml
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from move_to_room_entrance import MoveToXandY
from find_circle import CircleFinder
from Find_and_Move_To_Poster import posterFinder
from final_identify_character import whodunit

class cluedoSolver():

    def __init__(self):

        # Required to convert ros image to openCV image
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.createCvImage)

        #Initialise Node
        rospy.init_node("cluedoSolver", anonymous=True)
        #print("initialised cluedo node")

        #Initialise Variables
        self.getPositionPoints()

    def getPositionPoints(self):
        #get points out of yaml file
        path = os.path.realpath("project/Input_Points/input_points.yaml")
        with open(path, 'r') as stream:
            points = yaml.safe_load(stream)

        # get room entrances
        self.room1_entrance_x = points['room1_entrance_xy'][0]
        self.room1_entrance_y = points['room1_entrance_xy'][1]
        self.room2_entrance_x = points['room2_entrance_xy'][0]
        self.room2_entrance_y = points['room2_entrance_xy'][1]

        # get room centres
        self.room1_centre_x = points['room1_centre_xy'][0]
        self.room1_centre_y = points['room1_centre_xy'][1]
        self.room2_centre_x = points['room2_centre_xy'][0]
        self.room2_centre_y = points['room2_centre_xy'][1]

    def goToPosition(self,x,y):
        #position = {'x': x, 'y': y}
        theta = 30
        r1 = 0.000
        r2 = 0.000
        r3 = np.sin(theta/2.0)
        r4 = np.cos(theta/2.0)
        #quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
        #pose = Pose(Point(x, y, 0.00),Quaternion(r1, r2, r3, r4))

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.000

        pose.orientation.x = r1
        pose.orientation.y = r2
        pose.orientation.z = r3
        pose.orientation.w = r4

        #print(type(pose))
        movement = MoveToXandY()
        sucess = movement.movetoposition(pose)
        return sucess
        #print(pose)

    def findCircle(self):
        cF = CircleFinder()
        circleFound = False
        while not circleFound:
            moveOn, colourOfCircle = cF.findCircle(self.cvImage)
            circleFound = moveOn
        return colourOfCircle

    def findPoster(self):
        pF = posterFinder()
        posterFound = False
        backToCentre = False
        endSearch = False
        while endSearch == False:
            posterFound, backToCentre = pF.findPoster(self.cvImage)
            if(backToCentre == True or posterFound == True):
                endSearch = True

        return posterFound, backToCentre

    def createCvImage(self, cameraFeed):
        try:
            self.cvImage = self.bridge.imgmsg_to_cv2(cameraFeed, "bgr8")
        except CvBridgeError as e:
			print(e)

def main(args):
    #try:
    cS = cluedoSolver()
    #go to room 1 entrance

    correctRoomFound = False
    roomToCheck = 1
    posterFound = False
    while not correctRoomFound:
        if roomToCheck == 1:
            cS.goToPosition(cS.room1_entrance_x, cS.room1_entrance_y)
            foundCircleColour = cS.findCircle()
            if foundCircleColour == "Green":
                correctRoomFound = True
                cS.goToPosition(cS.room1_centre_x, cS.room1_centre_y)
            elif foundCircleColour =="Red":
                roomToCheck = 2
        elif roomToCheck == 2:
            cS.goToPosition(cS.room2_entrance_x, cS.room2_entrance_y)
            foundCircleColour = cS.findCircle()
            if foundCircleColour == "Green":
                correctRoomFound = True
                cS.goToPosition(cS.room2_centre_x, cS.room2_centre_y)
            elif foundCircleColour =="Red":
                roomToCheck = 1
    while not posterFound:
        posterFound, backToCentre = cS.findPoster()
        if backToCentre == True:
            print("main file, moving to centre of room again")
            if roomToCheck == 1:
                cS.goToPosition(cS.room1_centre_x, cS.room1_centre_y)
            elif roomToCheck == 2:
                cS.goToPosition(cS.room2_centre_x, cS.room2_centre_y)
    wdi = whodunit()
    wdi.identifyCharacter()
    cv2.destroyAllWindows()


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
        cv2.destroyAllWindows()

# Check if the node is executing in the main path
if __name__ == '__main__':
	main(sys.argv)
