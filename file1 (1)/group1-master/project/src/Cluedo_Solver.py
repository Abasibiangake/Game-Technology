#!/usr/bin/env python
# This final piece fo skeleton code will be centred around gettign the students to follow a colour and stop upon sight of another one.

from __future__ import division
import cv2
import cv2.cv as cv
import numpy as np
import rospy
import sys

from move_to_room_entrance import MoveToXandY
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

class cluedoSteps(Enum):
    moveToFirstRoom = 1
    scanForGreenOrRedCircle = 2
    moveToSecondRoom = 3
    moveToCentreOfCorrectRoom = 4
    scanForCluedoPoster = 5
    moveAndOrientateTowardsCluedoPoster = 6
    savePosterImage = 7
    identifyCluedoCharacter = 8
    saveTextFile = 9

class circleColour(Enum):
    noGreenOrRed = 0
    redFound = 1
    greenFound = 2

class cluedoSolver():

    def __init__(self):
        self.pubMove = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        #self.pubColours = rospy.Publisher('colour_chatter', String, queue_size=10)
        self.pubNavigation = rospy.Publisher('desired_position',Pose, queue_size = 10

        self.desiredVelocity = Twist()

        # Sensitivity for deciding range of accepted colours
        self.sensitivity = 5

        #Initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()		#Init cvBridge
        self.navigation = MoveToXandY()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)		#Subscriber topic to 3d camera

        self.cluedoSolverStepName = cluedoSteps.moveToFirstRoom      # TETSING PURPOSES STARTING HERE






    def callback(self, data):

        try:
            cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
            greenCircleSaveImage = cvImage
            hsvImage = cv2.cvtColor(cvImage, cv2.COLOR_BGR2HSV)

        except CvBridgeError as e:
			print(e)

        # Insert main code here
        if self.cluedoSolverStepName == cluedoSteps.moveToFirstRoom:
            room1_entrance_x = -2.3
            room1_entrance_y = 5.63
            position = {'x': room1_entrance_x, 'y': room1_entrance_y}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : -0.5, 'r4' : 0.1}
            self.navigation.movetoposition(position, quaternion)

        elif self.cluedoSolverStepName == cluedoSteps.scanForGreenOrRedCircle:
            foundCircle, turnSpeed, circleInMiddle = scanForCircleImage(cvImage, self.sensitivity)
            if foundCircle == circleColour.greenFound:                  #Success!
                if circleInMiddle == True:
                    self.cluedoSolverStepName = cluedoSteps.moveToCentreOfCorrectRoom
                    self.desiredVelocity.linear.x = 0
                    self.desiredVelocity.angular.z = 0
                    saveImageFile("green_circle.png", greenCircleSaveImage)

                else:
                    self.desiredVelocity.linear.x = 0
                    self.desiredVelocity.angular.z = turnSpeed
            elif scanForCircleImage(cvImage, self.sensitivity) == circleColour.redFound:
                self.cluedoSolverStepName = cluedoSteps.moveToSecondRoom
                self.desiredVelocity.linear.x = 0
                self.desiredVelocity.angular.z = 0
            else:
                self.desiredVelocity.linear.x = 0
                self.desiredVelocity.angular.z = 0.2

        elif self.cluedoSolverStepName == cluedoSteps.moveToSecondRoom:
            desiredVelocity = moveToPosition(desiredVelocity)

        elif self.cluedoSolverStepName == cluedoSteps.moveToCentreOfCorrectRoom:
            desiredVelocity = moveToPosition(desiredVelocity)

        elif self.cluedoSolverStepName == cluedoSteps.scanForCluedoPoster:
            foundPoster = scanForCluedoPoster(cvImage)
            if foundPoster == True:
                self.cluedoSolverStepName = cluedoSteps.moveAndOrientateTowardsCluedoPoster
            else:
                self.desiredVelocity.linear.x = 0
                self.desiredVelocity.angular.z = 0.2

        elif self.cluedoSolverStepName == cluedoSteps.moveAndOrientateTowardsCluedoPoster:
            foundPoster, posterInCorrectPosition, turnSpeed, linearSpeed = moveAndOrientateTowardsCluedoPoster(cvImage, self.sensitivity)
            if foundPoster == False:
                self.cluedoSolverStepName = cluedoSteps.moveToCentreOfCorrectRoom
            else:
                if posterInCorrectPosition == True:
                    self.cluedoSolverStepName = cluedoSteps.moveToCentreOfCorrectRoom
                    self.desiredVelocity.linear.x = 0
                    self.desiredVelocity.angular.z = 0
                else:
                    self.desiredVelocity.linear.x = linearSpeed
                    self.desiredVelocity.angular.z = turnSpeed


        elif self.cluedoSolverStepName == cluedoSteps.savePosterImage:
            x = 0
        elif self.cluedoSolverStepName == cluedoSteps.identifyCluedoCharacter:
            x = 0
        elif self.cluedoSolverStepName == cluedoSteps.saveTextFile:
            x = 0

        self.pubMove.publish(self.desiredVelocity)
        cv2.waitKey(3)


def moveToPosition(desiredVelocity):
    x = 0
    return desiredVelocity

def scanForCircleImage(rawCVImage, sensitivity):
    foundCircle = circleColour.noGreenOrRed
    greenCircleInMiddle = False

    hsvCircleImage = cv2.cvtColor(rawCVImage, cv2.COLOR_BGR2HSV)
    circleImage = cv2.medianBlur(rawCVImage, 5)                # Blur the image so that we avoid false circle detection by reducing noise
    circleImage = cv2.cvtColor(circleImage,cv2.COLOR_BGR2GRAY)  # Convert it to grayscale

    circles = cv2.HoughCircles(circleImage, cv.CV_HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=20, maxRadius=65)
    #circles = np.uint16(np.around(circles))
    #print(len(circles))

    turnSpeed = 0
    if circles is not None:
        for circle in circles[0,:]:
            #x = 0
            #cv2.circle(circleImage,(100,100),20,(255,255,255), 2)
            colourCircle = hsvCircleImage[int(circle[1]),int(circle[0])]
            #print("colour of circle is:" + str(colourCircle[0]) + " "  + str(colourCircle[1]) + " " + str(colourCircle[2]))
            if colourCircle[0] > (75 - sensitivity) and colourCircle[0] < (75 + sensitivity) and colourCircle[1] > 100: # and colourCircle[2] > 100:

                greenCentreX = circle[0]
                wy, wx, channels = rawCVImage.shape
                greenToCentreDistance = (0.5 * wx) - greenCentreX
                turnSpeed = greenToCentreDistance/400
                if turnSpeed > 0.2:
                    turnSpeed = 0.2
                if turnSpeed < -0.2:
                    turnSpeed = -0.2
                foundCircle = circleColour.greenFound
                cv2.circle(rawCVImage,(circle[0],circle[1]),circle[2],(255,255,255), 2)
                if greenToCentreDistance < 10 and greenToCentreDistance > -10:
                    greenCircleInMiddle = True


            if colourCircle[0] > (0 - sensitivity) and colourCircle[0] < (0 + sensitivity) and colourCircle[1] > 100 and colourCircle[2] > 100:
                cv2.circle(rawCVImage,(circle[0],circle[1]),circle[2],(0,0,0), 2)
                foundCircle = circleColour.redFound


    #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
    cv2.imshow('Camera_Feed', circleImage)	#final_image
    cv2.imshow('RGB_Feed', rawCVImage)

    return foundCircle, turnSpeed, greenCircleInMiddle

def scanForCluedoPoster(rawCVImage):
    foundPoster = False

    posterImage = cv2.medianBlur(rawCVImage, 5)     #blur the image
    grayScaleImage = cv2.cvtColor(posterImage,cv2.COLOR_BGR2GRAY)      #Convert to grayscale so we can look for edges

    edgedImage = cv2.Canny(grayScaleImage, 100, 250)                     # Run a canny edge detector algorithm
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    closedEdgedImage = cv2.morphologyEx(edgedImage, cv2.MORPH_CLOSE, kernel)
    #contourDrawnImage = closedEdgedImage.copy()

    contours, hierarchy = cv2.findContours(edgedImage.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    #print("number of contours: " + str(len(contours)))
    for contour in contours:
        # looking for 4 sided polygons, since our poster is a four sided shape
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)

        if len(approx) == 4 and cv2.contourArea(contour) > 250 and cv2.contourArea(contour) < 10000:
            print(cv2.contourArea(contour))
            cv2.drawContours(rawCVImage, contours, -1, (0, 255, 0), 3)
            print("contour drawn")



    cv2.imshow('Edges Image', edgedImage)
    cv2.imshow("Closed Edges Image", closedEdgedImage)
    #cv2.imshow("Contour drawn  Image", contourDrawnImage)
    cv2.imshow('RGB_Feed', rawCVImage)


    return foundPoster

def moveAndOrientateTowardsCluedoPoster(rawCVImage, sensitivity):
    turnSpeed = 0


    return turnSpeed

def saveImageFile(fileName, Image):
    try:
        cv2.imwrite(fileName, Image)
        print("file saved")
    except:
        print("error saving image of circle")







# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
	# Instantiate your class
	# And rospy.init the entire node
	cI = cluedoSolver()
	rospy.init_node("cluedoSolver", anonymous=True)		# And rospy.init the entire node
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down")
		cv2.destroyAllWindows()
	# Ensure that the node continues running with rospy.spin()
	# You may need to wrap it in an exception handler in case of KeyboardInterrupts
	# Remember to destroy all image windows before closing node

# Check if the node is executing in the main path
if __name__ == '__main__':
	main(sys.argv)
