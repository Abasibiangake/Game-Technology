#!/usr/bin/env python

from __future__ import division
import cv2
import cv2.cv as cv
import numpy as np
import rospy
import sys
import os

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from enum import Enum
from geometry_msgs.msg import Pose, Point, Quaternion

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

class CircleFinder():
    def __init__(self):
        # Publishers
        self.pubMove = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        self.pubNavigation = rospy.Publisher('desired_position',Pose, queue_size = 10)

        # Subscribers
        #self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.findCircle)

        # Allows us to send correct type to robot base
        self.desiredVelocity = Twist()
        # Sensitivity for deciding range of accepted colours
        self.sensitivity = 5
        # Required to convert ros image to openCV image
        # self.bridge = CvBridge()

    def findCircle(self, data):

        moveOn = False

        imageToBeSaved = data

        foundCircle, turnSpeed, circleInMiddle = self.scanForCircleImage(data)
        if foundCircle == circleColour.greenFound:                  #Success!
            if circleInMiddle == True:
                self.desiredVelocity.linear.x = 0
                self.desiredVelocity.angular.z = 0
                path = os.path.realpath("project/Project_Output/green_circle.png")
                self.saveImageFile(path, imageToBeSaved)
                cv2.destroyAllWindows()
                moveOn = True
                colourOfCircle = "Green"
            else:
                self.desiredVelocity.linear.x = 0
                self.desiredVelocity.angular.z = turnSpeed
                colourOfCircle = "Green"
        elif foundCircle == circleColour.redFound:
            print("red circle found")
            self.desiredVelocity.linear.x = 0
            self.desiredVelocity.angular.z = 0
            moveOn = True
            colourOfCircle = "Red"
        else:
            self.desiredVelocity.linear.x = 0
            self.desiredVelocity.angular.z = 0.2
            colourOfCircle = "None"

        self.pubMove.publish(self.desiredVelocity)
        #print(foundCircle)
        return moveOn, colourOfCircle

    def scanForCircleImage(self, rawCVImage):

        foundCircle = circleColour.noGreenOrRed
        greenCircleInMiddle = False

        drawImage = rawCVImage.copy()


        hsvCircleImage = cv2.cvtColor(drawImage, cv2.COLOR_BGR2HSV)

        hsv_low_intensity = np.array([0 ,0, 0])
        hsv_high_intensity = np.array([255, 255, 255])

        intensity_mask = cv2.inRange(hsvCircleImage, hsv_low_intensity, hsv_high_intensity)

        filtered_intensity_image = cv2.bitwise_and(drawImage, drawImage, mask=intensity_mask)

        circleImage = cv2.medianBlur(filtered_intensity_image, 5)                # Blur the image so that we avoid false circle detection by reducing noise
        circleImage = cv2.cvtColor(circleImage,cv2.COLOR_BGR2GRAY)  # Convert it to grayscale

        circles = cv2.HoughCircles(circleImage, cv.CV_HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=20, maxRadius=65)
        #circles = np.uint16(np.around(circles))cv2.destroyAllWindows()
        #print(len(circles))

        turnSpeed = 0
        if circles is not None:
            for circle in circles[0,:]:
                #x = 0
                #cv2.circle(circleImage,(100,100),20,(255,255,255), 2)
                colourCircle = hsvCircleImage[int(circle[1]),int(circle[0])]
                #print("colour of circle is:" + str(colourCircle[0]) + " "  + str(colourCircle[1]) + " " + str(colourCircle[2]))

                averageColour = self.findAverageColour(circle[0], circle[1], circle[2], hsvCircleImage)
                if colourCircle[0] > (75 - self.sensitivity) and colourCircle[0] < (75 + self.sensitivity) and colourCircle[1] > 100: # if its a green circle

                    greenCentreX = circle[0]
                    wy, wx, channels = drawImage.shape
                    greenToCentreDistance = (0.5 * wx) - greenCentreX
                    turnSpeed = greenToCentreDistance/400
                    if turnSpeed > 0.2:
                        turnSpeed = 0.2
                    if turnSpeed < -0.2:
                        turnSpeed = -0.2
                    foundCircle = circleColour.greenFound
                    cv2.circle(drawImage,(circle[0],circle[1]),circle[2],(255,255,255), 2)
                    if greenToCentreDistance < 10 and greenToCentreDistance > -10:
                        #print("circle in middle")
                        greenCircleInMiddle = True
                # If the circle found is red
                if averageColour[0] > (0 - self.sensitivity) and averageColour[0] < (0 + self.sensitivity) and averageColour[1] > 100 and colourCircle[2] > 100:
                    cv2.circle(drawImage,(circle[0],circle[1]),circle[2],(0,0,0), 2)
                    #print(averageColour)
                    foundCircle = circleColour.redFound

        #print(circleImage)
        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        #cv2.imshow('Grayscale', circleImage)	#final_image
        #cv2.imshow('HSV', hsvCircleImage)
        cv2.imshow('RGB_Feed', drawImage)
        cv2.waitKey(3)

        return foundCircle, turnSpeed, greenCircleInMiddle

    def findAverageColour(self, x, y, r, image):

        wy, wx, channels = image.shape
        slightRight = int(x + (0.5 * r))
        farRight = int(x + r)
        if slightRight >= wx:
            slightRight = wx - 1
            farRight = wx - 1
        elif farRight >= wx:
            farRight = wx - 1

        slightLeft = int(x - (0.5 * r))
        farLeft = int(x - r)
        if slightLeft < 0:
            slightLeft = 0
            farLeft = 0
        elif farLeft < 0:
            farLeft = 0

        slightUp = y + (0.5 * r)
        farUp = y + r
        if slightUp >= wy:
            slightUp = wy - 1
            farUp = wy - 1
        elif farLeft >= wy:
            farUp = wy - 1

        slightDown = y - (0.5 * r)
        farDown = y - r
        if slightDown < 0:
            slightDown = 0
            farDown = 0
        elif farDown < 0:
            farDown = 0

        top2Colour = image[int(farUp),int(x)]
        top1Colour = image[int(slightUp),int(x)]
        middleColour = image[int(y),int(x)]
        bottom1Colour = image[int(slightDown),int(x)]
        bottom2Colour = image[int(farDown),int(x)]
        left1Colour = image[int(y),slightLeft]
        left2Colour = image[int(y),farLeft]
        right1Colour = image[int(y),slightRight]
        #print("farRight: " + str(farRight))
        #print("wx: " + str(wx))
        right2Colour = image[int(y),farRight]

        allTenPointsArray = np.array([top2Colour, top1Colour, middleColour, bottom1Colour, bottom2Colour, left1Colour, left2Colour, right1Colour, right2Colour])
        averageColour = np.average(allTenPointsArray, axis = 0)
        #averageColour = (middleColour + topColour + bottomColour + leftColour + rightColour)/5
        #print(middleColour)
        #print(averageColour)
        return averageColour


    def saveImageFile(self, fileName, Image):
        try:
            cv2.imwrite(fileName, Image)
            print("file saved")
        except:
            print("error saving image of circle")
