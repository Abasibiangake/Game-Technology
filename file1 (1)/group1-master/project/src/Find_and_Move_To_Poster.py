#!/usr/bin/env python
# This final piece fo skeleton code will be centred around gettign the students to follow a colour and stop upon sight of another one.

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
from kobuki_msgs.msg import BumperEvent

robotSeesPoster = False
areaIncreaseCounter = 0
posterTimeOutCounter = 100
changePlacesCounter = 0
collisionDetected = False
movementInOneDirectionCounter = 0


class posterFinder():

    def __init__(self):
        self.pubMove = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)

        # this publisher talks back to our main file once the poster has been found and positioned correctly
        #self.pubGoalReached = rospy.Publisher('poster_found',String, queue_size = 10)
        # This subscriber is listening for a command form main file to start looking for the poster
        #rospy.Subscriber('scan_for_poster', String, self.receivedCommand)

        self.desiredVelocity = Twist()
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, callback)

        # Sensitivity for deciding range of accepted colours
        self.scanningMode = True
        self.sensitivity = 5
        self.posterCloseEnough = False
        self.stayInThisLocationCounter = 750
        self.relocationCounter = 0


        #Initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        #self.bridge = CvBridge()		#Init cvBridge
        #self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)		#Subscriber topic to 3d camera



    def findPoster(self, data):
        global areaIncreaseCounter
        global robotSeesPoster
        global posterTimeOutCounter
        global collisionDetected
        global movementInOneDirectionCounter
        posterLocated = False
        posterArea = 0
        posterXScreenPosition = 0
        linearVelocity = 0
        angularVelocity = 0
        posterFound = False
        moveBackToCentre = False

        # If we want the robot to rotate on the spot and scan for the poster
        if self.scanningMode == True:
            cvImage = data
            posterLocated, posterArea, posterXScreenPosition = scanForCluedoPosterThresholdForPosters(cvImage, self.sensitivity)
            if ( posterLocated == True or robotSeesPoster == True):
                imageDimensions = cvImage.shape
                linearVelocity, angularVelocity = moveAndOrientateTowardsCluedoPoster(posterArea, posterXScreenPosition, imageDimensions[1], posterLocated, self.posterCloseEnough)
                self.desiredVelocity.linear.x = linearVelocity
                self.desiredVelocity.angular.z = angularVelocity
                areaIncreaseCounter = areaIncreaseCounter + 1
                posterTimeOutCounter = posterTimeOutCounter - 1
                posterTimeOutCounterChecker()

                if checkPosterSizeBigEnough(posterArea) == True or self.posterCloseEnough == True:
                    self.desiredVelocity.linear.x = 0
                    self.posterCloseEnough = True
                    if checkPosterIsCentralised(posterXScreenPosition, imageDimensions[1]) == True:
                        self.desiredVelocity.angular.z = 0
                        robotSeesPoster = False
                        print("your search is over, poster found")
                        path = os.path.realpath("project/Project_Output/cluedo_character.png")
                        saveImageFile(path, cvImage)
                        posterFound = True
            else:
                self.stayInThisLocationCounter = self.stayInThisLocationCounter - 1
                self.desiredVelocity.linear.x = 0
                self.desiredVelocity.angular.z = 0.2
            if self.stayInThisLocationCounter < 0:
                print("change Places!")
                self.scanningMode = False
                self.desiredVelocity.linear.x = 0
                self.desiredVelocity.angular.z = 0
        # if we cant find the poster we will attempt to move to a sudo random point
        else:
            if collisionDetected == True:
                if movementInOneDirectionCounter < 0:
                    collisionDetected = False
                    self.movementInOneDirectionCounter = 0
                    self.stayInThisLocationCounter = 500
                    self.desiredVelocity.linear.x = 0
                    self.desiredVelocity.angular.z = 0
                    self.scanningMode = True
                    self.relocationCounter = self.relocationCounter + 1
                    if self.relocationCounter > 0:
                        moveBackToCentre = True
                        areaIncreaseCounter = 0
                        print("move back to centre of room")
                        self.relocationCounter = 0
                    print("new location")
                else:
                    self.desiredVelocity.linear.x = -0.2
                    self.desiredVelocity.angular.z = 0
                    movementInOneDirectionCounter = movementInOneDirectionCounter - 1
            else:
                self.desiredVelocity.linear.x = 0.2
                self.desiredVelocity.angular.z = 0
                movementInOneDirectionCounter = movementInOneDirectionCounter + 1
                print("Im moving forward")
                if movementInOneDirectionCounter > 5000:
                    collisionDetected = True
                    movementInOneDirectionCounter = 0

        self.pubMove.publish(self.desiredVelocity)
        cv2.waitKey(3)

        return posterFound, moveBackToCentre

def callback(data):
    global collisionDetected
    global movementInOneDirectionCounter
    collisionDetected = True
    movementInOneDirectionCounter = movementInOneDirectionCounter/2

def posterTimeOutCounterChecker():
    global posterTimeOutCounter
    global areaIncreaseCounter
    global robotSeesPoster
    if posterTimeOutCounter < 0:
        print("poster timeout counter triggered")
        posterTimeOutCounter = 0
        #areaIncreaseCounter = 0
        robotSeesPoster = False

def scanForCluedoPosterThresholdForPosters(rawCVImage, sensitivity):
    foundPoster = False
    posterArea = 0
    posterX = 0
    upperSizeLimit = 0
    arrayXPositions = []
    global robotSeesPoster
    global areaIncreaseCounter
    global posterTimeOutCounter
    contourImage = rawCVImage.copy()

    cv_image_hsv = cv2.cvtColor(rawCVImage, cv2.COLOR_BGR2HSV)


    mustardMask = returnMustardMask(cv_image_hsv)
    scarlettMask = returnScarlettMask(cv_image_hsv)
    peacockMask = returnPeacockMask(cv_image_hsv)
    plum_mask = returnPlumMask(cv_image_hsv)
    #wardrobeMask = returnWardrobeMask(cv_image_hsv)

    final_mask = cv2.bitwise_or(scarlettMask, mustardMask)
    final_mask = cv2.bitwise_or(final_mask, peacockMask)
    final_mask = cv2.bitwise_or(final_mask, plum_mask)
    #final_mask = cv2.bitwise_or(final_mask, wardrobeMask)

    filtered_intensity_image = cv2.bitwise_and(rawCVImage, rawCVImage, mask=final_mask)
    # This image is the normal cv image that the camera sees but filtering out low values and saturations.

    blurredImage = cv2.medianBlur(filtered_intensity_image, 11)          #blur the image
    grayScaleImage = cv2.cvtColor(blurredImage,cv2.COLOR_BGR2GRAY)      #Convert to grayscale so we can look for edges

    edgedImage = cv2.Canny(grayScaleImage, 25, 80)                     # Run a canny edge detector algorithm
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    closedEdgedImage = cv2.morphologyEx(edgedImage, cv2.MORPH_CLOSE, kernel)
    #contourDrawnImage = closedEdgedImage.copy()

    contours, hierarchy = cv2.findContours(closedEdgedImage.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    count = 0
    posterContoursCounter = 0
    upperSizeLimit = 6500 + (areaIncreaseCounter * 100)
    biggestArea = 0
    contourBiggestArea = 0
    for contour in contours:
        # looking for 4 sided polygons, since our poster is a four sided shape

        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.024 * peri, True)
        cv2.drawContours(contourImage, contours, count, (255, 255, 255), 3)               # draw all contours white
        cv2.drawContours(contourImage, [approx], -1, (255, 0, 0), 2)                        # draw blue around approximation
        if len(approx) == 4:
            cv2.drawContours(contourImage, contours, count, (0, 0, 255), 3)               # if they are a quadrilateral draw them red
            if( cv2.contourArea(contour) > biggestArea ):
                biggestArea = cv2.contourArea(contour)
                contourBiggestArea = count

            if  cv2.contourArea(contour) > 1500 and cv2.contourArea(contour) < upperSizeLimit :                                           #cv2.contourArea(contour) > 800 and
                # lines = cv2.HoughLines(closedEdgedImage, 1, np.pi/180, 200)
                # if lines is not None:
                #     for line in lines:
                #         for x1, y1, x2, y2 in line:
                #             cv2.line(rawCVImage, (x1, y1), (x2, y2), (255,255,255), 3)

                cv2.drawContours(contourImage, contours, count, (0, 255, 0), 3)               # if you think its the poster draw it green

                foundPoster = True
                robotSeesPoster = True
                posterTimeOutCounter = 100
                posterArea = cv2.contourArea(contour)
                M = cv2.moments(contour)
                cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                posterX = cx
                arrayXPositions.append(cx)
        count = count + 1

    #cv2.drawContours(contourImage, contours, contourBiggestArea, (0, 255, 0), 3)
    #cv2.imshow("Gray scale", grayScaleImage)
    cv2.imshow("Closed Edges Image", closedEdgedImage)
    cv2.imshow('intensity masked', filtered_intensity_image)
    cv2.imshow('contours', contourImage)
    #cv2.imshow('RGB_Feed', rawCVImage)


    return foundPoster, posterArea, posterX

def returnMustardMask(cv_image_hsv):
    hsv_mustardBackground_low = np.array([20 ,0, 100])
    hsv_mustardBackground_high = np.array([35 ,145, 255])
    hsv_mustardShirt_low = np.array([18 ,0, 130])
    hsv_mustardShirt_high = np.array([25 ,20, 225])
    hsv_mustardJacket_low = np.array([16 ,110, 0])                         #Saturation seems fine, value needs slght adjustment
    hsv_mustardJacket_high =  np.array([60 ,180, 190])

    mustardBackgroundMask = cv2.inRange(cv_image_hsv, hsv_mustardBackground_low, hsv_mustardBackground_high)
    mustardShirtMask = cv2.inRange(cv_image_hsv, hsv_mustardShirt_low, hsv_mustardShirt_high)
    mustardJacketMask = cv2.inRange(cv_image_hsv, hsv_mustardJacket_low, hsv_mustardJacket_high)

    mustardMask = cv2.bitwise_or(mustardBackgroundMask, mustardShirtMask)      #mustardshirtmask
    mustardMask = cv2.bitwise_or(mustardMask, mustardJacketMask)

    return mustardMask

def returnScarlettMask(cv_image_hsv):
    hsv_scarlettBackground_low = np.array([0 ,100, 0])             #np.array([0 ,100, 30])
    hsv_scarlettBackground_high = np.array([10 ,190, 180])          #np.array([10 ,190, 180])

    hsv_scarlettShirt_low = np.array([170 ,100, 0])                #np.array([170 ,100, 30])
    hsv_scarlettShirt_high = np.array([180 ,190, 180])              #np.array([180 ,190, 180])
    #hsv_scarlettJacket_low = np.array([16 ,110, 0])
    #hsv_scarlettJacket_high =  np.array([60 ,180, 190])

    scarlettBackgroundMask = cv2.inRange(cv_image_hsv, hsv_scarlettBackground_low, hsv_scarlettBackground_high)
    scarlettShirtMask = cv2.inRange(cv_image_hsv, hsv_scarlettShirt_low, hsv_scarlettShirt_high)

    scarlettMask = cv2.bitwise_or(scarlettBackgroundMask, scarlettShirtMask)

    return scarlettMask

def returnPlumMask(cv_image_hsv):
    hsv_plumBackground_low = np.array([148 ,50, 25])
    hsv_plumBackground_high = np.array([162 ,150, 90])

    hsv_plumShirt_low = np.array([130 ,15, 18])
    hsv_plumShirt_high = np.array([180 ,60, 50])

    hsv_plumBlack_low = np.array([100 ,0, 0])
    hsv_plumBlack_high = np.array([180 ,60, 50])
    #hsv_scarlettJacket_low = np.array([16 ,110, 0])
    #hsv_scarlettJacket_high =  np.array([60 ,180, 190])

    plumBackgroundMask = cv2.inRange(cv_image_hsv, hsv_plumBackground_low, hsv_plumBackground_high)
    plumShirtMask = cv2.inRange(cv_image_hsv, hsv_plumShirt_low, hsv_plumShirt_high)
    plumBlackMask = cv2.inRange(cv_image_hsv, hsv_plumBlack_low, hsv_plumBlack_high)

    plumMask = cv2.bitwise_or(plumBackgroundMask, plumShirtMask)
    plumMask = cv2.bitwise_or(plumBackgroundMask, plumBlackMask)

    return plumMask

def returnPeacockMask(cv_image_hsv):
    hsv_peacockBackground_low = np.array([95 ,100, 50])
    hsv_peacockBackground_high = np.array([105 ,180, 255])

    hsv_peacockShirt_low = np.array([102 ,140, 50])
    hsv_peacockShirt_high = np.array([119 ,255, 255])

    hsv_peacockSkin_low = np.array([10 ,40, 80])
    hsv_peacockSkin_high = np.array([35 ,90, 255])
    #hsv_scarlettJacket_low = np.array([16 ,110, 0])
    #hsv_scarlettJacket_high =  np.array([60 ,180, 190])

    peacockBackgroundMask = cv2.inRange(cv_image_hsv, hsv_peacockBackground_low, hsv_peacockBackground_high)
    peacockShirtMask = cv2.inRange(cv_image_hsv, hsv_peacockShirt_low, hsv_peacockShirt_high)
    peacockSkinMask = cv2.inRange(cv_image_hsv, hsv_peacockSkin_low, hsv_peacockSkin_high)

    peacockMask = cv2.bitwise_or(peacockBackgroundMask, peacockShirtMask)
    peacockMask = cv2.bitwise_or(peacockMask, peacockSkinMask)

    return peacockMask

def returnWardrobeMask(cv_image_hsv):
    hsv_wardrobe_low = np.array([13 ,30, 30])
    hsv_wardrobe_high = np.array([18 ,255, 160])

    hsv_wardrobeEdge_low = np.array([13 ,170, 30])
    hsv_wardrobeEdge_high = np.array([18 ,255, 90])

    wardrobeMask = cv2.inRange(cv_image_hsv, hsv_wardrobe_low, hsv_wardrobe_high)
    wardrobeEdgeMask = cv2.inRange(cv_image_hsv, hsv_wardrobeEdge_low, hsv_wardrobeEdge_high)
    wardrobeMask = cv2.bitwise_xor(wardrobeMask, wardrobeEdgeMask)

    return wardrobeMask

def scanForCluedoPoster(rawCVImage, sensitivity):
    foundPoster = False
    posterArea = 0
    posterX = 0
    upperSizeLimit = 0
    arrayXPositions = []
    global robotSeesPoster
    global areaIncreaseCounter
    contourImage = rawCVImage.copy()

    # first convert to hsv so we can filte3r by the intensity of the image, attempt to get rid of the background wall
    cv_image_hsv = cv2.cvtColor(rawCVImage, cv2.COLOR_BGR2HSV)
    hsv_low_intensity = np.array([0 ,0, 0])
    hsv_high_intensity = np.array([255, 50, 255])
    hsv_low_wardrobes = np.array([10, 100, 50])
    hsv_high_wardrobes = np.array([25, 255, 255])
    hsv_low_cone = np.array([0, 200, 0])
    hsv_high_cone = np.array([55, 255, 255])

    hsv_low_highintensity = np.array([0, 0, 100])
    hsv_high_highintensity = np.array([255, 255, 255])

    intensity_mask = cv2.inRange(cv_image_hsv, hsv_low_intensity, hsv_high_intensity)
    wardrobe_mask = cv2.inRange(cv_image_hsv, hsv_low_wardrobes, hsv_high_wardrobes)
    cone_mask = cv2.inRange(cv_image_hsv, hsv_low_cone, hsv_high_cone)
    high_intensity_mask = cv2.inRange(cv_image_hsv, hsv_low_highintensity, hsv_high_highintensity)

    final_mask = cv2.bitwise_or(intensity_mask, wardrobe_mask)
    final_mask = cv2.bitwise_or(final_mask, cone_mask)
    #final_mask = cv2.bitwise_or(final_mask, high_intensity_mask)

    final_mask = cv2.bitwise_not(final_mask)

    filtered_intensity_image = cv2.bitwise_and(rawCVImage, rawCVImage, mask=final_mask)
    # This image is the normal cv image that the camera sees but filtering out low values and saturations.

    blurredImage = cv2.medianBlur(filtered_intensity_image, 9)          #blur the image
    grayScaleImage = cv2.cvtColor(blurredImage,cv2.COLOR_BGR2GRAY)      #Convert to grayscale so we can look for edges

    edgedImage = cv2.Canny(grayScaleImage, 50, 150)                     # Run a canny edge detector algorithm
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    closedEdgedImage = cv2.morphologyEx(edgedImage, cv2.MORPH_CLOSE, kernel)
    #contourDrawnImage = closedEdgedImage.copy()

    contours, hierarchy = cv2.findContours(closedEdgedImage.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    count = 0
    posterContoursCounter = 0
    upperSizeLimit = 3500 + (areaIncreaseCounter * 120)
    biggestArea = 0
    contourBiggestArea = 0
    for contour in contours:
        # looking for 4 sided polygons, since our poster is a four sided shape

        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
        cv2.drawContours(contourImage, contours, count, (255, 255, 255), 3)               # draw all contours white
        cv2.drawContours(contourImage, [approx], -1, (255, 0, 0), 2)                        # draw blue around approximation
        if len(approx) == 4:
            cv2.drawContours(contourImage, contours, count, (0, 0, 255), 3)               # if they are a quadrilateral draw them red
            if( cv2.contourArea(contour) > biggestArea ):
                biggestArea = cv2.contourArea(contour)
                contourBiggestArea = count

            if  cv2.contourArea(contour) > 1000 and cv2.contourArea(contour) < 25000:                                           #cv2.contourArea(contour) > 800 and
                # lines = cv2.HoughLines(closedEdgedImage, 1, np.pi/180, 200)
                # if lines is not None:
                #     for line in lines:
                #         for x1, y1, x2, y2 in line:
                #             cv2.line(rawCVImage, (x1, y1), (x2, y2), (255,255,255), 3)

                cv2.drawContours(contourImage, contours, count, (0, 255, 0), 3)               # if you think its the poster draw it green

                foundPoster = True
                robotSeesPoster = True
                posterArea = cv2.contourArea(contour)
                M = cv2.moments(contour)
                cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                posterX = cx
                arrayXPositions.append(cx)
        count = count + 1

    #cv2.drawContours(contourImage, contours, contourBiggestArea, (0, 255, 0), 3)
    cv2.imshow("Gray scale", grayScaleImage)
    cv2.imshow("Closed Edges Image", closedEdgedImage)
    cv2.imshow('intensity masked', filtered_intensity_image)
    cv2.imshow('contours', contourImage)
    cv2.imshow('RGB_Feed', rawCVImage)


    return foundPoster, posterArea, posterX

def moveAndOrientateTowardsCluedoPoster(posterArea, posterXScreenPosition, widthOfScreen, posterFound, posterCloseEnough):
    turnSpeed = 0
    linearSpeed = 0

    if posterFound == False:
        #print("move and orientate, poster not found")
        turnSpeed = 0
        linearSpeed = 0
    else:
        posterToCentreDistance = (0.5 * widthOfScreen) - posterXScreenPosition
        turnSpeed = posterToCentreDistance/2000
        if turnSpeed > 0.2:
            turnSpeed = 0.2
        if turnSpeed < -0.2:
            turnSpeed = -0.2

        if posterArea != 0:
            linearSpeed = (22000 / posterArea) * 0.1
            if linearSpeed > 0.2:
                linearSpeed = 0.2
            if posterCloseEnough == True:
                linearSpeed = 0

    return linearSpeed, turnSpeed

def checkPosterSizeBigEnough(posterArea):
    posterIsWithinRange = False
    #print("poster area is " + str(posterArea))
    if posterArea > 22000:
        posterIsWithinRange = True

    return posterIsWithinRange

def checkPosterIsCentralised(posterCentreX, widthOfScreen):
    posterInMiddle = False
    posterToCentreDistance = (0.5 * widthOfScreen) - posterCentreX
    if posterToCentreDistance < 100 and posterToCentreDistance > -100:
        posterInMiddle = True

    return posterInMiddle

def saveImageFile(fileName, Image):
    try:
        cv2.imwrite(fileName, Image)
        print("file saved")
    except:
        print("error saving image of poster")
