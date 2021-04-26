#!/usr/bin/env python
# This final piece fo skeleton code will be centred around gettign the students to follow a colour and stop upon sight of another

from __future__ import division
import cv2
import cv2.cv as cv
import numpy as np
import rospy
import sys
import imutils
import copy
import os

class whodunit():
    def identifyCharacter(self):


        image_path = os.path.realpath("project/Project_Output/cluedo_character.png")
        scarlet_path = os.path.realpath("project/cluedo_images/scarlet.png")
        mustard_path = os.path.realpath("project/cluedo_images/mustard.png")
        peacock_path = os.path.realpath("project/cluedo_images/peacock.png")
        plum_path = os.path.realpath("project/cluedo_images/plum.png")

        image = cv2.imread(image_path,1)
        template1 = cv2.imread(scarlet_path,1)
        template2 = cv2.imread(mustard_path,1)
        template3 = cv2.imread(peacock_path,1)
        template4 = cv2.imread(plum_path,1)

        template_array = [template1, template2, template3, template4]

        #name of each template in order
        name1 = 'Scarlet.png'
        name2 = 'Mustard.png'
        name3 = 'Peacock.png'
        name4 = 'Plum.png'
        name_array = [name1, name2, name3, name4]

        n=-1
        visualize = False
        found = None

        for cnt, template in enumerate(template_array):
            template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
            template_resized=imutils.resize(template, width = int(template.shape[1] * 0.2))
            template_edged = cv2.Canny(template_resized, 50, 200)
            (tH,tW) = template_edged.shape[:2]

            image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # Make image gray
            for scale in np.linspace(0.1, 1.0, 20)[::-1]: #0.5, 1.5, 20 for scarlett_diff, 0.1, 1.0, 20 for cluedo_characterone
                image_resized = imutils.resize(image_gray, width = int(image_gray.shape[1] * scale))
                ratio = image_gray.shape[1] / float(image_resized.shape[1])

                if image_resized.shape[0] < tH or image_resized.shape[1] < tW:
                    continue

                image_edged = cv2.Canny(image_resized, 50, 200) #Apply canny to get image edges
                result = cv2.matchTemplate(image_edged, template_edged, cv2.TM_CCOEFF) #match image edges to template edges
                (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result) #Takes the correlation result and returns min correlation value, max correlation value, xy coordinates for min value,xy coordinates for max val
                if visualize: #flag
                    clone = np.dstack([image_edged, image_edged, image_edged]) # stack 2D arrays (images) into a single 3D array for processing
                    cv2.rectangle(clone, (maxLoc[0], maxLoc[1]),(maxLoc[0] + tW, maxLoc[1] + tH), (0, 0, 255), 2) #draw rectangle
                    cv2.imshow("Visualize", clone) # display clone for each loop
                    cv2.waitKey(100)

                if found is None or maxVal > found[0]: # if current max correlation value > previous max correlation value, save this new value
                    found = (maxVal, maxLoc, ratio)
                    bestTemplateCnt = copy.copy(cnt) #save index of matching template
            continue


        print name_array[bestTemplateCnt] # print out the close matching template name
        text_path = os.path.realpath("project/Project_Output/cluedo_character.txt")
        data = name_array[bestTemplateCnt]
        data = data.replace(".png", "")
        txtf = open(text_path, "w+")
        txtf.write(data)
        txtf.close()

        #Save character in the room with the right name identified
        #cv2.imwrite(name_array[bestTemplateCnt],image)

        # unpack the bookkeeping variable and compute the (x, y) coordinates
        # of the bounding box based on the image resized ratio
        (_, maxLoc, ratio) = found
        (startX, startY) = (int(maxLoc[0] * ratio), int(maxLoc[1] * ratio))
        (endX, endY) = (int((maxLoc[0] + tW) * ratio), int((maxLoc[1] + tH) * ratio))

        # draw a bounding box around the detected result and display the image
        cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2)
        cv2.imshow("Image", image)
        cv2.waitKey(2000) # wait 2s



def main(args):

    characterFinder = whodunit()
    characterFinder.identifyCharacter()

# Check if the node is executing in the main path
if __name__ == '__main__':
	main(sys.argv)
