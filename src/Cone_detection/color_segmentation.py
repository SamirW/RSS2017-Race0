from matplotlib import colors
import cv2
import numpy as np
import copy
import math
# These imports are just for displaying all images in directory, remove in robot code
import glob, os

class TestCV():
    def __init__(self, image_name, image_file_string):
        self.image_name = image_name
        self.image = None
        self.image_file_string = image_file_string

    def readImage(self):
        img = cv2.imread(self.image_name)
        self.image = img
        #print self.image
    def parseImage(self):
        # Convert the image from BGR to HSV
        height, width, channels = self.image.shape
           

        # Convert the image from BGR to HSV
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        sliced_height = int(round(height*0.25))

        image_cut = hsv[sliced_height:]


        #print self.image
        #hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        # This sets the limits for what is considered "orange". A pixel with values in the range lower-upper for H, S, and V is considered part of the cone.
        # These limts are currently HARDCODED for an orange cone. Really bright lighting, etc, could mess this up.
        # The limits are in HSV. In cv2, hue ranges fr2424e2a014f5ccafc18b1121c493387bba9f878fom 0-179, and saturation ranges from 0-255.
        upper_orange = np.array([16, 255, 255])

        # Lower orange bound for photos from our robot
        lower_orange = np.array([6, 200, 135])

        # Lower orange bound for stock footage
        # lower_orange = np.array([1, 20, 20])

        # Initialized to orange values
        upper_green = np.array([90, 255, 255])

        lower_green = np.array([70, 40, 35])


        # Apply upper and lower limits, where pixels inside limits -> white and pixels outside of limits -> black.
        mask_orange = cv2.inRange(image_cut, lower_orange, upper_orange)
        mask_green = cv2.inRange(image_cut, lower_green, upper_green)

        mask = cv2.bitwise_or(mask_orange, mask_green)

        image_orange, contours_orange, hierarchy_orange = cv2.findContours(mask_orange, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        image_green, contours_green, hierarchy_green = cv2.findContours(mask_green, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        longest_contour_orange = np.array(max(contours_orange, key=len))
        longest_contour_green = np.array(max(contours_green, key=len))

        if len(longest_contour_orange) > len(longest_contour_green):
            longest_contour = np.array(longest_contour_orange)
        else:
            longest_contour = np.array(longest_contour_green)

        # Find moments corresponding to all contour points
        M = cv2.moments(np.array(longest_contour))
        # Find centroid of cone
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        print cx,cy

        #print cx, cy

        # Flattening all contour points list
        flat_contour_pts = longest_contour.flat

        # Filtering out X coordinates
        contour_x_coord = flat_contour_pts[0:len(longest_contour.flat):2]

        # Finding max and min x-coordinates (edge points)
        min_x = min(contour_x_coord)
        max_x = max(contour_x_coord)

        index_min = np.where(contour_x_coord==min_x)
        index_max = np.where(contour_x_coord==max_x)

        min_y = flat_contour_pts[index_min[0][0]*2+1]
        max_y = flat_contour_pts[index_max[0][0]*2+1]

        # avg y coordinate
        avg_y = np.mean([min_y, max_y])





        # Find a bounding box for the cone.
        x, y, w, h = cv2.boundingRect(np.array(longest_contour))

        print 'x, y, w, h', x, y, w, h
        bbox = cv2.rectangle(self.image, (x, y+sliced_height), (x+w, y+h+sliced_height), (0, 255, 0), 1)

        centroid_img = cv2.circle(bbox, (cx, cy+sliced_height), 5, (255, 0, 0), -1)
        cv2.imwrite("Processed_"+self.image_file_string,centroid_img)
        cv2.imshow('bb cont', centroid_img)


        # Visualization of a variety of different things. Should not be in final product.
        #cv2.imshow('image', self.image)
        # cv2.imshow('mask', mask)
        #drawCont = cv2.drawContours(self.image, contours, -1, (0, 255, 0), 1)
        #print drawCont
        #cv2.imshow('cont', drawCont)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def height(self):
        # Find the height and width of the bounding box.
        height = abs(self.box[1][1]-self.box[0][1])
        #print 'height', height
        width = abs(self.box[2][0]-self.box[1][0])
        #print 'width', width

    def orientation(self):
        # Find the orientation of the box, assuming an orientation of zero = no heading.
        height = abs(self.box[2][1]-self.box[1][1])
        width = abs(self.box[2][0]-self.box[1][0])
        #print "height, width", height, width

        # Find orientation IN RADIANS
        orientation = math.atan(height/width)
        #print 'orientation', math.degrees(orientation)


if __name__ == '__main__':
    # img = TestCV('/home/racecar/racecar-ws/src/lab4/images/rotated_images/high_rotate_3feet.png', 'high_rotate_3feet.png')
    #img = TestCV('/home/racecar/racecar-ws/src/race_0/media/Img/Green/green_dark_0.png', 'green_dark_0.png')
    #img.readImage()
    #img.parseImage()
    # This runs filtering and identification for all images in directory
    for file in os.listdir("/home/racecar/racecar-ws/src/race_0/media/Img/Green/"):
        if file.endswith(".png"):
            print file
            img = TestCV("/home/racecar/racecar-ws/src/race_0/media/Img/Green/"+file, file)
            img.readImage()
            img.parseImage()
    