# Import necessary standard libraries
import cv2
import numpy as np
# Import necessary constants
from utils.constants import *


class Map:
    def __init__(self):
        """
        Initialize map class
        """
        # Various class parameters
        self.height = SCALING_FACTOR * MAP_SIZE[0]
        self.width = SCALING_FACTOR * MAP_SIZE[1]
        self.black = (0, 0, 0)
        # Define radius of circular obstacles
        # Radius is same for all circles
        self.circle_radius = int(SCALING_FACTOR * 0.2)
        # Define centers of all circles
        self.circle_centers = np.array([(self.width - 100, 50),
                                        (300, self.height - 80),
                                        (150, self.height - 250)],
                                       dtype=np.int32)
        # Define empty world and add obstacles to it
        self.map_img = self.draw_obstacles()
        # Get image to search for obstacles
        self.check_img = self.generate_check_image()

    def draw_circle(self, img, thresh=0):
        """
        Draw the 4 circular obstacles on the map-image
        :return: nothing
        """
        for center in self.circle_centers:
            # Draw the circle
            cv2.circle(img, (center[0], center[1]), self.circle_radius + thresh, self.black, -1)

    def generate_check_image(self):
        """
        Get eroded image to check for obstacles considering the robot radius and clearance
        :return: image with obstacle space expanded to distance threshold between robot and obstacle
        """
        # Get map with obstacles
        check_img = self.map_img.copy()
        # Erode map image for rigid robot
        # self.draw_circle(check_img, self.thresh)

        return check_img

    def draw_obstacles(self):
        """
        Draw map using half-plane equations
        :return: map-image with all obstacles
        """
        self.map_img = cv2.imread('images/map.png')
        if self.map_img is None:
            self.map_img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            # Fill map-image with white color
            self.map_img.fill(255)
            # Draw various obstacles on the map
            self.draw_circle(self.map_img)
            cv2.imwrite('images/map.png', self.map_img)

        return self.map_img
