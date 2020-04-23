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
        self.circle_centers = np.array([(8, 1),
                                        (6, 8.4),
                                        (3, 5)],
                                       dtype=np.int8)
        self.scaled_centers = SCALING_FACTOR * np.array([(8, 1),
                                                         (6, 8.4),
                                                         (3, 5)])
        # Define empty world and add obstacles to it
        self.map_img = self.generate_check_image()
        # Get image to search for obstacles
        self.check_img = self.draw_obstacles()

    def draw_circle(self, img, thresh=0):
        """
        Draw the 4 circular obstacles on the map-image
        :return: nothing
        """
        for center in self.scaled_centers:
            # Draw the circle
            cv2.circle(img, (int(center[0]), int(center[1])), self.circle_radius + thresh, self.black, -1)

    def generate_check_image(self):
        """
        Get eroded image to check for obstacles considering the robot radius and clearance
        :return: image with obstacle space expanded to distance threshold between robot and obstacle
        """
        # Get map with obstacles
        # Initialize empty grid
        check_img = np.zeros((MAP_SIZE[0], MAP_SIZE[1]), dtype=np.int8)
        # Mark obstacle locations
        for center in self.circle_centers:
            check_img[center[1]][center[0]] = -1
        # Define target location
        # check_img[9][6] = 1
        return check_img

    def draw_obstacles(self):
        """
        Draw map using half-plane equations
        :return: map-image with all obstacles
        """
        self.check_img = cv2.imread('images/map.png')
        if self.check_img is None:
            self.check_img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            # Fill map-image with white color
            self.check_img.fill(255)
            # Draw various obstacles on the map
            self.draw_circle(self.check_img)
            cv2.imwrite('images/map.png', self.check_img)

        return self.check_img
