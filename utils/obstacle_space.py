# Import necessary standard libraries
import cv2
import numpy as np
from random import randint
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
        self.scaled_centers = SCALING_FACTOR * self.circle_centers.copy()
        # Define empty world and add obstacles to it
        self.map_img = self.generate_map()
        # Get image to search for obstacles
        self.check_img = self.draw_obstacles()

    def update_scaled_centers(self):
        """
        Method to update scaled centers
        Only to be called if obstacle centers have been changed
        :return: nothing
        """
        self.scaled_centers = SCALING_FACTOR * self.circle_centers.copy()
        pass

    def draw_circle(self, img, thresh=0):
        """
        Draw the 4 circular obstacles on the map-image
        :return: nothing
        """
        for center in self.scaled_centers:
            # Draw the circle
            cv2.circle(img, (int(center[0]), int(center[1])), self.circle_radius + thresh, self.black, -1)

    def generate_map(self):
        """
        Get eroded image to check for obstacles considering the robot radius and clearance
        :return: image with obstacle space expanded to distance threshold between robot and obstacle
        """
        # Get map with obstacles
        # Initialize empty grid
        #check_img = np.zeros((MAP_SIZE[0], MAP_SIZE[1]), dtype=np.int8)
        check_img = np.full((MAP_SIZE[0], MAP_SIZE[1]), fill_value=FREE_SPACE_VALUE, dtype=np.int8)
        # Mark obstacle locations
        for center in self.circle_centers:
            check_img[center[1]][center[0]] = OBSTACLE_LOC_VALUE
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

    def update_obstacle_space(self):
        """
        Method to move obstacles around on the map
        :return: map-image with updated positions of the obstacles
        """
        for i in range(len(self.circle_centers)):
            center = self.circle_centers[i][0], self.circle_centers[i][1]
            self.map_img[center[1]][center[0]] = FREE_SPACE_VALUE
            action = randint(0, TOTAL_ACTIONS)
            self.circle_centers[i] = self.take_action(action, center)
            self.map_img[self.circle_centers[i][1]][self.circle_centers[i][0]] = OBSTACLE_LOC_VALUE
        return self.map_img

    @staticmethod
    def take_action(action, center_obstacle):
        """
        Method to change centers of the obstacles
        :param action: an integer indicating the action
        :param center_obstacle: a tuple containing center of the obstacle
        :return: a tuple containing new centers of the obstacle
        """
        center_new = center_obstacle
        # Take required and get new center coordinates of the obstacles
        if action == ACTION_SPACE['up']:
            center_new = center_obstacle[0], center_obstacle[1] - VELOCITY_OBSTACLE
        elif action == ACTION_SPACE['down']:
            center_new = center_obstacle[0], center_obstacle[1] + VELOCITY_OBSTACLE
        elif action == ACTION_SPACE['left']:
            center_new = center_obstacle[0] - VELOCITY_OBSTACLE, center_obstacle[1]
        elif action == ACTION_SPACE['right']:
            center_new = center_obstacle[0] + VELOCITY_OBSTACLE, center_obstacle[1]
        elif action == ACTION_SPACE['top_left']:
            center_new = center_obstacle[0] - VELOCITY_OBSTACLE, center_obstacle[1] - VELOCITY_OBSTACLE
        elif action == ACTION_SPACE['top_right']:
            center_new = center_obstacle[0] + VELOCITY_OBSTACLE, center_obstacle[1] - VELOCITY_OBSTACLE
        elif action == ACTION_SPACE['bottom_left']:
            center_new = center_obstacle[0] - VELOCITY_OBSTACLE, center_obstacle[1] + VELOCITY_OBSTACLE
        elif action == ACTION_SPACE['bottom_right']:
            center_new = center_obstacle[0] + VELOCITY_OBSTACLE, center_obstacle[1] + VELOCITY_OBSTACLE
        # Check for out of bounds condition
        if ((center_new[0] <= 0 or center_new[0] > MAP_SIZE[0] - 1) or
                (center_new[1] <= 0 or center_new[1] > MAP_SIZE[1] - 1)):
            center_new = center_obstacle
        return center_new
