import numpy as np
from utils import obstacle_space, constants
import cv2


class PathPlanning:

    def __init__(self, robot_loc, target_loc):
        self.grid_obj = obstacle_space.Map()
        self.grid = self.grid_obj.map_img
        self.robot = list(robot_loc)
        self.target = list(target_loc)
        self.grid[self.robot[0], self.robot[1]] = constants.ROBOT_LOC_VALUE
        self.grid[self.target[0], self.target[1]] = constants.TARGET_LOC_VALUE
        self.distance = np.full_like(self.grid, fill_value=constants.MAX_DISTANCE, dtype=float)
        cv2.circle(self.grid_obj.animation_img, (int(constants.SCALING_FACTOR*(self.target[1]+0.5)),
                                                 int(constants.SCALING_FACTOR*(self.target[0]+0.5))),
                   self.grid_obj.circle_radius, constants.TARGET_BGR, -1)
        self.distance[self.target[0], self.target[1]] = 0
        self.action_encoding = {'00': None, '0-1': 0, '1-1': 1, '10': 2, '11': 3, '01': 4, '-11': 5, '-10': 6, '-1-1': 7}
        # Define video-writer of open-cv to record the exploration and final path
        video_format = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
        self.video_output = cv2.VideoWriter('video_output.avi', video_format, 200.0,
                                            (500, 500))

    def get_target_dist(self, loc):
        return np.sqrt((loc[0]-self.target[0])**2 + (loc[1]-self.target[1])**2)

    def move_robot(self):
        cv2.circle(self.grid_obj.animation_img, (int(constants.SCALING_FACTOR * (self.target[1] + 0.5)),
                                                 int(constants.SCALING_FACTOR * (self.target[0] + 0.5))),
                   self.grid_obj.circle_radius, constants.TARGET_BGR, -1)
        self.grid[self.robot[0]][self.robot[1]] = constants.FREE_SPACE_VALUE
        cv2.circle(self.grid_obj.animation_img, (int(constants.SCALING_FACTOR*(self.robot[1]+0.5)),
                                                 int(constants.SCALING_FACTOR*(self.robot[0]+0.5))),
                   self.grid_obj.circle_radius, constants.FREE_SPACE_BGR, -1)
        min_dist_loc = self.get_next_move()
        action = np.subtract(min_dist_loc, self.robot)
        if 0 <= min_dist_loc[0] < self.grid.shape[0] and 0 <= min_dist_loc[1] < self.grid.shape[1]:
            self.robot = min_dist_loc
        self.grid[self.robot[0]][self.robot[1]] = constants.ROBOT_LOC_VALUE
        cv2.circle(self.grid_obj.animation_img, (int(constants.SCALING_FACTOR*(self.robot[1]+0.5)),
                                                 int(constants.SCALING_FACTOR*(self.robot[0]+0.5))),
                   self.grid_obj.circle_radius, constants.ROBOT_BGR, -1)
        return self.action_encoding[str(action[0])+str(action[1])]

    def get_next_move(self):
        min_dist = constants.MAX_DISTANCE
        min_dist_loc = self.robot
        for row in range(-1, 2, 1):
            for col in range(-1, 2, 1):
                if 0 <= self.robot[0] + row < self.grid.shape[0] and 0 <= self.robot[1] + col < self.grid.shape[1]:
                    if min_dist > self.distance[self.robot[0]+row][self.robot[1]+col]:
                        min_dist = self.distance[self.robot[0]+row][self.robot[1]+col]
                        min_dist_loc = [self.robot[0]+row, self.robot[1]+col]
        return min_dist_loc

    @staticmethod
    def get_dist(loc1, loc2):
        return np.sqrt((loc1[0]-loc2[0])**2 + (loc1[1]-loc2[1])**2)

    def get_f_value(self, index):
        min_f_value = constants.MAX_DISTANCE
        for i in range(-1, 2, 1):
            for j in range(-1, 2, 1):
                if (self.grid.shape[0] <= index[0]+i or index[0] + i < 0 or
                        self.grid.shape[1] <= index[1]+j or index[1] + j < 0):
                    continue
                f_value = self.get_dist(index, (index[0]+i, index[1]+j)) + self.distance[index[0]+i][index[1]+j]
                if f_value < min_f_value:
                    min_f_value = f_value
        return min_f_value

    def explore(self):
        # 2 - target, 1- empty, 0 - obstacles, 3- robot
        step = 0
        waypoints = []
        while True:
            step += 1
            self.grid_obj.map_img = self.grid_obj.update_obstacle_space()
            self.grid = np.copy(self.grid_obj.map_img)
            old_distance = np.copy(self.distance)
            for row in range(len(self.grid)):
                for col in range(len(self.grid[0])):
                    if self.grid[row][col] == constants.TARGET_LOC_VALUE:
                        old_distance[row][col] = 0
                    elif self.grid[row][col] == constants.OBSTACLE_LOC_VALUE:
                        old_distance[row][col] = constants.MAX_DISTANCE
                    else:
                        f_value = self.get_f_value((row, col))
                        old_distance[row][col] = min([constants.MAX_DISTANCE, f_value])
            self.distance = old_distance
            action = self.move_robot()
            waypoints.append([self.robot, action])
            self.grid_obj.map_img = np.copy(self.grid)
            for _ in range(250):
                self.video_output.write(self.grid_obj.animation_img)
            cv2.imshow('map', self.grid_obj.animation_img)
            cv2.waitKey(250)
            if self.robot == self.target:
                print('Reached Target!!...')
                self.video_output.release()
                np.save('waypoints.npy', waypoints)
                for key in self.grid_obj.obstacles.keys():
                    np.save(key, self.grid_obj.obstacles[key])
                # cv2.waitKey(0)
                break
