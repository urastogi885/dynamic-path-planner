import numpy as np
from utils import obstacle_space


class PathPlanning:

    def __init__(self, grid, robot_loc, target_loc):
        self.grid = grid
        self.robot = robot_loc
        self.target = target_loc
        self.distance = np.full_like(self.grid,fill_value=float('inf'), dtype= float)
        self.distance[self.target[0],self.target[1]] = 0
        self.b_grid = np.full_like(self.grid, fill_value=0, dtype= int)

    def get_target_dist(self, loc):
        return np.sqrt((loc[0]-self.target[0])**2 + (loc[1]-self.target[1])**2)

    def move_robot(self):
        pass

    def get_next_move(self):
        min_dist = float('inf')
        min_dist_loc = self.robot
        for row in range(-1, 1, 1):
            for col in range(-1, 1, 1):
                if min_dist > self.distance[self.robot[0]+row][self.robot[1]+col]:
                    min_dist = self.distance[self.robot[0]+row][self.robot[1]+col]
                    min_dist_loc = [self.robot[0]+row, self.robot[1]+col]
        return min_dist_loc

    def explore(self):
        # 1 - target, 0- empty, -1 - obstacles, 2- robot
        while True:
            for row in range(len(self.grid)):
                for col in range(len(self.grid[0])):
                    if self.grid[row][col] == 0:
                        old_value = self.distance[row][col]
                        self.distance[row][col] += self.get_target_dist((row, col))

            self.move_robot()


