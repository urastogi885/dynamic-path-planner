import numpy as np
from utils import obstacle_space


class PathPlanning:

    def __init__(self, grid, robot_loc, target_loc):
        self.grid = grid
        self.robot = list(robot_loc)
        self.target = list(target_loc)
        self.grid[self.robot[0], self.robot[1]] = 3
        self.grid[self.target[0], self.target[1]] = 2
        self.distance = np.full_like(self.grid,fill_value=float('inf'), dtype=float)
        self.distance[self.target[0],self.target[1]] = 0
        #self.b_grid = np.full_like(self.grid, fill_value=0, dtype=int)

    def get_target_dist(self, loc):
        return np.sqrt((loc[0]-self.target[0])**2 + (loc[1]-self.target[1])**2)

    def move_robot(self):
        self.grid[self.robot[0]][self.robot[1]] = 1
        min_dist_loc = self.get_next_move()
        if 0 <= min_dist_loc[0] < self.grid.shape[0] and 0 <= min_dist_loc[1] < self.grid.shape[1]:
            self.robot = min_dist_loc
        self.grid[self.robot[0]][self.robot[1]] = 3
        return

    def get_next_move(self):
        min_dist = float('inf')
        min_dist_loc = self.robot
        for row in range(-1, 2, 1):
            for col in range(-1, 2, 1):
                if 0 <= self.robot[0] + row < self.grid.shape[0] and 0 <= self.robot[1] + col < self.grid.shape[1]:
                    if min_dist > self.distance[self.robot[0]+row][self.robot[1]+col]:
                        min_dist = self.distance[self.robot[0]+row][self.robot[1]+col]
                        min_dist_loc = [self.robot[0]+row, self.robot[1]+col]
        return min_dist_loc

    def get_dist(self, loc1, loc2):
        return np.sqrt((loc1[0]-loc2[0])**2 + (loc1[1]-loc2[1])**2)

    def get_fvalue(self, index):
        min_f_value = float('inf')
        for i in range(-1,2,1):
            for j in range(-1,2,1):
                if self.grid.shape[0] <= index[0]+i or index[0] + i < 0 or self.grid.shape[1] <= index[1]+j or index[1] + j < 0:
                    continue
                    #return min_f_value
                f_value = self.get_dist(index, (index[0]+i, index[1]+j)) + self.distance[index[0]+i][index[1]+j]
                if f_value < min_f_value:
                    min_f_value = f_value
        return min_f_value

    def explore(self):
        # 2 - target, 1- empty, 0 - obstacles, 3- robot
        step = 0
        while True:
            step += 1
            old_distance = np.copy(self.distance)
            for row in range(len(self.grid)):
                for col in range(len(self.grid[0])):
                    if self.grid[row][col] == 2:
                        old_distance[row][col] = 0
                    elif self.grid[row][col] == 0:
                        old_distance[row][col] = float('inf')
                    else:
                        f_value = self.get_fvalue((row, col))
                        old_distance[row][col] = min([float('inf'), f_value])
            self.distance = old_distance
            self.move_robot()

            if self.robot == self.target:
                print('Reached Target!!...')
                break


if __name__=="__main__":
    #grid = obstacle_space.Map()
    grid = np.ones((10, 10))
    #grid
    planner = PathPlanning(grid, (0, 0), (9, 6))
    planner.explore()

