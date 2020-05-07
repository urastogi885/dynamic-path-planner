from utils import planner, obstacle_space
import cv2


if __name__ == "__main__":
    grid = obstacle_space.Map()

    planner = planner.PathPlanning(grid, (0, 0), (9, 6))
    planner.explore()
    cv2.destroyAllWindows()
