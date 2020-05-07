from utils import planner
import cv2


if __name__ == "__main__":
    planner = planner.PathPlanning((0, 0), (9, 6))
    planner.explore()
    cv2.destroyAllWindows()
