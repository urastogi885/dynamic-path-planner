# Define map size
MAP_SIZE = 10, 10
# Define scaling factor
SCALING_FACTOR = 50
# Define velocity of the obstacle
VELOCITY_OBSTACLE = 1   # unit/time-step
# Define action space for the obstacle
TOTAL_ACTIONS = 8
ACTION_SPACE = {'up': 0, 'down': 1, 'left': 2, 'right': 3,
                'top_left': 4, 'top_right': 5, 'bottom_left': 6, 'bottom_right': 7}
# Define robot's value in the map grid
ROBOT_LOC_VALUE = 3
# Define target's value in the map grid
TARGET_LOC_VALUE = 2
# Define free space's value in the map grid
FREE_SPACE_VALUE = 1
# Define obstacle's value in the map grid
OBSTACLE_LOC_VALUE = 0
# Define max distance value in the distance grid
MAX_DISTANCE = float('inf')
# Defining robot BGR values in rendered map
ROBOT_BGR = (0, 255, 0)
# Defining target BGR values in rendered map
TARGET_BGR = (0, 0, 255)
# Defining free space BGR values in rendered map
FREE_SPACE_BGR = (255, 255, 255)
