#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rospkg
import sys
import math
import numpy as np

# global param, err

# waypoints
waypoints = []
way_n = 0

# PID parameters
param = [0.5, 0.2, 0]

# PID Errors
err = [0, 0]


# function to get Cross-Track Error
def getCTE(current_pose):

    # coordinates of start point
    start_pose = waypoints[way_n]

    # coordinates of goal point
    goal_pose = waypoints[way_n + 1]

    # percentage path covered b/w waypoints
    distance = (((current_pose[0] - start_pose[0]) * (goal_pose[0] - start_pose[0])) +
                ((current_pose[1] - start_pose[1]) * (goal_pose[1] - start_pose[1]))) / \
               (((goal_pose[0] - start_pose[0]) ** 2) + ((goal_pose[1] - start_pose[1]) ** 2))

    return distance


# function to convert quaternion to euler
def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    if yaw < -3.14:
        yaw += 2 * 3.14
    elif yaw > 3.14:
        yaw -= 2 * 3.14

    return yaw


# function to get omega angle
def yawE(x, y, t):
    goal = np.divide(waypoints[way_n +1], 2)
    print('waypoint index is {}'.format(way_n+1))
    print('waypoint  is {}'.format(waypoints[way_n+1]))
    print('goal is {}'.format(goal))

    direction = math.atan2(goal[1] - y, goal[0] - x + 0.001)

    angle = direction - t
    if angle > 0:
        if angle > abs(2 * math.pi - angle):
            angle = -(2 * math.pi - angle)
    else:
        if abs(angle) > abs((2 * math.pi - abs(angle))):
            angle = (2 * math.pi - abs(angle))

    return angle


# function to get angular velocity
def control(error):
    global err, param

    # angular velocity
    omega = param[0] * error + param[1] * (error - err[0]) + param[2] * err[1]

    # for Kd and Ki error
    err[1] += error
    err[0] = error

    return omega


# function to callback subscriber node
def callback_odom(odom):
    global way_n

    # declaring object
    msg = Twist()
    global state_action
    while state_action[way_n][1] is None:
        way_n += 1
    # current position stored
    current_pose = (odom.pose.pose.position.x, odom.pose.pose.position.y)

    # get error
    dist = getCTE(current_pose)

    yaw = quaternion_to_euler(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, \
                              odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)

    error = yawE(odom.pose.pose.position.x, odom.pose.pose.position.y, yaw  )

    if dist > 1:
        # update waypoints
        way_n += 1
        print('Way-point: ', way_n)

        if way_n >= len(waypoints) - 1:
            print("GOAL REACHED")
            msg.angular.z = 0
            msg.linear.x = 0
            pub.publish(msg)
            rospy.signal_shutdown("Goal Reached")

    # getting angular velocity
    angular_velocity = control(error)
    # assign angular velocity
    msg.angular.z = angular_velocity
    msg.linear.x = 0.15

    # publishing
    pub.publish(msg)


# function to convert waypoints into meaning
def convertor(wayp, size=10):
    cord_wayp = []
    yaw_angle = []

    # getting x, y location
    for i in range(len(wayp)):
        x = - (wayp[i][0][1] + 0.25)
        y = - (wayp[i][0][0] + 0.25)
        if wayp[i][1] is not None:
            cord_wayp.append((x, y))
            angle = wayp[i][1] * 0.785
            yaw_angle.append(angle)

    return cord_wayp, yaw_angle


# main function
if __name__ == '__main__':
    # load state_action pair
    ros_root = rospkg.RosPack()
    state_action = np.load(ros_root.get_path('dynamic-path-planner') + '/o2.npy', allow_pickle=True)

    # get coordinates and angles
    waypoints, angles = convertor(state_action)

    # initialize node
    rospy.init_node('o2', anonymous=True)

    # publisher object publishing to command velocity
    pub = rospy.Publisher('/obstacle2/cmd_vel', Twist, queue_size=10)

    # initialize subscriber node to get robot's odometry
    sub = rospy.Subscriber('/obstacle2/odom', Odometry, callback_odom)

    rospy.spin()
