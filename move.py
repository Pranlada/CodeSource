#!/usr/bin/env python3

import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan

def angle_diff(a, b):
    """Shortest signed angular difference between a and b."""
    d = (a - b + math.pi) % (2 * math.pi) - math.pi
    return d


def computeAngularVelocity(target_angle, max_w=1.0):
    """Angular velocity controller."""
    error = angle_diff(target_angle, 0.0)   # assumes robot faces yaw=0 in body frame
    w = 1.0 * error
    return max(-max_w, min(max_w, w))


def PIDgain():
    return 1.0, 0.0, 0.0  


class SimpleAvoidance:
    def __init__(self, safe_distance=0.5):
        self.safe_distance = safe_distance

    def __call__(self, scan):
        """Check if something is in front; return steering angle if blocked."""
        ranges = np.array(scan.ranges)
        ranges = ranges[np.isfinite(ranges)]  # clean up NaN/inf
        if len(ranges) == 0:
            return 0.0

        mid = len(ranges) // 2
        front = ranges[mid - 5: mid + 5]  # narrow slice
        if np.min(front) < self.safe_distance:
            return math.pi / 4  # suggest turning left
        return 0.0


def get_pose(topic):
    msg = rospy.wait_for_message(topic, Pose2D, timeout=5.0)
    return np.array([msg.x, msg.y, msg.theta])


def moveToGoal(lidarTopic, poseTopic, cmdPub, velMsg, goal, tol=0.3):
    avoidance = SimpleAvoidance()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pose = get_pose(poseTopic)
        dx, dy = goal[0] - pose[0], goal[1] - pose[1]
        dist = math.hypot(dx, dy)

        if dist < tol:
            break

        angle_to_goal = math.atan2(dy, dx)
        heading_error = angle_diff(angle_to_goal, pose[2])

        v = 0.2 if dist > 0.5 else 0.1
        w = 1.0 * heading_error

        scan = rospy.wait_for_message(lidarTopic, LaserScan)
        avoid_angle = avoidance(scan)
        if avoid_angle != 0.0:
            v = 0.0
            w = computeAngularVelocity(avoid_angle, 0.5)

        velMsg.linear.x = v
        velMsg.angular.z = w
        cmdPub.publish(velMsg)
        rate.sleep()

    velMsg.linear.x = 0.0
    velMsg.angular.z = 0.0
    cmdPub.publish(velMsg)

def toy_following():
    rospy.init_node('toy_following_simple')

    cmdPub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    velMsg = Twist()

    toyPoseTopic = '/toy_pose'
    robotPoseTopic = '/robot_position'
    lidarTopic = '/scan'

    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        toy = rospy.wait_for_message(toyPoseTopic, Pose2D)
        goal = [toy.x, toy.y]

        moveToGoal(lidarTopic, robotPoseTopic, cmdPub, velMsg, goal, tol=0.4)


if __name__ == "__main__":
    try:
        toy_following()
    except rospy.ROSInterruptException:
        pass
