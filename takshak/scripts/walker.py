#! /usr/bin/env python

import rospy
import cv2
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from takshak.msg import poseDict

def count_balls():
    img = cv2.imread("arena_map.pgm",0)
    cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1.29,4.0,
                            param1=26,param2=16,minRadius=7,maxRadius=10)
    count=0
    if circles is not None:
        for i in circles[0,:]:
            count=count+1
    print(count)
    return count%5

rospy.init_node('walker')
rospy.loginfo('walker is running...')
stepper = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=10)

# dictionary of poses
msg = rospy.wait_for_message('/gate_poses',poseDict)
table = {}
j = 0
for i in msg.gates:
    table[i] = [ msg.gate_poses.poses[j+0], msg.gate_poses.poses[j+1], msg.gate_poses.poses[j+2] ]
    j += 3

# print(table)
# from the map
count = count_balls()

# lessgo
steps = table[count]
for step in steps:

    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose = step
    goal.pose.orientation.x = 0
    goal.pose.orientation.y = 0
    goal.pose.orientation.z = 0
    goal.pose.orientation.w = 1

    while stepper.get_num_connections() == 0:
        pass

    goal.header.stamp = rospy.Time.now()
    stepper.publish(goal)
    print(goal)
    result = rospy.wait_for_message('/move_base/result',MoveBaseActionResult)

rospy.loginfo('reached')