#! /usr/bin/env python

# ROS node to kill the exploration node once the target is reached  

import rospy
import os
import math
import copy
from takshak.msg import poseDict
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal

# gate_barrier = 7

def pose_callback(msg):
    # set additional offset
    offset = 2
    # calc the avg x position of the gates
    avg_x = 0
    c = 0
    for i in msg.gate_poses.poses:
        avg_x += i.position.x
        c += 1

    global gate_barrier
    gate_barrier = avg_x/c - offset

def move_base_callback(msg):
    global dist_to_goal

    x = msg.goal.target_pose.pose.position.x - current_pose.pose.position.x
    y = msg.goal.target_pose.pose.position.y - current_pose.pose.position.y
    z = msg.goal.target_pose.pose.position.z - current_pose.pose.position.z

    dist_to_goal = math.sqrt(x**2 + y**2 + z**2) 

def odom_callback(msg):    
    global current_x, current_pose
    current_pose = PoseStamped()
    current_pose.pose = msg.pose.pose
    current_x = msg.pose.pose.position.x

def kill_explore():
    os.system("rosnode kill explore")

def save_map():
    os.system("rosrun map_server map_saver -f arena_map")

def stop_move_base():
    pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=10)

    goal = copy.deepcopy(current_pose)
    goal.header.frame_id = "odom"
    goal.header.stamp = rospy.Time.now()

    while pub.get_num_connections() == 0:
        pass
    rospy.loginfo('sending stop command')
    pub.publish(goal)

def main(pose_dict_topic):
    rospy.init_node('exploration_killer')
    global pose_sub, odom_sub
    pose_sub = rospy.Subscriber(pose_dict_topic, poseDict, pose_callback)
    odom_sub = rospy.Subscriber('/odom',Odometry, odom_callback)
    move_base_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, move_base_callback)
    while not 'gate_barrier' in globals() and not rospy.is_shutdown():
        rospy.loginfo('waiting for gate locations')

    while not 'current_x' in globals() and not rospy.is_shutdown(): 
        rospy.loginfo('waiting for /odom')

    while not 'dist_to_goal' in globals() and not rospy.is_shutdown(): 
        rospy.loginfo('waiting for /move_base/goal')

    while not rospy.is_shutdown():
        
        if current_x > gate_barrier or dist_to_goal > 14:
            rospy.loginfo('initiating endgame')
            kill_explore()
            stop_move_base()
            save_map()
            os.system('rosrun takshak walker.py')
            break

    rospy.loginfo('crossed barrier. killing exploration. saving map. shutting down node')
        
if __name__ == '__main__':
    main('/gate_poses')