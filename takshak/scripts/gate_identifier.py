#! /usr/bin/env python
# this node will sub from image. depth, aruco dictionary and pub to gate_poses and rviz markers
from copy import copy
import rospy
import cv2
import numpy as np
import ros_numpy
import tf2_ros
import tf
import copy
from takshak.msg import IntColor,poseDict
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from tf2_geometry_msgs import PoseStamped
from visualization_msgs.msg import Marker

def depth_callback(msg):
    global ros_point_cloud
    ros_point_cloud = msg

def image_callback(ros_image):
    global cv2_image
    cv2_image = bridge.imgmsg_to_cv2(ros_image,desired_encoding="bgr8")

def dict_callback(msg):
    tolerance = 5
    global colour_list, colour_values

    colour_list = []
    for i in msg.num:
        colour_list.append(int(i))

    # if len(colour_list) == 5:
    #     dict_listener.unregister()

    colour_values = {}
    i = 0
    for colour in colour_list:
        lower = [msg.rgb[i+0] -tolerance, msg.rgb[i+1] -tolerance, msg.rgb[i+2] -tolerance]
        upper = [msg.rgb[i+0] +tolerance, msg.rgb[i+1] +tolerance, msg.rgb[i+2] +tolerance]
        for j in range(0,3):
            if lower[j] < 0:
                lower[j] = 0

        colour_values[colour] = [lower,upper]
        i += 3
    print(colour_values)

def create_masks(height,width):
    masks = {}
    for colour in range(0,5):
        masks[colour] = np.zeros((int(height), int(width), 1), dtype=np.uint8)

    return masks

def update_masks(masks,safety_value):
    masks_detected = []
    for colour in colour_list:
        masks[colour] = cv2.inRange(cv2_image, np.array(colour_values[colour][0]), np.array(colour_values[colour][1]))
        if np.sum(masks[colour]) > safety_value:
            masks_detected.append(colour)

    return masks, masks_detected 

def find_contours(masks, masks_detected):
    contour_dict = {}
    contours_detected = []
    for colour in masks_detected:
        im, contours, hierarchy = cv2.findContours(masks[colour],cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 2:
            contour_dict[colour] = contours
            contours_detected.append(colour)

    return contour_dict, contours_detected

def magnitude(vec):
    return np.sqrt(np.sum(np.square(np.array(vec))))

def find_nav_points(point1, point2):

    fixed_dist = 1
    centre = ((point2[0] + point1[0])/2, (point2[1] + point1[1])/2, (point2[2] + point1[2])/2)

    # vector = np.array([point2[0] - centre[0], point2[1] - centre[1], centre[2]])
    # unit_vector = np.divide(vector, magnitude(vector))

    # normal_unit_vector = np.array([-unit_vector[1], unit_vector[0], unit_vector[2]])

    # nav_point_1 = tuple(np.multiply(normal_unit_vector,-fixed_dist))
    # nav_point_2 = tuple(np.multiply(normal_unit_vector,fixed_dist))
    nav_point_1 = (centre[0], centre[1], centre[2]-fixed_dist)
    nav_point_2 = (centre[0], centre[1], centre[2]+fixed_dist)

    return nav_point_1, centre, nav_point_2

def find_centres(contour_dict, contours_detected):

    centres_dict = {}
    centres_detected = []

    for colour in contours_detected:

        M1 = cv2.moments(contour_dict[colour][0])
        M2 = cv2.moments(contour_dict[colour][1])
        try:
            pix1 = (int(M1["m10"] / M1["m00"]),int(M1["m01"] / M1["m00"]))
            pix2 = (int(M2["m10"] / M2["m00"]),int(M2["m01"] / M2["m00"]))
        except ZeroDivisionError:
            continue
        
        xyz_array = ros_numpy.point_cloud2.pointcloud2_to_array(ros_point_cloud, squeeze=False)
        point1 = ( xyz_array['x'][pix1[1]][pix1[0]], xyz_array['y'][pix1[1]][pix1[0]], xyz_array['z'][pix1[1]][pix1[0]])
        point2 = ( xyz_array['x'][pix2[1]][pix2[0]], xyz_array['y'][pix2[1]][pix2[0]], xyz_array['z'][pix2[1]][pix2[0]])
        centre = ((point2[0] + point1[0])/2, (point2[1] + point1[1])/2, (point2[2] + point1[2])/2)

        if not np.isnan(np.sum(centre)):
            centres_dict[colour] = centre
            centres_detected.append(colour)

    return centres_dict, centres_detected

def transform_to_odom(centres_dict, centres_detected):
    
    pose_dict = {}
    poses_detected = []
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = 'camera_depth_frame'
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 0.0
    pose_stamped.pose.orientation.z = 0.9999997
    pose_stamped.pose.orientation.w = 0.0007963


    for colour in centres_detected:
        try:
            pose_stamped.pose.position.x = centres_dict[colour][0]
            pose_stamped.pose.position.y = centres_dict[colour][1]
            pose_stamped.pose.position.z = centres_dict[colour][2]
            output_pose_stamped = tf_buffer.transform(pose_stamped, 'odom', rospy.Duration(1))

            poses_detected.append(colour)
            output_pose_stamped.pose.position.z = 0
            pose_dict[colour] = output_pose_stamped

        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('waiting for TF..')

    return pose_dict, poses_detected

def visualize_poses(pose_dict, poses_detected):

    marker = Marker()
    marker.header.frame_id = 'odom'
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    
    for colour in poses_detected:
        
        for i in range(0,len(pose_dict[colour])):

            marker.id = colour*10 + i
            marker.pose = pose_dict[colour][i].pose
            pose_visualizer.publish(marker)

def extra_nav_points(pose_dict, poses_detected):
    new_pose_dict = {}
    front_offset = 0.5
    rear_offset = 3

    for colour in poses_detected:
        
        centre = copy.deepcopy(pose_dict[colour])
        nav_1 = copy.deepcopy(pose_dict[colour])
        nav_2 = copy.deepcopy(pose_dict[colour])

        nav_1.pose.position.x -= front_offset 
        nav_2.pose.position.x += rear_offset

        new_pose_dict[colour] = [nav_1, centre, nav_2]

    return new_pose_dict, poses_detected
        
def send_poses(pose_dict, poses_detected):

    msg = poseDict()
    msg.header.frame_id = 'odom'
    msg.header.stamp = rospy.Time.now()
    list_of_gates = []
    list_of_poses = []
    for gate in poses_detected:
        list_of_gates.append(gate)
        for i in range(0,len(pose_dict[gate])):
            list_of_poses.append(pose_dict[gate][i].pose)

    msg.gates = list_of_gates
    msg.gate_poses.poses = list_of_poses
    pose_publisher.publish(msg)

def main(image_topic, depth_topic, dict_topic):

    global bridge, pose_publisher, tf_buffer, pose_visualizer, dict_listener, pose_publisher, dis_list, dis_dict
    
    dis_list = []
    dis_dict = {}
    bridge = CvBridge()

    rospy.init_node('gate_identifier')

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber(image_topic,Image,image_callback)
    rospy.Subscriber(depth_topic,PointCloud2,depth_callback)
    dict_listener = rospy.Subscriber(dict_topic, IntColor,dict_callback)
    pose_publisher = rospy.Publisher('/gate_poses',poseDict,queue_size=10)

    pose_visualizer = rospy.Publisher('/gate_markers',Marker,queue_size=10)

    rospy.loginfo('gate_identifier set up complete.\nimage topic: '+ image_topic +'\ndepth topic: '+ depth_topic + '\nDict topic: '+dict_topic)

    while not 'cv2_image' in globals() and not rospy.is_shutdown():
        rospy.loginfo('waiting for image')

    while not 'ros_point_cloud' in globals() and not rospy.is_shutdown():
        rospy.loginfo('waiting for cloud')

    while not 'colour_values' in globals() and not rospy.is_shutdown():
        rospy.loginfo('waiting for dict')

    masks = create_masks(480,640)

    while not rospy.is_shutdown():
        
        poses_detected = []
        try:
            masks, masks_detected = update_masks(masks, 255000)
        except KeyError:
            continue

        if len(masks_detected) != 0:
            contour_dict, contours_detected = find_contours(masks, masks_detected)
            if len(contours_detected) != 0:
                centres_dict, centres_detected = find_centres(contour_dict, contours_detected)
                if len(centres_detected) != 0:
                    pose_dict, poses_detected = transform_to_odom(centres_dict, centres_detected)
    
        for pose in poses_detected:
            if not pose in dis_list:
                dis_list.append(pose)
            dis_dict[pose] = pose_dict[pose]
        
        if len(dis_list) == 0:
            continue

        rospy.loginfo(str(len(dis_list)) + ' gates found')

        pose_dict, poses_detected = extra_nav_points(dis_dict, dis_list)

        visualize_poses(pose_dict, poses_detected)
        send_poses(pose_dict,poses_detected)

    rospy.loginfo('shutting down node')

if __name__ == '__main__':

    # adjust subscriptions here
    main('/camera/color/image_raw','/camera/depth/points','/ArucoDict')
