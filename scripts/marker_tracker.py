#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

HORIZONTAL_ANGLE = np.pi * 70.42 / 180 # HFOV of camera
IMAGE_WIDTH = 640 # image width after YOLO processing
ERROR = 0.01 # error as percentage of screen

# globals
past_markers = []
current_yaw = 0
current_position_x = 0
current_position_y = 0

def is_in_view(marker):
    global HORIZONTAL_ANGLE
    x_diff = marker.pose.position.x - current_position_x
    y_diff = marker.pose.position.y - current_position_y
    angle = math.atan(y_diff / x_diff)
    angle_min = -HORIZONTAL_ANGLE / 2 * math.pi / 180
    angle_max = HORIZONTAL_ANGLE / 2 * math.pi / 180
    return angle <= angle_max and angle >= angle_min

def get_distance(marker1, marker2):
    x_diff = marker1.pose.position.x - marker2.pose.position.x
    y_diff = marker1.pose.position.y - marker2.pose.position.y
    z_diff = marker1.pose.position.z - marker2.pose.position.z
    return math.sqrt(x_diff**2 + y_diff**2 + z_diff**2)

def check_if_new(marker_new):
    global IMAGE_WIDTH
    global ERROR
    for marker_old in past_markers:
        if get_distance(marker_new, marker_old) < IMAGE_WIDTH * ERROR:
            return marker_old
    return None

def get_missing(new_markers):
    missing_markers = []
    for marker in past_markers:
        if is_in_view(marker) and not (marker in new_markers):
            missing_markers.append(marker)
    return missing_markers

def find_movement(new_markers):
    global past_markers
    display_markers = MarkerArray()
    id = 0
    for marker in new_markers.markers:
        new_marker = check_if_new(marker)
        if new_marker == None:
            marker.id = id
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 1
        else:
            marker.id = id
            marker.color.r = 1
            marker.color.g = 0
            marker.color.b = 1
            past_markers.remove(new_marker)
        id += 1
        display_markers.markers.append(marker)
    
    missing_markers = get_missing(display_markers.markers)
    for marker in missing_markers:
        marker.id = id
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        display_markers.markers.append(marker)
        id += 1
    
    for marker in past_markers:
        if not is_in_view(marker):
            marker.id = id
            marker.color.r = 1
            marker.color.g = 0
            marker.color.b = 1
            display_markers.markers.append(marker)
            id += 1

    rospy.loginfo(display_markers)
    pub.publish(display_markers)

    past_markers = []
    
    for marker in display_markers.markers:
        if marker not in missing_markers:
            past_markers.append(marker)

def odom_recieved(odom):
    global current_position_x
    global current_position_y
    global current_yaw
    orientation_q = odom.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    current_yaw = yaw
    current_position_x = odom.pose.pose.position.x
    current_position_y = odom.pose.pose.position.y

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('marker_tracking', MarkerArray, queue_size=20)
        rospy.init_node('marker_tracking', anonymous=True)
        rospy.Subscriber('depth_estimation', MarkerArray, find_movement)
        rospy.Subscriber("odom", Odometry, odom_recieved)
        rospy.loginfo("Waiting for messages...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
