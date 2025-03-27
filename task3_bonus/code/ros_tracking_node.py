#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String  # Import String message type

# Initialize ROS Node
rospy.init_node('ros_bbox_and_video_publisher', anonymous=True)

# Initialize CvBridge
bridge = CvBridge()

# ROS Publishers
viz_pub = rospy.Publisher('/me5413/viz_output', Image, queue_size=10)
groundtruth_pub = rospy.Publisher('/me5413/groundtruth', Detection2DArray, queue_size=10)
tracked_pub = rospy.Publisher('/me5413/track', Detection2DArray, queue_size=10)
nusnet_pub = rospy.Publisher('/me5413/nusnetID', String, queue_size=1)  # NUSNET ID publisher

NUSNET_ID = "E1373531"  

# File Paths
ground_truth_file = "task1_tracking/data/seq3/groundtruth.txt"
track_results_file = "task1_tracking/results/3_improved/trackresults_improved_seq3.txt"
result_images_folder = "task1_tracking/data/seq3/results3"  

# Load Bounding Box Data
with open(ground_truth_file, "r") as file:
    ground_truth_boxes = [list(map(int, line.strip().split(","))) for line in file.readlines()]

with open(track_results_file, "r") as file:
    tracked_boxes = [list(map(int, line.strip().split(","))) for line in file.readlines()]

# Sort images by filename (assuming they are named sequentially)
image_files = sorted([os.path.join(result_images_folder, f) for f in os.listdir(result_images_folder) if f.endswith(".jpg")])

frame_idx = 0
rate = rospy.Rate(10)  # 10 FPS video playback


def publish_data():
    global frame_idx



    while not rospy.is_shutdown():
        
        nusnet_msg = String()
        nusnet_msg.data = NUSNET_ID
        nusnet_pub.publish(nusnet_msg)
        rospy.loginfo(f"Published NUSNET ID: {NUSNET_ID}")
        # Loop back to the first frame when reaching the end
        if frame_idx >= min(len(ground_truth_boxes), len(tracked_boxes), len(image_files)):
            rospy.logwarn("Reached the last frame. Restarting from the first frame...")
            frame_idx = 0  # Reset to the first frame

     
        gt_msg_array = Detection2DArray()
        gt_bbox = ground_truth_boxes[frame_idx]
        gt_msg = Detection2D()
        gt_msg.center = Pose2D(x=gt_bbox[0] + gt_bbox[2] / 2, y=gt_bbox[1] + gt_bbox[3] / 2, theta=0)  # Center of the bounding box
        gt_msg.size_x = gt_bbox[2]  # Width
        gt_msg.size_y = gt_bbox[3]  # Height
        gt_msg_array.detections.append(gt_msg)
        groundtruth_pub.publish(gt_msg_array)

    
        tr_msg_array = Detection2DArray()
        tr_bbox = tracked_boxes[frame_idx]
        tr_msg = Detection2D()
        tr_msg.center = Pose2D(x=tr_bbox[0] + tr_bbox[2] / 2, y=tr_bbox[1] + tr_bbox[3] / 2, theta=0)  # Center of the bounding box
        tr_msg.size_x = tr_bbox[2]  # Width
        tr_msg.size_y = tr_bbox[3]  # Height
        tr_msg_array.detections.append(tr_msg)
        tracked_pub.publish(tr_msg_array)

        rospy.loginfo(f"Frame {frame_idx}: GT {gt_bbox} | TR {tr_bbox}")

    
        image_path = image_files[frame_idx]
        cv_image = cv2.imread(image_path)

        if cv_image is not None:
            try:
                ros_image_msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")
                viz_pub.publish(ros_image_msg)
            except CvBridgeError as e:
                rospy.logerr(f"CV Bridge Error: {e}")

        frame_idx += 1
        rate.sleep()


if __name__ == '__main__':
    rospy.loginfo("ROS Bounding Box & Video Publisher Started")
    try:
        publish_data()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Bounding Box & Video Publisher Stopped")




