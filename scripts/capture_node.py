#!/usr/bin/env python3

# imports
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String # To pass string messages
from cv_bridge import CvBridge
import os # Misc. operating system interface functions
import time # Python timing functions
import cv2 # OpenCV
import tqdm # Prints a progress bar on console
from pathlib import Path # To find the "home" directory location
import shutil # High level folder operation tool


class DatasetCaptureNode:
    def __init__(self):
        
        
        # Commandline parameters
        # Initialize ROS
        rospy.init_node('depth_to_grayscale_node', anonymous=True)
        rospy.set_param("r0_cam0", "NULL")
        rospy.set_param("r0_depth0", "NULL")
        rospy.set_param("r1_cam0", "NULL")
        rospy.set_param("r1_depth0", "NULL")
        
        self.br = CvBridge()
        
        # my_param_value = rospy.get_param('~my_param', default_value)
        # Parse topic names
        self.robot0_cam0 = rospy.get_param("~r0_cam0")
        self.robot0_depth0 = rospy.get_param("~r0_depth0")
        self.robot1_cam0 = rospy.get_param("~r1_cam0")
        self.robot1_depth0 = rospy.get_param("~r1_depth0")
        
        print()
        print(f"self.robot0_cam0 : {self.robot0_cam0}")
        print(f"self.robot0_depth0: {self.robot0_depth0}")
        print(f"self.robot1_cam0 : {self.robot1_cam0}")
        print(f"self.robot1_depth0: {self.robot1_depth0}")
        print()

        #self.robot0_cam0_sub_ = rospy.Subscriber('robot0_cam0', Image, self.robot0_cam0_callback)
        # self.robot0_depth0_sub_ = rospy.Subscriber('robot0_depth0', Image, self.robot0_depth0_callback)
        
        # Setup directories
        dataset_dir = str(Path.cwd()) + "/dataset"
        self.create_folder_once(dataset_dir)


    def robot0_cam0_callback(self):
        """rgb image callback for robot0"""
        # Initialize work variables
        rgb_img = None

    def robot0_depth0_callback(self):
        pass

    def create_folder_once(self, folder_name_with_path):
        """Creates folder if it does not exsist."""
        # Make sure to pass the full folder name prior to executing this function
        if not os.path.exists(folder_name_with_path):
            os.makedirs(folder_name_with_path)
        else:
            pass

    def delete_and_make_folder(self,folder_name_with_path):
        """Recreates a folder everytime this function is invoked."""

        if not os.path.exists(folder_name_with_path):
            os.makedirs(folder_name_with_path)
        else:
            # First delete the exsisting folder and then recreate a fresh one
            shutil.rmtree(folder_name_with_path)
            os.makedirs(folder_name_with_path)

    def ros_timestamp_to_int_timestamp(self,ros_msg):
        """Converts ros time into long integer timestamp string."""
        bx = None
        bx = int(ros_msg.header.stamp.to_time() * 1000) # sec.nanosec in float to long integer
        return bx

    def run(self):
        """Method to keep ROS node alive, blocking loop."""
        rospy.spin()

# Main entry point of the script
if __name__ == '__main__':
    try:
        node = DatasetCaptureNode()
        # node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException Detected, terminating node!")
        rospy.signal_shutdown("Shutdown requested by user")