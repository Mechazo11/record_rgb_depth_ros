#!/usr/bin/env python3

# imports
import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String # To pass string messages
from cv_bridge import CvBridge, CvBridgeError
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
        rospy.init_node('capture_node', anonymous=True)
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

        # Setup subscribers
        self.robot0_cam0_sub_ = rospy.Subscriber(self.robot0_cam0, CompressedImage, self.robot0_cam0_callback)
        # self.robot0_depth0_sub_ = rospy.Subscriber('robot0_depth0', Image, self.robot0_depth0_callback)
        
        # Setup directories
        dataset_dir = str(Path.cwd()) + "/src/dataset_capture/dataset/"
        print(dataset_dir)
        # self.create_folder_once(dataset_dir)

        # create the two folders as needed
        self.r0_cam0_dir = dataset_dir + "robot0/cam0/data/"
        self.r0_depth0_dir = dataset_dir + "robot0/depth0/data/"
        self.delete_and_make_folder(self.r0_cam0_dir)
        self.delete_and_make_folder(self.r0_depth0_dir)

        print("Initialization complete!!")

    def robot0_cam0_callback(self,data):
        """rgb image callback for robot0"""
        # Initialize work variables
        rgb_img = None
        rgb_timestamp = None
        try:
            rgb_img = self.br.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # Convert the timestamp to a Python datetime object (if needed)
        rgb_timestamp = int(rospy.Time.to_sec(data.header.stamp) * 1000)
        print("Image timestamp:", rgb_timestamp)

        # Display the image
        cv2.imshow("Robot0 Camera 0", rgb_img)
        cv2.waitKey(1)  # Wait for a key press (1 millisecond)
        

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

    # def run(self):
    #     """Method to keep ROS node alive, blocking loop."""
    #     print("Running")
    #     rospy.spin()

# Main entry point of the script
if __name__ == '__main__':
    node = DatasetCaptureNode()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException Detected, terminating node!")
        rospy.signal_shutdown("Shutdown requested by user")