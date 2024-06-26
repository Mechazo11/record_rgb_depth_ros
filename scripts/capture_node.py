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
import numpy as np
import struct

class DatasetCaptureNode:
    def __init__(self):
        
        # Commandline parameters
        # Initialize ROS
        rospy.init_node('capture_node', anonymous=True)
        rospy.set_param("agent", "NULL")
        rospy.set_param("cam0", "NULL")
        rospy.set_param("depth0", "NULL")
        
        # rospy.set_param("save_data", True)
        
        self.br = CvBridge()
        
        # Parse command-line values
        self.agent = rospy.get_param("~agent")
        self.cam0 = rospy.get_param("~cam0")
        self.depth0 = rospy.get_param("~depth0")
        
        #self.robot1_cam0 = rospy.get_param("~r1_cam0")
        #self.robot1_depth0 = rospy.get_param("~r1_depth0")
        #self.save_data = rospy.get_param("~save_data")
        self.save_data = True
        
        print()
        print(f"self.cam0 : {self.cam0}")
        print(f"self.depth0: {self.depth0}")
        print(f"self.agent: {self.agent}")
        print()

        # Setup subscribers
        self.cam0_sub_ = rospy.Subscriber(self.cam0, CompressedImage, self.cam0_callback)
        self.depth0_sub_ = rospy.Subscriber(self.depth0, CompressedImage, self.depth0_callback)
        
        # Setup directories
        dataset_dir = str(Path.home()) + "/DATASETS/data_capture/"
        
        if not os.path.exists(dataset_dir):
            error_msg = "Error: Dataset directory does not exist: {}".format(dataset_dir)
            raise FileNotFoundError(error_msg)
            # You can customize the error type and message as needed
        else:
            print("Dataset directory exists:", dataset_dir)
        
        # create the two folders as needed
        self.cam0_dir = dataset_dir + self.agent + "/cam0/data/"
        self.depth0_dir = dataset_dir + self.agent +"/depth0/data/"
        
        print()
        print(f"self.cam0_dir : {self.cam0_dir }")
        print(f"self.depth0_dir: {self.depth0_dir}")
        print()

        self.delete_and_make_folder(self.cam0_dir)
        self.delete_and_make_folder(self.depth0_dir)

        print("Initialization complete!!")

    def cam0_callback(self,data):
        """rgb image callback for robot0"""
        # Initialize work variables
        rgb_img = None
        rgb_timestamp = None
        im_path = None
        try:
            rgb_img = self.br.compressed_imgmsg_to_cv2(data, "bgr8")
            # Convert the timestamp to a Python datetime object (if needed)
            rgb_timestamp = int(rospy.Time.to_sec(data.header.stamp) * 1000)
            # print("Image timestamp:", rgb_timestamp)
            im_path = self.cam0_dir + str(rgb_timestamp)+".png"
            # print(im_path)
        except CvBridgeError as e:
            print(e)

        if self.save_data:
            try:
                cv2.imwrite(im_path,rgb_img)
                pass
            except:  # noqa: E722
                print("Failed to write rgb image for robot0_cam0!!")

        # Display the image
        name_str = self.agent + " cam00"
        cv2.imshow(name_str, rgb_img)
        cv2.waitKey(1)  # Wait for a key press (1 millisecond)
        
    def depth0_callback(self, msg):
        """depth image callback for robot0"""
        # 'msg' as type CompressedImage
        depth_timestamp = int(rospy.Time.to_sec(msg.header.stamp) * 1000)
        #print(depth_timestamp)
        depth_fmt, compr_type = msg.format.split(';')
        # remove white space
        depth_fmt = depth_fmt.strip()
        compr_type = compr_type.strip()
        if compr_type != "compressedDepth":
            raise Exception("Compression type is not 'compressedDepth'."
                            "You probably subscribed to the wrong topic.")

        # remove header from raw data
        depth_header_size = 12
        raw_data = msg.data[depth_header_size:]

        #depth_img_raw = cv2.imdecode(np.fromstring(raw_data, np.uint8), cv2.IMREAD_UNCHANGED)
        # Suggested by numpy
        depth_img_raw = cv2.imdecode(np.frombuffer(raw_data, np.uint8), cv2.IMREAD_UNCHANGED)
        
        if depth_img_raw is None:
            # probably wrong header size
            raise Exception("Could not decode compressed depth image."
                            "You may need to change 'depth_header_size'!")

        if depth_fmt == "16UC1":
            print("Mono16 depth image")
            # TODO write raw depth data
        
        elif depth_fmt == "32FC1":
            # print("Mono32 depth image")
            raw_header = msg.data[:depth_header_size]
            # header: int, float, float
            [compfmt, depthQuantA, depthQuantB] = struct.unpack('iff', raw_header)
            depth_img_scaled = depthQuantA / (depth_img_raw.astype(np.float32)-depthQuantB)
            depth_img_scaled[depth_img_raw==0] = 0  # filter max values

            # depth_img_scaled provides distance in meters as f32
            # for storing it as png, we need to convert it to 16UC1 again (depth in mm)
            depth_img_mm = (depth_img_scaled*1000).astype(np.uint16)
            depth_path = self.depth0_dir + str(depth_timestamp)+".png"
            if self.save_data:
                try:
                    cv2.imwrite(depth_path, depth_img_mm)
                except:
                    print("Failed to write depth image for robot0_depth0!!")

            depth_img_mm_8bit = cv2.normalize(depth_img_mm, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            name_str = self.agent + " depth0"
            cv2.imshow(name_str, depth_img_mm_8bit)
            cv2.waitKey(1)  # Wait for a key press (1 millisecond)
            
        else:
            raise Exception("Depth data is not 8UC1, 16UC1 or 32FC1")

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

    def debug_lock(self):
        """Locks system in an infinite loop when called."""
        print("LOCK")
        while (1):
            pass

# Main entry point of the script
if __name__ == '__main__':
    node = DatasetCaptureNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException Detected, terminating node!")
        rospy.signal_shutdown("Shutdown requested by user")