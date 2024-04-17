## dataset_capture

Python node that takes CompressedDepth and compressed rgb image and saves them directly in RBKAIROS dataset format.

Topic names [updated 04/15/2024] \
agent0 \
cam0 --> ```/robot_0/front_rgbd_camera/rgb/image_rect_color/compressed``` \

depth0 --> ```/robot_0/front_rgbd_camera/depth_registered/image_raw/compressedDepth```

agent1 \
cam0 --> ```/robot_1/front_rgbd_camera/rgb/image_rect_color/compressed``` \

depth0 --> ```/robot_1/front_rgbd_camera/depth_registered/image_raw/compressedDepth```

* ```rosrun my_package my_node.py _one_topic:="/topic1" _another_topic:="/topic2" __name:="my_node_name"```


#### NOTES

* Depth image from compressedDepth: https://answers.ros.org/question/249775/display-compresseddepth-image-python-cv2/

* https://gist.github.com/awesomebytes/958a5ef9e63821a28dc05775840c34d9

* Call signature

1. KUKA robot only
```
rosrun dataset_capture capture_node.py _r0_cam0:="/robot_0/front_rgbd_camera/rgb/image_rect_color/compressed" _r0_depth0:="/robot_0/front_rgbd_camera/depth/image_rect/compressedDepth" _r1_cam0:="/robot_1/front_rgbd_camera/rgb/image_rect_color/compressed" _r1_depth0:="/robot_1/front_rgbd_camera/depth/image_rect/compressedDepth"

```

```
rosrun dataset_capture capture_node.py _r0_cam0:="/robot_0/front_rgbd_camera/rgb/image_rect_color/compressed" _r0_depth0:="/robot_0/front_rgbd_camera/depth_registered/image_raw/compressedDepth" _r1_cam0:="FOO" _r1_depth0:="BAR"
```