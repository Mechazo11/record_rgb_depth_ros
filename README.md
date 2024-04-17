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

* Call signature

1. KUKA robot only
```rosrun dataset_capture capture_node.py r0_cam0:="/robot_0/front_rgbd_camera/rgb/image_rect_color/compressed"```