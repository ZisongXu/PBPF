

import os
import sys
import yaml
from rosbag.bag import Bag
import cv2

import roslib;   #roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
import numpy as np
import argparse
import os
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

class ImageCreator():
    def __init__(self, bagfile, rgbpath, depthpath, rgbstamp, depthstamp):
        self.bridge = CvBridge()
        with rosbag.Bag(bagfile, 'r') as bag:
            count_rgb = 0
            count_depth = 0
            for topic,msg,t in bag.read_messages():
                if topic == "/camera/color/image_raw": #图像的topic；
                    count_rgb = count_rgb + 1
                    cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                    image_name = f"{count_rgb:04d}.png"
                    cv2.waitKey(1);
                    cv2.imwrite(rgbpath + image_name, cv_image)  #保存；
                    print("count_rgb:", count_rgb)
                elif topic == "/camera/aligned_depth_to_color/image_raw": #图像的topic；
                    count_depth = count_depth + 1
                    cv_image = self.bridge.imgmsg_to_cv2(msg,"16UC1")
                    image_name = f"{count_depth:04d}.png"
                    cv2.imwrite(depthpath + image_name, cv_image) 
                    print("count_depth:", count_depth)



rosbag_file_path = os.path.expanduser("~/pyvkdepth/rosbag/")
rosbag_file_name = "2_scene1_crackersoup1.bag"
save_file_path = os.path.expanduser("~/")
if __name__ == '__main__':
    ImageCreator(rosbag_file_path+rosbag_file_name, save_file_path+"rgb/", save_file_path+"depth/", 1, 1)

