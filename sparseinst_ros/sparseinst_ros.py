#!/usr/bin/env python3
import argparse
import glob
import multiprocessing as mp
import os
import time
import cv2
import tqdm
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from detectron2.config import get_cfg
from detectron2.data.detection_utils import read_image
from detectron2.utils.logger import setup_logger

from sparseinst import VisualizationDemo, add_sparse_inst_config


# constants
# WINDOW_NAME = "COCO detections"

class Nodo:
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(40)
        self.model_path = rospy.get_param('~model_path')
        self.config_path = rospy.get_param('~config_path')
        cfg = self.setup_cfg()
        self.latest_data = None

        self.demo = VisualizationDemo(cfg)

        # Publishers
        self.pub = rospy.Publisher('/sparseinst_seg', Image,queue_size=30)

        # Subscribers
        rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

    def callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        self.image = cv2.rotate(self.image, cv2.ROTATE_90_CLOCKWISE)
        self.latest_data = msg
    
    def setup_cfg(self):
        rospy.loginfo('setup config')
        cfg = get_cfg()
        add_sparse_inst_config(cfg)
        cfg.merge_from_file(self.config_path)
        opts_param = ['MODEL.WEIGHTS', self.model_path, 'INPUT.MIN_SIZE_TEST', '480']
        cfg.merge_from_list(opts_param)
        cfg.MODEL.RETINANET.SCORE_THRESH_TEST = 0.5
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5
        cfg.MODEL.PANOPTIC_FPN.COMBINE.INSTANCES_CONFIDENCE_THRESH = 0.5
        cfg.freeze()
        return cfg


    def start(self):
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            if self.image is not None:
                predictions, visualized_output = self.demo.run_on_image(self.image, 0.5)
                final_image = cv2.rotate(visualized_output.get_image(),cv2.ROTATE_90_COUNTERCLOCKWISE)
                final_image_message = self.br.cv2_to_imgmsg(final_image,encoding="bgr8")
                final_image_message.header = self.latest_data.header
                self.pub.publish(final_image_message)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("sparseinst_ros", anonymous=True)
    my_node = Nodo()
    my_node.start()