#!/usr/bin/python
import sys
import os
import roslib
roslib.load_manifest('mechknownet')
pkg_dir = roslib.packages.get_pkg_dir('mechknownet')
mrcnn_dir = os.path.join(pkg_dir,"script/mask_rcnn")
sys.path.append(mrcnn_dir)

import mask_rcnn_utils as utils
import rospy
import numpy as np
import tensorflow as tf

import skimage
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import message_filters
from mechknownet.srv import *
#import tf as ros_tf

class mask_rcnn_server:
    
    def __init__(self):
        self.drawer_demo = False


        if self.drawer_demo:
            meta_file = mrcnn_dir + "/log/610objects.meta"
            log_file = mrcnn_dir + "/log/610objects"
            self.num_class = 1 + 8
        else:
            meta_file = mrcnn_dir + "/log_daily_new/610objects.meta"
            log_file = mrcnn_dir + "/log_daily_new/610objects"
            self.num_class = 1 + 8
        self.loader = tf.train.import_meta_graph(meta_file)
        self.sess = tf.Session()
        self.loader.restore(self.sess,log_file)


        
        self.mrcnnSrv = rospy.Service("mechknownet_mrcnn", mrcnn, self.srvCb)
        rospy.loginfo("mask_rcnn_server: ready to serve")
        self.bridge = CvBridge()
        

    def srvCb(self,req):
        rospy.loginfo("mask_rcnn_server: got request")
        image = req.Image
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) 
        results = utils.detect(self.sess,[cv_image_rgb],self.num_class)[0]
        cls_ids = results['class_ids']
        scores = results['scores']
        num_detected_objs = len(cls_ids)
        masks = []
        if num_detected_objs > 0:
            print results['class_ids']
            for i in range(num_detected_objs):
                mask = results['masks'][:,:,i].astype(np.uint8)
                mask = mask * 255    
                print mask.shape
                maskImg = self.bridge.cv2_to_imgmsg(mask,"mono8")
                masks.append(maskImg)
                color_mask = cv2.bitwise_and(cv_image_rgb,cv_image_rgb,mask = mask)                
        resp = mrcnnResponse()
        resp.Masks = masks
        resp.Class_ids = cls_ids
        resp.Scores = scores
        return resp

def main(args):
  rospy.init_node('mrcnn_server', anonymous=True)
  mrcnn_server= mask_rcnn_server()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)




'''
if __name__ == '__main__':
    loader = tf.train.import_meta_graph("./mask_rcnn/log/610objects.meta")
    sess = tf.Session()
    loader.restore(sess,"./mask_rcnn/log/610objects")
    #image = skimage.io.imread("0001.jpg")
    num_class = 1+2
    utils.detect_and_color_splash(sess,image,num_class)
'''
