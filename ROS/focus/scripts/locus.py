#! /usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import roslib
import rospy
import numpy as np
import dynamic_reconfigure.server
from focus.cfg import LocusParam
from sensor_msgs.msg import CompressedImage

class Convolution:
    def __init__(self,kernel,dest_shape):
        if dest_shape[1] % 2 != 0 :
            print("\n\n\n\nfilter.Convolution.__init__ : Warning : dest_shape[1] should be even ! \n\n\n\n")
        mask = np.zeros(dest_shape, np.float32)
        mask[0:kernel.shape[0],0:kernel.shape[1]] = kernel
        #self.mask     = np.roll(np.roll(mask,-(kernel.shape[1]//2-1), axis=1),-(kernel.shape[0]//2-1), axis=0)
        self.mask     = np.roll(np.roll(mask,-(kernel.shape[1]//2), axis=1),-(kernel.shape[0]//2), axis=0)
        self.fft_mask = np.fft.rfft2(self.mask)

    def __call__(self,image):
        return np.fft.irfft2(np.fft.rfft2(image)*self.fft_mask)

    
class Locus:
    def __init__(self):
        self.config_srv = dynamic_reconfigure.server.Server(LocusParam, self.on_reconf)
        self.sub        = rospy.Subscriber("/image_in/compressed",  CompressedImage, self.on_image,  queue_size = 1)
        self.pub        = rospy.Publisher ("/image_out/compressed", CompressedImage,                 queue_size = 1)
        self.img_width  = 0
        self.img_height = 0
        
    def on_image(self, ros_data):
        # image in
        compressed_in = np.fromstring(ros_data.data, np.uint8)
        image_in      = cv2.imdecode(compressed_in, cv2.IMREAD_GRAYSCALE)
        width         = image_in.shape[1]
        height        = image_in.shape[0]

        # image out
        msg              = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format       = "jpeg"
        msg.data         = np.array(cv2.imencode('.jpg', image_out)[1]).tostring()
        self.pub.publish(msg)

        
    def on_reconf(self, config, level):
        print(config['inh_width'])
        return config
        
    
rospy.init_node('locus_node', anonymous=True)
locus = Locus()

