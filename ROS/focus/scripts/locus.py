#! /usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import roslib
import rospy
import math
import numpy as np
import dynamic_reconfigure.server
from focus.cfg import LocusParamConfig
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point

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

    def check_in(self, image):
        if self.kernel is not None and image.shape is not self.in_shape :
            self.in_shape = image.shape
            self.img_coef      = 1.0/image.shape[1]
            self.conv     = Convolution(self.kernel, self.in_shape)
        
    def on_image(self, ros_data):
        # image in
        compressed_in = np.fromstring(ros_data.data, np.uint8)
        image_in      = cv2.imdecode(compressed_in, cv2.IMREAD_GRAYSCALE) # in [0,255]
        self.check_in(image_in)
        image_in = ((image_in.clip(self.min_input,self.max_input) - self.min_input)*self.inv_range_input).astype(np.uint8)
        if self.conv is not None and (image_in.max() > 0) :
            conv   = self.conv(image_in)
            argmax = np.unravel_index(np.argmax(conv), conv.shape)
            img = image_in

            img = conv
            toto = img.min()
            img -= toto
            img *= 1.0/img.max()
            img *= 250.0
            img = img.astype(np.uint8)
            
            
            image_out = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            current = rospy.Time.now()
            frozen = (current - self.last_shift).to_sec() < self.shift_freeze_duration
            color = (0,255,0)
            if frozen :
                color = (0,0,255)
            cv2.circle(image_out, (argmax[1],argmax[0]) , 5, color, -1)
            # image out
            msg              = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format       = "jpeg"
            msg.data         = np.array(cv2.imencode('.jpg', image_out)[1]).tostring()
            self.pub.publish(msg)
            
            if not frozen :
                msg = Point()
                msg.z = 0
                msg.x = argmax[1]*self.img_coef-.5
                msg.y = argmax[0]*self.img_coef-.5
                self.shift.publish(msg)
                if msg.x*msg.x + msg.y*msg.y > self.max_small_shift_radius_2 :
                    self.last_shift = current

        
    def make_kernel(self, kp_rad, ki_ratio):
        ksize = 100
        half_ksize = ksize//2
        self.kernel = np.zeros((ksize,ksize), dtype=np.float)
        for h in range(ksize) :
            dh = (h-half_ksize)**2
            for w  in range(ksize) :
                r  = math.sqrt(dh + (w-half_ksize)**2)/ksize
                if r < kp_rad :
                    self.kernel[h,w] = 2 - r/kp_rad
                else :
                    self.kernel[h,w] = - ki_ratio*r**2
        
    def on_reconf(self, config, level):
        self.max_input   = config['max_input']
        self.min_input   = config['min_input']
        if self.min_input+5 > self.max_input :
            self.min_input = 0
            self.max_input = 255
        self.inv_range_input = 255.0/(self.max_input - self.min_input)
        self.shift_freeze_duration = config['shift_freeze']
        self.max_small_shift_radius_2 = config['shift_rad']**2
        self.make_kernel(config['kp_rad'], config['ki_ratio'])
        return config
    
    def __init__(self):
        self.in_shape      = None
        self.img_coef      = 0
        self.conv          = None
        self.kernel = None
        
        self.min_input                =  0
        self.max_input                = 20
        self.inv_range_input          = 1.0/(self.max_input - self.min_input)
        self.shift_freeze_duration    = .1;
        self.max_small_shift_radius_2 = .2*.2;
        self.last_shift               = rospy.Time.now()
        #### ROS ####
        self.config_srv = dynamic_reconfigure.server.Server(LocusParamConfig, self.on_reconf)
        self.sub        = rospy.Subscriber("in/compressed",  CompressedImage, self.on_image,  queue_size = 1)
        self.pub        = rospy.Publisher ("out/compressed", CompressedImage,                 queue_size = 1)
        self.shift      = rospy.Publisher ("shift",          Point,                           queue_size = 1)
        
    
rospy.init_node('locus_node', anonymous=True)
locus = Locus()
rospy.spin()


