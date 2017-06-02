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
    def __init__(self,kernel,dest_shape, init_closest, value=0):
        if dest_shape[1] % 2 != 0 :
            print("\n\n\n\nfilter.Convolution.__init__ : Warning : dest_shape[1] should be even ! \n\n\n\n")
        mask = np.zeros(dest_shape, np.float64)
        mask[...] = value
        
        # let us center the kernel first
        
        m_center_h = dest_shape[0]   //2
        k_center_h = kernel.shape[0] //2
        m_min_h    = max(m_center_h - k_center_h, 0)
        k_min_h    = max(k_center_h - m_center_h, 0)
        size_h     = min(kernel.shape[0], dest_shape[0])
        
        m_center_w = dest_shape[1]   //2
        k_center_w = kernel.shape[1] //2
        m_min_w    = max(m_center_w - k_center_w, 0)
        k_min_w    = max(k_center_w - m_center_w, 0)
        size_w     = min(kernel.shape[1], dest_shape[1])

        mask[m_min_h:m_min_h+size_h, m_min_w:m_min_w+size_w] = kernel[k_min_h:k_min_h+size_h, k_min_w:k_min_w+size_w]

        if init_closest :
            # Let us pad

            if m_min_w != 0 :
                for h in range(m_min_h, m_min_h+size_h) :
                    mask[h, 0:m_min_w] = mask[h,m_min_w]
            if m_min_h != 0 :
                for w in range(m_min_w, m_min_w+size_w) :
                    mask[0:m_min_h, w] = mask[m_min_h, w]
            if m_min_w + size_w < dest_shape[1]:
                for h in range(m_min_h, m_min_h+size_h) :
                    mask[h, m_min_w+size_w:] = mask[h,m_min_w+size_w-1]
            if m_min_h + size_h < dest_shape[0]:
                for w in range(m_min_w, m_min_w+size_w) :
                    mask[m_min_h+size_h:, w] = mask[m_min_h+size_h-1, w]
            if m_min_w != 0 and m_min_h != 0:
                mask[0:m_min_h, 0:m_min_w] =  mask[m_min_h,m_min_w]
            if m_min_w != 0 and m_min_h + size_h < dest_shape[0]:
                mask[m_min_h+size_h:, 0:m_min_w] =  mask[m_min_h+size_h-1,m_min_w]
            if m_min_w + size_w < dest_shape[1] and m_min_h != 0:
                mask[0:m_min_h, m_min_w+size_w:] =  mask[m_min_h,m_min_w+size_w-1]
            if m_min_w + size_w < dest_shape[1] and m_min_h + size_h < dest_shape[0]:
                mask[m_min_h+size_h:, m_min_w+size_w:] =  mask[m_min_h+size_h-1,m_min_w+size_w-1]
        
        self.mask     = np.roll(np.roll(mask,-m_center_w, axis=1),-m_center_h, axis=0)
        self.fft_mask = np.fft.rfft2(self.mask)

    def __call__(self,image):
        return np.fft.irfft2(np.fft.rfft2(image)*self.fft_mask)




class Locus:

    def make_conv(self) :
        if self.kernel is not None and self.in_shape is not None :
            self.conv = Convolution(self.kernel, self.in_shape, True)
            
    def check_in(self, image):
        make = False
        if image.shape is not self.in_shape :
            self.in_shape = image.shape
            self.img_coef      = 1.0/image.shape[1]
            make = True
        if self.kp_rad is not self.k_kp_rad or self.ki_ratio is not self.k_ki_ratio:
            self.k_kp_rad = self.kp_rad
            self.k_ki_ratio = self.ki_ratio
            make = True
        if make :
            self.make_kernel()
            self.make_conv()
            
            
        
    def on_image(self, ros_data):
        # image in
        compressed_in = np.fromstring(ros_data.data, np.uint8)
        image_in      = cv2.imdecode(compressed_in, cv2.IMREAD_GRAYSCALE) # in [0,255]
        self.check_in(image_in)
        image_in = ((np.clip(image_in, self.min_input,self.max_input) - self.min_input)*self.inv_range_input).astype(np.uint8)
        image_out = cv2.cvtColor(image_in, cv2.COLOR_GRAY2RGB)
        if self.conv is not None and (image_in.max() > 0) :
            conv   = self.conv(image_in.astype(np.float32))
            argmax = np.unravel_index(np.argmax(conv), conv.shape)
            
            current = rospy.Time.now()
            frozen = (current - self.last_shift).to_sec() < self.shift_freeze_duration
            color = (0,255,0)
            if frozen :
                color = (0,0,255)
            cv2.circle(image_out, (argmax[1],argmax[0]) , 5, color, -1)
            
            if not frozen :
                msg = Point()
                msg.z = 0
                msg.x = argmax[1]*self.img_coef-.5
                msg.y = argmax[0]*self.img_coef-.5
                self.shift.publish(msg)
                if msg.x*msg.x + msg.y*msg.y > self.max_small_shift_radius_2 :
                    self.last_shift = current
        # image out
        msg              = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format       = "jpeg"
        msg.data         = np.array(cv2.imencode('.jpg', image_out)[1]).tostring()
        self.pub.publish(msg)

        
    def make_kernel(self):
        ksize = 100
        if self.in_shape is not None :
            ksize = min(self.in_shape[0], self.in_shape[1])-10
        half_ksize = ksize//2
        self.kernel = np.zeros((ksize,ksize), dtype=np.float32)
        for h in range(ksize) :
            dh = (h-half_ksize)**2
            for w in range(ksize) :
                r  = math.sqrt(dh + (w-half_ksize)**2)/ksize
                rr = r/self.k_kp_rad
                if r < self.k_kp_rad :
                    self.kernel[h,w] = 1
                else :
                    self.kernel[h,w] = - (self.k_ki_ratio*rr)**2
        
    def on_reconf(self, config, level):
        self.max_input   = config['max_input']
        self.min_input   = config['min_input']
        if self.min_input+5 > self.max_input :
            self.min_input = 0
            self.max_input = 255
        self.inv_range_input = 255.0/(self.max_input - self.min_input)
        self.shift_freeze_duration = config['shift_freeze']
        self.max_small_shift_radius_2 = config['shift_rad']**2
        self.kp_rad = config['kp_rad']
        self.ki_ratio = config['ki_ratio']
        return config
    
    def __init__(self):
        self.in_shape      = None
        self.img_coef      = 0
        self.conv          = None
        self.kernel = None
        self.kp_rad = None
        self.ki_ratio = None
        self.k_kp_rad = 0
        self.k_ki_ratio = 0
        
        self.min_input                =  0
        self.max_input                = 20
        self.inv_range_input          = 255.0/(self.max_input - self.min_input)
        self.shift_freeze_duration    = .3;
        self.max_small_shift_radius_2 = .1*.1;
        self.last_shift               = rospy.Time.now()
        #### ROS ####
        self.config_srv = dynamic_reconfigure.server.Server(LocusParamConfig, self.on_reconf)
        self.sub        = rospy.Subscriber("in/compressed",  CompressedImage, self.on_image,  queue_size = 1)
        self.pub        = rospy.Publisher ("out/compressed", CompressedImage,                 queue_size = 1)
        self.shift      = rospy.Publisher ("shift",          Point,                           queue_size = 1)
        
    
rospy.init_node('locus_node', anonymous=True)
locus = Locus()
rospy.spin()


