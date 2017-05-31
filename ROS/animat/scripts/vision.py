#! /usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import roslib
import rospy
from sensor_msgs.msg   import CompressedImage
from std_msgs.msg      import Float32
from std_msgs.msg      import String
from geometry_msgs.msg import Point
from color_filter.msg  import HSVParams

param_lgn_center_radius          = 10
param_lgn_shift_radius           = 0.1
param_lgn_shift_nb_stables       = 5
param_center_intensity_threshold = .5
param_focus_excentricity         = .15
param_focus_near                 = .3

class Vision:
    
    def __init__(self):
        self.center_point = Point(0,0,0)
        self.center_intensity  = 0
        self.stability_counter = 0
        self.stable_focus      = False
        self.focus_area        = "focus_unstable"
        self.img_sub           = rospy.Subscriber("in/compressed",  CompressedImage, self.on_image,   queue_size = 1)
        self.shift_sub         = rospy.Subscriber("shift",          Point,           self.on_shift,   queue_size = 1)
        self.focus_sub         = rospy.Subscriber("focus",          Point,           self.on_focus,   queue_size = 1)
        self.cmd_sub           = rospy.Subscriber("command",        String,          self.on_command, queue_size = 1)
        self.lookat_pub        = rospy.Publisher ("lookat",         Point,                            queue_size = 1)
        self.fovea_pub         = rospy.Publisher ("fovea",          Float32,                          queue_size = 1)
        self.event_pub         = rospy.Publisher ("events",         String,                           queue_size = 1)
        self.hsv_pub           = rospy.Publisher ("hsv",            HSVParams,                        queue_size = 1)
    
    def on_command(self, command):
        if command.data == "red" :
            self.hsv_pub.publish(HSVParams(  5,10,15,110,False))
        elif command.data == "blue" :
            self.hsv_pub.publish(HSVParams(116,10,15,110,False))
        elif command.data == "dontcare" :
            self.hsv_pub.publish(HSVParams(  0,0,10,100,True))
        elif command.data == "reset" :
            self.lookat_pub.publish(self.center_point)
        else :
            pass
            
    def on_focus(self, focus):
        area = None
        if not self.stable_focus :
            area = "focus_unstable"
        else :
            if focus.y > param_focus_near :
                area = "focus_near"
            else :
                area = "focus_far"
            if focus.x < -param_focus_excentricity :
                area += "_left"
            elif focus.x > param_focus_excentricity :
                area += "_right"
            else :
                area += "_center"
        if area != self.focus_area :
            self.event_pub.publish(area)
        self.focus_area = area

        
    def on_shift(self, shift):
        r2 = shift.x*shift.x + shift.y*shift.y
        if r2 < param_lgn_shift_radius*param_lgn_shift_radius and self.center_intensity > param_center_intensity_threshold :
            if self.stability_counter < param_lgn_shift_nb_stables :
                self.stability_counter = self.stability_counter + 1
        else:
            self.stability_counter = 0
        stable = (self.stability_counter == param_lgn_shift_nb_stables)
        
        if stable and not self.stable_focus:
            self.event_pub.publish('fovea_on')
        elif not stable and self.stable_focus:
            self.event_pub.publish('fovea_off')
        self.stable_focus = stable
        
    def on_image(self, ros_data):

        #### From ros message to cv image ####
        compressed_in = np.fromstring(ros_data.data, np.uint8)
        image_in      = cv2.imdecode(compressed_in, cv2.IMREAD_GRAYSCALE)
        width         = image_in.shape[1]
        height        = image_in.shape[0]
        
        #### Processing ####
        w_2 = int(width*.5)
        h_2 = int(height*.5)
        center = image_in[h_2 - param_lgn_center_radius : h_2 + param_lgn_center_radius+1,
                          w_2 - param_lgn_center_radius : w_2 + param_lgn_center_radius+1]
        self.center_intensity = np.average(center)/255.0
        self.fovea_pub.publish(self.center_intensity)

    
if __name__ == '__main__':
    rospy.init_node('vision', anonymous=True)
    try:
        vision = Vision()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision"
