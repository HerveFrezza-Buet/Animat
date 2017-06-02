#! /usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np
import cv2
import roslib
import rospy
from sensor_msgs.msg   import CompressedImage
from std_msgs.msg      import Float32
from std_msgs.msg      import String
from geometry_msgs.msg import Point
from color_filter.msg  import HSVParams
import dynamic_reconfigure.server
from animat.cfg import VisionParamConfig

class Vision:
    
    def __init__(self):
        self.focus_near         = .3  
        self.focus_excentricity = .15
        self.center_point = Point(0,0,0)
        self.center_intensity  = 0
        self.focus_max_speed   = .02
        self.focus_time        = rospy.Time.now()
        self.focus             = None
        self.stable_focus      = False
        self.focus_area        = "focus_unstable"
        self.config_srv = dynamic_reconfigure.server.Server(VisionParamConfig, self.on_reconf)
        self.raw_sub           = rospy.Subscriber("raw/compressed", CompressedImage, self.on_raw,     queue_size = 1)
        self.in_sub            = rospy.Subscriber("in/compressed",  CompressedImage, self.on_image,   queue_size = 1)
        self.gaze_pub          = rospy.Publisher ("gaze/compressed", CompressedImage,                  queue_size = 1)
        self.focus_sub         = rospy.Subscriber("focus",          Point,           self.on_focus,   queue_size = 1)
        self.cmd_sub           = rospy.Subscriber("command",        String,          self.on_command, queue_size = 1)
        self.lookat_pub        = rospy.Publisher ("lookat",         Point,                            queue_size = 1)
        self.fovea_pub         = rospy.Publisher ("fovea",          Float32,                          queue_size = 1)
        self.event_pub         = rospy.Publisher ("events",         String,                           queue_size = 1)
        self.hsv_pub           = rospy.Publisher ("hsv",            HSVParams,                        queue_size = 1)
    
    def on_reconf(self, config, level):
        self.focus_near         = config['near_limit']  
        self.focus_excentricity = config['center_limit']
        self.focus_max_speed    = config['focus_max_speed']  
        return config
    
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
        self.stable_focus = False
        current = rospy.Time.now()
        if self.focus is not None :
            speed = math.sqrt((focus.x - self.focus.x)**2 + (focus.y - self.focus.y)**2) / (current - self.focus_time).to_sec()
            self.stable_focus = speed < self.focus_max_speed
        self.focus = focus
        self.focus_time = current
        area = None
        if not self.stable_focus :
            area = "focus_unstable"
        else :
            if focus.y > self.focus_near :
                area = "focus_near"
            else :
                area = "focus_far"
            if focus.x < -self.focus_excentricity :
                area += "_left"
            elif focus.x > self.focus_excentricity :
                area += "_right"
            else :
                area += "_center"
        if area != self.focus_area :
            self.event_pub.publish(area)
        self.focus_area = area
        
    def on_raw(self, ros_data):
        if self.gaze_pub.get_num_connections() > 0 :
            #### From ros message to cv image ####
            compressed_in = np.fromstring(ros_data.data, np.uint8)
            image_in      = cv2.imdecode(compressed_in, cv2.IMREAD_COLOR)
            width         = image_in.shape[1]
            height        = image_in.shape[0]

            darken = .5
            
            near = int((1 - self.focus_near)*height)
            image_in[:near,:] = (darken*image_in[:near,:]).astype(np.uint8)

            wmin = int((.5-self.focus_excentricity)*width)
            wmax = int((.5+self.focus_excentricity)*width)
            image_in[:,:wmin] = (darken*image_in[:,:wmin]).astype(np.uint8)
            image_in[:,wmax:] = (darken*image_in[:,wmax:]).astype(np.uint8)

            fx = int(width*(.5 + self.focus.x)+.5)
            fy = int(height*(.5 + self.focus.y)+.5)
            color = (0,0,255)
            if self.stable_focus :
                color = (0,255,0)
            cv2.circle(image_in, (fx, fy) , 20, color,  3)
            cv2.line(image_in, (fx-30, fy) , (fx+30, fy), color,  3)
            cv2.line(image_in, (fx, fy-30) , (fx, fy+30), color,  3)
            
            # image out
            msg              = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format       = "jpeg"
            msg.data         = np.array(cv2.imencode('.jpg', image_in)[1]).tostring()
            self.gaze_pub.publish(msg)


    def on_image(self, ros_data):

        #### From ros message to cv image ####
        compressed_in = np.fromstring(ros_data.data, np.uint8)
        image_in      = cv2.imdecode(compressed_in, cv2.IMREAD_GRAYSCALE)
        width         = image_in.shape[1]
        height        = image_in.shape[0]
        
        """#### Processing ####
        w_2 = int(width*.5)
        h_2 = int(height*.5)
        center = image_in[h_2 - param_lgn_center_radius : h_2 + param_lgn_center_radius+1,
                          w_2 - param_lgn_center_radius : w_2 + param_lgn_center_radius+1]
        self.center_intensity = np.average(center)/255.0
        self.fovea_pub.publish(self.center_intensity)"""

    
if __name__ == '__main__':
    rospy.init_node('vision', anonymous=True)
    try:
        vision = Vision()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision"
