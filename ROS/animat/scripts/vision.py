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
        self.near_limit         = -10
        self.near_below         = True
        self.center_origin      =  10
        self.center_limit       =  10
        self.left_below         = False
        self.keep_focus_centered = False
        self.center_point = Point(0,0,0)
        self.center_intensity  = 0
        self.focus_max_speed   = 3
        self.fovea_radius      = .05
        self.focus_time        = rospy.Time.now()
        self.focus             = Point()
        self.stable_focus      = False
        self.last_fast_focus   = rospy.Time.now()
        self.wait_stable_focus = .5
        self.focus_area        = "focus_unstable"
        self.config_srv = dynamic_reconfigure.server.Server(VisionParamConfig, self.on_reconf)
        self.raw_sub           = rospy.Subscriber("raw/compressed",  CompressedImage, self.on_raw,     queue_size = 1)
        self.in_sub            = rospy.Subscriber("in/compressed",   CompressedImage, self.on_image,   queue_size = 1)
        self.gaze_pub          = rospy.Publisher ("gaze/compressed", CompressedImage,                  queue_size = 1)
        self.focus_sub         = rospy.Subscriber("focus",           Point,           self.on_focus,   queue_size = 1)
        self.cmd_sub           = rospy.Subscriber("command",         String,          self.on_command, queue_size = 1)
        self.set_focus_pub     = rospy.Publisher ("set_focus",       Point,                            queue_size = 1)
        self.fovea_pub         = rospy.Publisher ("fovea",           Float32,                          queue_size = 1)
        self.area_pub          = rospy.Publisher ("focus_area",      String,                           queue_size = 1)
        self.hsv_pub           = rospy.Publisher ("hsv",             HSVParams,                        queue_size = 1)
    
    def on_reconf(self, config, level):
        self.near_limit         = config['near_limit']  
        self.near_below         = config['near_below']  
        self.center_origin        = config['center_origin']
        self.center_limit        = config['center_limit']
        self.left_below         = config['left_below']  
        self.focus_max_speed    = config['focus_max_speed']  
        self.fovea_radius       = config['fovea_radius']
        self.wait_stable_focus  = config['wait_stable']
        self.keep_focus_centered = config['keep_focus_centered']
        return config
    
    def on_command(self, command):
        if command.data == "red" :
            self.hsv_pub.publish(HSVParams(  5,10,15,110,False))
        elif command.data == "blue" :
            self.hsv_pub.publish(HSVParams(116,10,15,110,False))
        elif command.data == "dontcare" :
            self.hsv_pub.publish(HSVParams(  0,0,10,100,True))
        elif command.data == "reset" :
            self.set_focus_pub.publish(self.center_point)
        else :
            pass
            
    def on_focus(self, focus):
        self.stable_focus = False
        current = rospy.Time.now()
        if self.focus is not None :
            speed = math.sqrt((focus.x - self.focus.x)**2 + (focus.y - self.focus.y)**2) / (current - self.focus_time).to_sec()
            if speed > self.focus_max_speed :
                rospy.loginfo("high speed = %f",speed)
                self.last_fast_focus   = rospy.Time.now()
                self.stable_focus = False
            else :
                self.stable_focus = (rospy.Time.now() - self.last_fast_focus).to_sec() > self.wait_stable_focus
        
        self.focus = focus
        self.focus_time = current
        if not self.stable_focus :
            self.focus_area = "focus_unstable"
        else :
            if self.near_below :
                if focus.y < self.near_limit :
                    self.focus_area = "focus_near"
                else :
                    self.focus_area = "focus_far"
            else :
                if focus.y > self.near_limit :
                    self.focus_area = "focus_near"
                else :
                    self.focus_area = "focus_far"
            if self.left_below :
                if focus.x < self.center_origin - self.center_limit :
                    self.focus_area += "_left"
                elif focus.x > self.center_origin + self.center_limit :
                    self.focus_area += "_right"
                else :
                    self.focus_area += "_center"
            else:
                if focus.x < self.center_origin - self.center_limit :
                    self.focus_area += "_right"
                elif focus.x > self.center_origin + self.center_limit :
                    self.focus_area += "_left"
                else :
                    self.focus_area += "_center"
        self.area_pub.publish(self.focus_area)
        
    def on_raw(self, ros_data):
        if self.gaze_pub.get_num_connections() > 0 :
            #### From ros message to cv image ####
            compressed_in = np.fromstring(ros_data.data, np.uint8)
            image_in      = cv2.imdecode(compressed_in, cv2.IMREAD_COLOR)
            width         = image_in.shape[1]
            height        = image_in.shape[0]

            Winw, Winh = 10,10
            Wsize = 110
            Aw, Bw, Cw, Dw = Winw+10, Winw+40, Winw+70, Winw+100
            Ah, Bh, Ch     = Winh+10,          Winh+70, Winh+100
            
            image_in[Winw:Winw+Wsize, Winh:Winh+Wsize, ...] = 0
            cv2.line(image_in, (Aw,Ah), (Dw,Ah), (255,255,255) ,  3)
            cv2.line(image_in, (Dw,Ah), (Dw,Ch), (255,255,255) ,  3)
            cv2.line(image_in, (Dw,Ch), (Aw,Ch), (255,255,255) ,  3)
            cv2.line(image_in, (Aw,Ch), (Aw,Ah), (255,255,255) ,  3)
            cv2.line(image_in, (Bw,Ah), (Bw,Ch), (255,255,255) ,  3)
            cv2.line(image_in, (Cw,Ah), (Cw,Ch), (255,255,255) ,  3)
            cv2.line(image_in, (Aw,Bh), (Dw,Bh), (255,255,255) ,  3)

            if self.focus_area == 'focus_far_left' :
                image_in[Ah:Bh,Aw:Bw,...] = 255
            elif self.focus_area == 'focus_far_center' :
                image_in[Ah:Bh,Bw:Cw,...] = 255
            elif self.focus_area == 'focus_far_right' :
                image_in[Ah:Bh,Cw:Dw,...] = 255
            elif self.focus_area == 'focus_near_left' :
                image_in[Bh:Ch,Aw:Bw,...] = 255
            elif self.focus_area == 'focus_near_center' :
                image_in[Bh:Ch,Bw:Cw,...] = 255
            elif self.focus_area == 'focus_near_right' :
                image_in[Bh:Ch,Cw:Dw,...] = 255
            
            
            
            if self.keep_focus_centered :
                fx = width//2
                fy = height//2
            else:
                fx = int(width*(.5 + self.focus.x)+.5)
                fy = int(height*(.5 + self.focus.y)+.5)
            color = (0,0,255)
            if self.stable_focus :
                color = (0,255,0)
            rad    = 30
            margin = 10
            r  = (int)(rad*self.center_intensity)
            rr = rad+margin
            cv2.circle(image_in, (fx, fy) , rad, color,  3)
            cv2.circle(image_in, (fx, fy) , r, color,  -1)
            cv2.line(image_in, (fx-rr, fy) , (fx+rr, fy), color,  3)
            cv2.line(image_in, (fx, fy-rr) , (fx, fy+rr), color,  3)
            
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
        
        #### Processing ####
        w_2 = int(width*.5)
        h_2 = int(height*.5)
        rad = int(self.fovea_radius*width)
        center = image_in[h_2 - rad : h_2 + rad+1,
                          w_2 - rad : w_2 + rad+1]
        self.center_intensity = np.average(center)/255.0
        self.fovea_pub.publish(self.center_intensity)

    
if __name__ == '__main__':
    rospy.init_node('vision', anonymous=True)
    try:
        vision = Vision()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision"
