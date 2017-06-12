#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
from std_msgs.msg      import String
from std_msgs.msg      import Float32
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import math
import random

class Hardwired:
    
    def __init__(self):
        self.hydration       = 0
        self.glycemia        = 0
        self.fovea           = 0
        self.focus           = Point()
        self.nav_state       = 'reach'
        self.twist           = Twist()
        self.fovea_min       = .05
        self.last_fovea      = rospy.Time.now()
        self.last_search     = rospy.Time.now()
        self.hydration_sub   = rospy.Subscriber('hydration',  Float32, self.on_hydration,  queue_size = 1)
        self.glycemia_sub    = rospy.Subscriber('glycemia',   Float32, self.on_glycemia,   queue_size = 1)
        self.fovea_sub       = rospy.Subscriber('fovea',      Float32, self.on_fovea,      queue_size = 1)
        self.focus_sub       = rospy.Subscriber('focus',      Point,   self.on_focus,      queue_size = 1)
        self.twist_pub       = rospy.Publisher ('cmd_vel',    Twist,                       queue_size = 1)
        self.pantilt_pub     = rospy.Publisher ('pantilt',    Point,                       queue_size = 1)
        

    def move(self):
        if self.twist.angular.z < -.5 :
            self.twist.angular.z = -.5
        elif self.twist.angular.z > .5 :
            self.twist.angular.z = .5
        self.twist_pub.publish(self.twist)

    def on_focus(self, focus):
        self.focus = focus
        
    def on_fovea(self, msg):
        self.fovea = msg.data
        
    def on_hydration(self, msg):
        self.hydration = msg.data

    def on_glycemia(self, msg):
        self.glycemia = msg.data

    def idle(self):
        if self.nav_state is None:
            pass
        
        elif self.nav_state == 'search':
            if self.fovea > self.fovea_min :
                self.nav_state = 'reach'
            elif (rospy.Time.now() - self.last_search).to_sec() > 2.0 :
                self.last_search = rospy.Time.now()
                msg = Point()
                msg.y = 0
                msg.x = random.uniform(-180,180)
                self.pantilt_pub.publish(msg)
                
        elif self.nav_state == 'reach':
            if self.fovea < self.fovea_min :
                self.twist.angular.z = 0
                self.twist.linear.x  = 0
                if (rospy.Time.now() - self.last_fovea).to_sec() > 2.0 :
                    self.nav_state = 'search'
            else :
                self.last_fovea      = rospy.Time.now()
                self.twist.angular.z = .05*self.focus.x
                max_deg = 45
                f = min(math.fabs(self.focus.x),max_deg) / float(max_deg)
                self.twist.linear.x  = .1*(1-f)
            self.move()
            
        
        
if __name__ == '__main__':
    rospy.init_node('hardwired', anonymous=True)
    try:
        hardwired = Hardwired()
        rate = rospy.Rate(5) 
        while not rospy.is_shutdown():
            hardwired.idle()
            rate.sleep()
    except KeyboardInterrupt:
        print "Shutting down hardwired"
