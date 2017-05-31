#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32

class Hardwired:
    
    def __init__(self):
        self.hydration_sub   = rospy.Subscriber('hydration',  Float32, self.on_hydration, queue_size=1)
        self.glycemia_sub    = rospy.Subscriber('glycemia',   Float32, self.on_glycemia,  queue_size=1)
        self.fovea_sub       = rospy.Subscriber('fovea',      Float32, self.on_fovea,     queue_size=1)
        self.attention_pub   = rospy.Publisher ('vision_cmd', String,                     queue_size=1)
        self.hydration       = 0
        self.glycemia        = 0
        self.attention       = 'blue'
        self.fovea_thresh    = .10
        self.fovea_duration  = 3
        self.last_fovea      = rospy.Time.now()

    def on_fovea(self, msg):
        if msg.data > self.fovea_thresh :
            self.last_fovea = rospy.Time.now()
        elif (rospy.Time.now() - self.last_fovea).to_sec() > self.fovea_duration :
            self.last_fovea = rospy.Time.now()
            self.attention_pub.publish("reset")
        
    def on_hydration(self, msg):
        self.hydration = msg.data

    def on_glycemia(self, msg):
        self.glycemia = msg.data
        if self.glycemia > self.hydration :
            self.attention = 'blue'
        elif self.glycemia < self.hydration:
            self.attention = 'red'
        self.attention_pub.publish(self.attention)
        
        
if __name__ == '__main__':
    rospy.init_node('hardwired', anonymous=True)
    try:
        hardwired = Hardwired()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down hardwired"
