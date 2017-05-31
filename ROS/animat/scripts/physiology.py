#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
from std_msgs.msg      import Float32
from std_msgs.msg      import String

param_freq               = 10
param_decay_H            = .01
param_decay_G            = .01
param_shortage_threshold = .01

class Physiology:
    
    def __init__(self):
        self.hydration  = 1
        self.glycemia   = 1
        self.dh         = False
        self.dg         = False
        self.cmd_sub           = rospy.Subscriber("command",        String,          self.on_command, queue_size = 1)
        self.event_pub         = rospy.Publisher ("events",         String,                           queue_size = 1)
        self.hydration_pub     = rospy.Publisher ("hydration",     Float32,                           queue_size = 1)
        self.glycemia_pub      = rospy.Publisher ("glycemia",      Float32,                           queue_size = 1)

    def evt_phy(self,old,new) :
        if old < param_shortage_threshold :
            if new >= param_shortage_threshold :
                return "refill"
            else :
                return None
        else :
            if new >= param_shortage_threshold :
                return None
            else :
                return "alert"

    def step(self):
        h = 0
        g = 0
        if self.dh :
            h = 1
        else :
            h = self.hydration - param_decay_H
            if h < 0 :
                h = 0
        if self.dg :
            g = 1
        else :
            g = self.glycemia - param_decay_G
            if g < 0 :
                g = 0

        self.dh = False
        self.dg = False

        x = self.evt_phy(self.hydration,h)
        if not x is None :
            self.event_pub.publish("hydration_"+x)
        
        x = self.evt_phy(self.glycemia,g)
        if not x is None :
            self.event_pub.publish("glycemia_"+x)
            
        self.hydration = h
        self.glycemia  = g
        self.hydration_pub.publish(self.hydration)
        self.glycemia_pub.publish(self.glycemia)
        
        
    def on_command(self, command):
        if command.data == "eat" :
            self.dg = True
        if command.data == "drink" :
            self.dh = True
        else :
            pass
    
if __name__ == '__main__':
    rospy.init_node('physiology', anonymous=True)
    try:
        physiology = Physiology()
        rate = rospy.Rate(param_freq)
        while not rospy.is_shutdown():
            physiology.step()
            rate.sleep()
    except KeyboardInterrupt:
        print "Shutting down physiology"
