#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
from std_msgs.msg      import Float32
from std_msgs.msg      import String

import dynamic_reconfigure.server
from animat.cfg import PhysiologyParamConfig


class Physiology:
    

    def __init__(self):
        self.hydration          = 1
        self.glycemia           = 1
        self.dh                 = False
        self.dg                 = False
        self.freq               =   5 # cannot be reconfigured.
        self.decay_H            = .01
        self.decay_G            = .01
        self.shortage           = .01
        self.last_update        = rospy.Time.now()
        self.cmd_sub            = rospy.Subscriber("command",        String,          self.on_command, queue_size = 1)
        self.event_pub          = rospy.Publisher ("events",         String,                           queue_size = 1)
        self.hydration_pub      = rospy.Publisher ("hydration",     Float32,                           queue_size = 1)
        self.glycemia_pub       = rospy.Publisher ("glycemia",      Float32,                           queue_size = 1)
        self.config_srv = dynamic_reconfigure.server.Server(PhysiologyParamConfig, self.on_reconf)
        
    def on_reconf(self, config, level):
        self.decay_H  = config['decay_H']  
        self.decay_G  = config['decay_G']  
        self.shortage = config['shortage']  
        return config

    def evt_phy(self,old,new) :
        if old < self.shortage :
            if new >= self.shortage :
                return "refill"
            else :
                return None
        else :
            if new >= self.shortage :
                return None
            else :
                return "alert"

    def step(self):
        h = 0
        g = 0
        now = rospy.Time.now()
        dt = (now - self.last_update).to_sec()
        self.last_update = now
        if self.dh :
            h = 1
        else :
            h = self.hydration - self.decay_H*dt
            if h < 0 :
                h = 0
        if self.dg :
            g = 1
        else :
            g = self.glycemia - self.decay_G*dt
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
        rate = rospy.Rate(physiology.freq)
        while not rospy.is_shutdown():
            physiology.step()
            rate.sleep()
    except KeyboardInterrupt:
        print "Shutting down physiology"
