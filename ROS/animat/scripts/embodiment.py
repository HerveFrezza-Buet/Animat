#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Embodiment:
    
    def __init__(self):
        self.cmd_sub   = rospy.Subscriber("command", String,  self.on_command, queue_size = 1)
        self.cmd_sub   = rospy.Subscriber("area",    String,  self.on_area,    queue_size = 1)
        self.cmd_pub   = rospy.Publisher ("physio",  String,                   queue_size = 1)
        self.twist_pub = rospy.Publisher ('cmd_vel', Twist,                    queue_size = 1)
        
        self.linear  = .1
        self.angular = .2
        self.twist   = Twist()
        self.time    = rospy.Time.now()
        self.area    = None 

    def on_command(self, command):
        if command.data == "go" :
            self.twist.angular.z = 0
            self.twist.linear.x  = self.linear
            self.twist_pub.publish(self.twist)
        elif command.data == "stop" :
            self.twist.angular.z = 0
            self.twist.linear.x  = 0
            self.twist_pub.publish(self.twist)
        elif command.data == "left" :
            self.twist.angular.z = self.angular
            self.twist.linear.x  = 0
            self.twist_pub.publish(self.twist)
        elif command.data == "right" :
            self.twist.angular.z = - self.angular
            self.twist.linear.x  = 0
            self.twist_pub.publish(self.twist)
        elif command.data == "ingest" :
            if (rospy.Time.now()-self.time).to_sec() < .2 :
                if self.area == "food" :
                    self.cmd_pub.publish("eat")
                if self.area == "water" :
                    self.cmd_pub.publish("drink")
                else :
                    pass
        else :
            pass
        
    def on_area(self, command):
        self.time = rospy.Time.now()
        self.area = command.data
        
if __name__ == '__main__':
    rospy.init_node('embodiment', anonymous=True)
    try:
        embodiment = Embodiment()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down embodiment"
