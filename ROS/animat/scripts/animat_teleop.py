#! /usr/bin/env python
# -*- coding: utf-8 -*-

import curses
import math
import rospy
import sys
from std_msgs.msg import String

def main(stdscr):
    rospy.init_node('animat_teleop', anonymous=True)
    bpub = rospy.Publisher('body',   String, queue_size=1)
    vpub = rospy.Publisher('vision', String, queue_size=1)
    rate = rospy.Rate(10) 
    keycode = -1
    stdscr.addstr("Command\n")
    stdscr.addstr(" - UP/DOWN    : go/stop\n")
    stdscr.addstr(" - LEFT/RIGHT : turn left/right\n")
    stdscr.addstr(" - b/r/d/+    : focus on blue, red or don't care, + resets the focus.\n")
    stdscr.addstr(" - <space>    : ingest.\n")
    # We set the "wait for a key press" period to 100 ms. 
    while (not rospy.is_shutdown()) : 
        keycode = stdscr.getch()
        if   keycode == -1               : pass # No key has been pressed
        elif keycode == curses.KEY_UP    : bpub.publish("go")
        elif keycode == curses.KEY_DOWN  : bpub.publish("stop")
        elif keycode == curses.KEY_LEFT  : bpub.publish("left")
        elif keycode == curses.KEY_RIGHT : bpub.publish("right")
        elif keycode ==  32              : bpub.publish("ingest")   # space
        elif keycode ==  98              : vpub.publish("blue")     # b
        elif keycode == 114              : vpub.publish("red")      # r
        elif keycode == 100              : vpub.publish("dontcare") # d
        elif keycode ==  43              : vpub.publish("reset")    # +
        else                             : pass
        rate.sleep()

# Starts curses (terminal handling) and run our main function.
if __name__ == '__main__':
    try:
        curses.wrapper(lambda w: main(w))
    except rospy.ROSInterruptException:
        pass
