#!/usr/bin/env python
import roslib; roslib.load_manifest('arm_bot_base')
import rospy;
import os.path
import subprocess
import time
from std_msgs.msg import String

RATE = 2

class start_up_node():
    def __init__(self):
        self.serv_state = String()
        self.nodes_started = False
        rospy.Subscriber('/client_node/serv_state', String, self.state_callback)

    def state_callback(self, serv_state):
        self.serv_state = serv_state

    def spin(self):

        if (self.serv_state.data == 'MAP_DONE' and not self.nodes_started):
            self.nodes_started = True
            subprocess.call('roslaunch arm_bot_base arm_bot_base.launch', shell = True)
            rospy.loginfo('Still running.....')
            return
        else:
            print 'Waiting for map....'

if __name__ == '__main__':
    rate = rospy.Rate(RATE)
    rospy.init_node("start_up_node")
    try:
        my_start_up_node = start_up_node()
        while not rospy.is_shutdown() and not my_start_up_node.nodes_started:
            my_start_up_node.spin()
            rate.sleep()
    except rospy.ROSInterruptException: pass