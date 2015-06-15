#!/usr/bin/env python
import roslib; roslib.load_manifest('arm_bot_base')
import rospy
import os
import subprocess
from std_msgs.msg import String

RATE = 2

class start_up_node():
    def __init__(self):
        self.serv_state = String()
        self.nodes_started = False

        # Subscriptions
        rospy.Subscriber('/client_node/serv_state', String, self.state_callback)

        # Publications
        self.state_pub = rospy.Publisher('/arm_bot_base/state', String, queue_size=10)


    def state_callback(self, serv_state):
        self.serv_state = serv_state


    def spin(self):
        # Publish initial /map_bot_base/state
        self.state_pub.publish('FSM_WAIT')

        if (self.serv_state.data == 'MAP_AT_SERVER' and (os.path.isfile('/home/jason/map/map.yaml') and os.path.isfile('/home/jason/map/map.pgm'))):
            rospy.loginfo('Map received. Waking up Bin Bot.... ')
            self.nodes_started = True
            rospy.sleep(rospy.Duration(5))
            subprocess.call('roslaunch arm_bot_base full_arm_bot_base.launch', shell = True)
            rospy.loginfo('Still running.....')
            return
        else:
            rospy.loginfo('Waiting for map....')

if __name__ == '__main__':
    rospy.init_node("start_up_node")
    rate = rospy.Rate(RATE)
    try:
        my_start_up_node = start_up_node()
        while not rospy.is_shutdown() and not my_start_up_node.nodes_started:
            my_start_up_node.spin()
            rate.sleep()
    except rospy.ROSInterruptException: pass
