#!/usr/bin/env python
import roslib; roslib.load_manifest('bin_bot_base')
import rospy;
import subprocess
from std_msgs.msg import String

RATE = 1

class start_up_node():
    def __init__(self):
        self.serv_state = String()
        self.nodes_started = False

        # Subscriptions
        rospy.Subscriber('/client_node/serv_state', String, self.state_callback)

        # Publications
        self.state_pub = rospy.Publisher('/map_bot_base/state', String, queue_size=10)


    def state_callback(self, serv_state):
        self.serv_state = serv_state


    def spin(self):
        # Publish initial /map_bot_base/state
        self.state_pub.publish('FSM_WAIT')

        if (self.serv_state.data == 'START_MAPPING' and not self.nodes_started):
            rospy.loginfo('Start signal received. Waking up Map Bot.... ')
            self.nodes_started = True
            subprocess.call('roslaunch map_bot_base map_bot_base.launch', shell = True)
            rospy.loginfo('Still running.....')
            return
        else:
            rospy.loginfo('Waiting for signal to begin the mapping process')

if __name__ == '__main__':
    rospy.init_node("start_up_node")
    rate = rospy.Rate(RATE)
    try:
        my_start_up_node = start_up_node()
        while not rospy.is_shutdown() and not my_start_up_node.nodes_started:
            my_start_up_node.spin()
            rate.sleep()
    except rospy.ROSInterruptException: pass