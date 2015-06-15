#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import subprocess

from std_srvs.srv import Empty
from std_msgs.msg import String
from move_base_msgs.msg import *
from actionlib import SimpleActionClient, GoalStatus
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, Vector3, PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

roslib.load_manifest('map_bot_base')

RATE = 10


class Enumerate(object):

    def __init__(self, names):
        self.names = []
        for number, name in enumerate(names.split()):
            setattr(self, name, number)
            self.names.append(name)

    def __geitem__(self, index):
        return self.names[index]

FSM_STATES = Enumerate("FSM_AUTONOMY FSM_MANUAL FSM_MOVE_TO_BASE FSM_IDLE")


class map_bot_base():

    ###########
    # __init__: Class instantiator.
    ##########
    def __init__(self):
        # global
        self.speed = Twist()
        self.state = FSM_STATES.FSM_AUTONOMY
        self.goal = MoveBaseGoal()
        self.base_pose = MoveBaseGoal()
        self.serv_state = String()


        # move_base Action Client
        self._move_base = SimpleActionClient('move_base', MoveBaseAction)

        # Subscribtions
        rospy.Subscriber('/map_bot_base/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/client_node/serv_state', String, self.serv_state_callback)

        # Publications
        self.state_pub = rospy.Publisher('/map_bot_base/state', String, queue_size=10)  # /navigation_velocity_smoother/raw_cmd_vel # /mobile_base/commands/velocity

        # Kill start up node to save processing power
        rospy.loginfo('Killing start up node......')
        subprocess.call('rosnode kill /start_up_node', shell = True)

        # Set up delay to allow for hector to startup
        rospy.loginfo('Starting up processes......')
        for i in range(9):  # 10 second delay
            rospy.loginfo('Initializing behaviour in %d' % (10-i))
            rospy.sleep(rospy.Duration(1))


    #################
    # goal_callback : Run everytime '\bin_bot_base\goal' topic receives message
    #               #  Sends a goal to the Action Server upon callback
    #################
    def goal_callback(self, pose_stamped):
        self.goal.target_pose = pose_stamped
        if(self.goal):
            self._move_base.send_goal(self.goal)
            rospy.loginfo("Just passed a goal")


    ######################
    # serv_state_callback : Run everytime the bin updates it's state.
    ######################
    def serv_state_callback(self, serv_state):
        self.serv_state = serv_state


    #################
    # spin_callback : Handles state transitions
    #################
    def spin_callback(self):

        # Print state
        print ("\n")
        rospy.loginfo("STATE : %s" % FSM_STATES.__geitem__(self.state))


        # FSM_AUTONOMY STATE
        # =============>
        if self.state == FSM_STATES.FSM_AUTONOMY:

            # Let hector handle navigation

            # Next state transition
            if (self._move_base.simple_state != 2): # if a goal is detected
                rospy.loginfo('Killing hector navigation nodes.......')
                subprocess.call('rosnode kill /hector_exploration_node', shell = True)
                self.state = FSM_STATES.FSM_MANUAL
            else:
                self.state = FSM_STATES.FSM_AUTONOMY


        # FSM_MANUAL STATE
        # =============>
        if self.state == FSM_STATES.FSM_MANUAL:

            # Let move base handle navigation

            # Next state transition
            # if-else statement to ensure error-less view of goal status
            if (self.serv_state.data == 'MAP_COMPLETE'): # if a goal is detected
                self.state = FSM_STATES.FSM_MOVE_TO_BASE
            else:
                self.state = FSM_STATES.FSM_MANUAL


        # FSM_MOVE_TO_BASE STATE
        # =============>
        if self.state == FSM_STATES.FSM_MOVE_TO_BASE:

            # Send a move base goal to the reset position
            #  and go to FSM_IDLE state
            self.base_pose.target_pose.header.frame_id = 'map'
            self.base_pose.target_pose.header.stamp = rospy.Time.now()
            self.base_pose.target_pose.pose.position.x = 0.5
            self.base_pose.target_pose.pose.position.y = 0.5
            self.base_pose.target_pose.pose.position.z = 0.0
            self.base_pose.target_pose.pose.orientation.x = 0.0
            self.base_pose.target_pose.pose.orientation.y = 0.0
            self.base_pose.target_pose.pose.orientation.z = 0.0
            self.base_pose.target_pose.pose.orientation.w = 0.0
            self._move_base.send_goal(self.base_pose)
            self.goal_state = self._move_base.get_state()
            rospy.loginfo("Map complete!. Moving back to base...")
            rospy.loginfo("Goal Status: %f" % self.goal_state)
            self._move_base.wait_for_result()
            if self._move_base.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo('Goal Reached!')
            else:
                rospy.loginfo('Goal aborted!')
            self.state = FSM_STATES.FSM_IDLE


        # FSM_IDLE STATE
        # =============>
        if self.state == FSM_STATES.FSM_IDLE:

            # Spin
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0

            # Next state transition
            # if-else statement to ensure error-less view of goal status
            if (self._move_base.simple_state == 1): # if goal is detected
                self.state = FSM_STATES.FSM_NAVIGATE
            else:
                self.state = FSM_STATES.FSM_IDLE


        # Print general info
        self.printINFO()

        # Publish /bin_bot_base/state
        self.state_pub.publish(String(FSM_STATES.__geitem__(self.state)))


    ##############
    # printINFO(): Prints general debug info
    #############
    def printINFO(self):
        # Print debug info about linear and angular speeds
        rospy.loginfo("speedPID: %f" %(self.speed.linear.x))
        rospy.loginfo("anglePID: %f" %(self.speed.angular.z))
        # Print goal status info
        if (self._move_base.simple_state == 2):
            self.goal_state = 'No goals received'
            rospy.loginfo("Goal Status: %s" %self.goal_state)
        else: # if a goal is detected
            self.goal_state = self._move_base.get_state()
            rospy.loginfo("Goal Status: %f" %self.goal_state)
        rospy.loginfo(self.serv_state.data)


    #########
    # spin(): Provides desired rate for speed_callback() and ensures termination
    ########
    def spin(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.spin_callback()
            rate.sleep()




### main()====================================================================>
if __name__ == '__main__':
    rospy.init_node("map_bot_base")

    try:
        my_map_bot_base = map_bot_base()
        my_map_bot_base.spin()
    except rospy.ROSInterruptException: pass
