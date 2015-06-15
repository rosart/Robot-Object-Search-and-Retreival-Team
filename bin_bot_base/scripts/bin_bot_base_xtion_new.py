#!/usr/bin/env python
import roslib
import rospy
import math
import tf

from std_srvs.srv import Empty
from std_msgs.msg import String
from move_base_msgs.msg import *
from actionlib import SimpleActionClient, GoalStatus
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, Vector3, PoseStamped, PoseWithCovarianceStamped
from cmvision_3d.msg import Blobs3d
from tf.transformations import euler_from_quaternion

roslib.load_manifest('bin_bot_base')

RATE = 10


class Enumerate(object):

    def __init__(self, names):
        self.names = []
        for number, name in enumerate(names.split()):
            setattr(self, name, number)
            self.names.append(name)

    def __geitem__(self, index):
        return self.names[index]

FSM_STATES = Enumerate("FSM_RESET FSM_LOCALIZE FSM_IDLE FSM_NAVIGATE FSM_SEARCH FSM_OPTIMIZE FSM_APPROACH_OBJ FSM_FINAL_APPROACH FSM_STABLE FSM_SPIN FSM_MOVE_TO_BASE FSM_WAIT_FOR_ACTION")


class bin_bot_base():

    ###########
    # __init__: Class instantiator.
    ##########
    def __init__(self, KpRot, KdRot, tol_z, tol_ang, maxRotSpeed, max_z, min_z):
        # global
        self.speed = Twist()
        self.tracked_blob = Twist()
        self.old_tracked_blob = Twist()
        self.state = FSM_STATES.FSM_LOCALIZE
        self.obj_pose = MoveBaseGoal()
        self.goal = MoveBaseGoal()
        self.base_pose = MoveBaseGoal()
        self.filtered_blobs_3d = Blobs3d()
        self.blobs_detected = False
        self.miss_counter = 0
        self.correction_start = 0.0
        self.serv_state = String()
        self.blob_count = 0
        self.avg_z = 0.0
        self.yaw = 0.0
        self.init_yaw = 0.0
        self.target_yaw = 0.0
        self.RotDerivator = 0.0
        self.yawDerivator = 0.0

        # params
        self.KpRot = KpRot
        self.KdRot = KdRot
        self.tol_z = tol_z
        self.tol_ang = tol_ang
        self.maxRotSpeed = maxRotSpeed
        self.max_z = max_z
        self.min_z = min_z

        # move_base Action Client
        self._move_base = SimpleActionClient('move_base', MoveBaseAction)

        # Subscribtions
        rospy.Subscriber('/blobs_3d', Blobs3d, self.blob_callback)
        rospy.Subscriber('/bin_bot_base/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/client_node/serv_state', String, self.serv_state_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)

        # Publications
        self.speed_pub = rospy.Publisher('/navigation_velocity_smoother/raw_cmd_vel', Twist)  # /navigation_velocity_smoother/raw_cmd_vel # /mobile_base/commands/velocity
        self.target_pose_pub = rospy.Publisher('/bin_bot_base/target_pose', PoseStamped)
        self.state_pub = rospy.Publisher('/bin_bot_base/state', String, queue_size=10)  # /navigation_velocity_smoother/raw_cmd_vel # /mobile_base/commands/velocity

        # Set up delay to allow for amcl to startup
        rospy.loginfo('Starting up processes......')
        for i in range(9):  # 10 second delay
            rospy.loginfo('Initializing behaviour in %d' % (10-i))
            rospy.sleep(rospy.Duration(1))
        self.loc_start = rospy.Time.now()


    #################
    # goal_callback : Run everytime '\bin_bot_base\goal' topic receives message
    #               #  Sends a goal to the Action Server upon callback
    #################
    def goal_callback(self, pose_stamped):
        self.goal.target_pose = pose_stamped
        if(self.goal):
            self._move_base.send_goal(self.goal)
            rospy.loginfo("Just passed a goal")

    #################
    # blob_callback : Run everytime \blobs_3d receives message. Filters blobs depending on their position to exclude blobs outside
    #               #  the arena and a certain range. The largest blob is selected and recorded for processing.
    #################
    def blob_callback(self, blobs_3d):
        self.old_tracked_blob = self.tracked_blob
        self.filtered_blobs_3d = Blobs3d()  # Initialize empty filtered blobs list
        self.max_blob_area = 0              # reset maximum blob area
        self.sum_x = 0.0
        self.blob_count = 0

        # Filter blobs
        if(len(blobs_3d.blobs)):
            for blob in blobs_3d.blobs:
                if (blob.center.y >= 0.000 and blob.center.z <= 0.0012):  # if blob is closer than 1.2m and below sensor level
                    self.filtered_blobs_3d.blobs.append(blob)

        # if filtered list contains blobs
        if(len(self.filtered_blobs_3d.blobs)):
            self.blobs_detected = True  # signal that a blobs have been detected
            self.miss_counter = 0   # reset miss counter
            # Find largest blob and track it
            for blob in self.filtered_blobs_3d.blobs:
                self.blob_count = self.blob_count + 1
                self.sum_x = self.sum_x + blob.center.x
                if(blob.area > self.max_blob_area):
                    self.max_blob_area = blob.area
                    self.tracked_blob.linear = self.getPose(blob)
            self.avg_x = float(self.sum_x / self.blob_count)
            self.update_avg_z()
            self.tracked_blob.linear.x = self.avg_x
            self.tracked_blob.linear.z = self.avg_z
            self.pub_tracked_blob_tf(self.tracked_blob.linear)

        # else if filtered list does not contain any blobs
        else:
            self.miss_counter += 1  # increment miss count
            if(self.miss_counter >= 10):  # Allow for 10 callbacks before signaling non-detection
                self.blobs_detected = False
                self.avg_z = 0
                self.tracked_blob = Twist()


    ######################
    # serv_state_callback : Run everytime the bin updates it's state.
    ######################
    def serv_state_callback(self, serv_state):
        self.serv_state = serv_state
        if self.serv_state.data == "RESET":
            self.state = FSM_STATES.FSM_RESET
            self.state_pub.publish(String(FSM_STATES.__geitem__(self.state)))


    #################
    # amcl_callback : Debug callback to compute yaw
    #################
    def amcl_callback(self, robot_pose):
        self.robot_pose = robot_pose
        (roll,pitch,self.yaw) = euler_from_quaternion([self.robot_pose.pose.pose.orientation.x,
        self.robot_pose.pose.pose.orientation.y, self.robot_pose.pose.pose.orientation.z, self.robot_pose.pose.pose.orientation.w])


    #################
    # speed_callback : Pubishes velocity and handles state transitions
    #################
    def speed_callback(self):
        self.zStable = False # flag to signify desired z satisfied
        self.angStable = False # flag to signify desired angle satisfied
        self.sleepDuration = rospy.Duration(3) # duration of forward movement

        # Print state
        print ("\n")
        rospy.loginfo("STATE : %s" % FSM_STATES.__geitem__(self.state))


        # FSM_LOCALIZE STATE
        # =============>
        if self.state == FSM_STATES.FSM_LOCALIZE:

            # Spin
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.5

            # Next state transition
            # if-else statement to ensure error-less view of goal status
            if(rospy.Time.now() < self.loc_start + rospy.Duration(20)):
                self.state = FSM_STATES.FSM_LOCALIZE
            else:
                self.state = FSM_STATES.FSM_IDLE


        # FSM_RESET STATE
        # =============>
        if self.state == FSM_STATES.FSM_RESET:

            # Send a move base goal to the reset position
            #  and go to FSM_IDLE state
            self.reset_goal = MoveBaseGoal()
            self.reset_goal.target_pose.header.frame_id = 'map'
            self.reset_goal.target_pose.header.stamp = rospy.Time.now()
            self.reset_goal.target_pose.pose.position.x = 0.0
            self.reset_goal.target_pose.pose.position.y = -4.0
            self.reset_goal.target_pose.pose.position.z = 0.0
            self.reset_goal.target_pose.pose.orientation.x = 0.0
            self.reset_goal.target_pose.pose.orientation.y = 0.0
            self.reset_goal.target_pose.pose.orientation.z = 0.5
            self.reset_goal.target_pose.pose.orientation.w = 1.0
            self._move_base.send_goal(self.reset_goal)
            self.goal_state = self._move_base.get_state()
            rospy.loginfo("Reset signal received by server. Moving back to base...")
            rospy.loginfo("Goal Status: %f" % self.goal_state)
            self._move_base.wait_for_result()
            if self._move_base.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo('Goal Reached!')
            else:
                rospy.loginfo('Goal aborted!')
            rospy.wait_for_service('/move_base/clear_costmaps')
            self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            try:
                rospy.wait_for_service('/move_base/clear_costmaps')
                self.clear_costmaps()
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
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



        # FSM_NAVIGATE STATE
        # =============>
        elif self.state == FSM_STATES.FSM_NAVIGATE:

            # No action required, simply allow path planner to navigate
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0
            # Print debug info
            if (self._move_base.get_state() == GoalStatus.SUCCEEDED):
                rospy.loginfo('Search for object failed!')

            # Next state transitions
            if(self.blobs_detected): # if a blob is seen
                rospy.loginfo("Goal Status: %f" %self._move_base.get_state())
                rospy.loginfo("Object detected! Aborting goal and approaching object...")
                self.state = FSM_STATES.FSM_OPTIMIZE
            else:
                if(self._move_base.get_state() != 3.0):
                    rospy.loginfo("Navigating to object at pose (%f, %f, %f)..." %(self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y, self.goal.target_pose.pose.position.z))
                else:
                     rospy.loginfo("Waiting for goal...")
                rospy.loginfo("Goal Status: %f" %self._move_base.get_state())
                self.state = FSM_STATES.FSM_NAVIGATE


        # FSM_SEARCH STATE
        # ===============>
        elif self.state == FSM_STATES.FSM_SEARCH:

            # Spin to search for object
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.5

            # Next state transitions
            if (self.blobs_detected):
                self.state = FSM_STATES.FSM_OPTIMIZE
            elif (self._move_base.simple_state == 1): # if goal is detected
                self.state = FSM_STATES.FSM_NAVIGATE
            else:
                self.state = FSM_STATES.FSM_SEARCH


        # FSM_OPTIMIZE STATE (PID/PD controller)
        # =================>
        elif self.state == FSM_STATES.FSM_OPTIMIZE :
            self._move_base.cancel_all_goals()
            rospy.loginfo("Blob detected at x:%f, y:%f, z:%f "%(self.tracked_blob.linear.x, self.tracked_blob.linear.y, self.tracked_blob.linear.z))


            # if statement to check object is with range (TO BE DELETED/AMENDED!)
            if (self.tracked_blob.linear.z > self.max_z or self.tracked_blob.linear.z < self.min_z):
                self.speed.linear.x = 0.0
                self.speed.angular.z = 0.0
                rospy.loginfo("Distance = %f" %(self.tracked_blob.linear.z))
                rospy.loginfo("Detected object is either too close or too far away.")
            else:

                # Set linear speed to 0
                self.speed.linear.x = 0.0

                # Compute PD angular error
                self.angError = -math.atan2(self.tracked_blob.linear.x, self.tracked_blob.linear.y);
                self.pRotError = self.KpRot*self.angError
                self.dRotError = self.KdRot*(self.angError-self.RotDerivator)
                self.RotDerivator = self.angError
                self.anglePID = self.pRotError + self.dRotError;

                # Cap maximum rotation speed
                if(math.fabs(self.anglePID)>self.maxRotSpeed):
                    self.anglePID=self.maxRotSpeed*math.fabs(self.anglePID)/self.anglePID

                # Set tolerance in angle to signify stable orientation
                if(math.fabs(self.angError) < self.tol_ang):
                    self.speed.angular.z = 0.0
                    self.RotDerivator = 0.0
                    self.angStable = True
                else:
                    self.speed.angular.z = self.anglePID
                    self.angStable = False

                # Debug messages
                rospy.loginfo("distance: %f" %(self.tracked_blob.linear.z))
                rospy.loginfo("angError: %f, pRotError: %f, dRotError: %f" %(self.angError, self.pRotError, self.dRotError))


            # Next state transitions
            if (self.angStable): # if stable position and orientation have been achieved
                self.goal = MoveBaseGoal() # reset goal
                if (self.tracked_blob.linear.z > 0.001): # if object is cloaser than 1 m
                    self.state = FSM_STATES.FSM_APPROACH_OBJ
                else:
                    self.correction_start = rospy.Time.now()
                    self.state = FSM_STATES.FSM_STABLE # move to stable state
            elif(self.blobs_detected):    # else if (while) blobs are being detected
                self.state = FSM_STATES.FSM_OPTIMIZE # loop in same state
            else:
                #self._move_base.send_goal(self.goal)
                #rospy.sleep(self.sleepDuration)
                #self.state = FSM_STATES.FSM_LOCALIZE # else move to nav state
                self.state = FSM_STATES.FSM_SEARCH


        # FSM_APPROACH_OBJ STATE
        # =============>
        elif self.state == FSM_STATES.FSM_APPROACH_OBJ:

            # Set constants and time in goal pose
            self.obj_pose.target_pose.header.frame_id = 'camera_depth_frame'
            self.obj_pose.target_pose.header.stamp = rospy.Time.now()
            self.obj_pose.target_pose.pose.orientation.w = 1.0

            # if-else statements to perform corrections in target pose
            # if distance is less than 2m and more than 1.50m (2.5 m gazebo)
            #if (self.tracked_blob.linear.z > 0.001):
            #    self.obj_pose.target_pose.pose.position.x = 1000*(self.tracked_blob.linear.z - 0.001) # set goal to (distance - 1m)
            #    self.target_pose_pub.publish(self.obj_pose.target_pose)
            #    self._move_base.send_goal(self.obj_pose) # send goal
            #    rospy.loginfo('Performing approach to 1m. Pose x: %f, w: %f.' %(self.obj_pose.target_pose.pose.position.x, self.obj_pose.target_pose.pose.orientation.w))
            #    rospy.loginfo('Waiting for result...')
            #    self._move_base.wait_for_result()
            #    if self._move_base.get_state() == GoalStatus.SUCCEEDED:
            #        rospy.loginfo('Goal Reached!')
            #    else:
            #        rospy.loginfo('Goal aborted!')

            # else if distance is less than 1m
            #else:
            self.obj_pose.target_pose.pose.position.x = 1000*(self.tracked_blob.linear.z) - 0.7 # set goal to final target
            self.target_pose_pub.publish(self.obj_pose.target_pose)
            self._move_base.send_goal(self.obj_pose)  # send goal
            rospy.loginfo('Performing final approach. Pose x: %f, w: %f.' % (self.obj_pose.target_pose.pose.position.x, self.obj_pose.target_pose.pose.orientation.w))
            rospy.loginfo('Waiting for result...')
            self._move_base.wait_for_result()
            if self._move_base.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo('Goal Reached!')
            else:
                rospy.loginfo('Goal aborted!')

            # Next state transitions
            if (self.blobs_detected):
                self.state = FSM_STATES.FSM_OPTIMIZE # loop back to FSM_OPTIMIZE to correct angle
            else:
                self.state = FSM_STATES.FSM_SEARCH



        # FSM_STABLE STATE
        # ===============>
        elif (self.state == FSM_STATES.FSM_STABLE):

            # Allow 2 seconds for blob distance to saturate and then compute forward movement duration
            if (rospy.Time.now() < self.correction_start + rospy.Duration(2)):
                rospy.loginfo('Correction of detection errors...')
                self.state = FSM_STATES.FSM_STABLE
            else:
                self.moveDuration = rospy.Duration((self.tracked_blob.linear.z - 0.00032)/0.0001)
                rospy.loginfo((self.tracked_blob.linear.z - 0.00032)/0.0001)
                self.moveStartTime = rospy.Time.now()
                self.state = FSM_STATES.FSM_FINAL_APPROACH



        # FSM_FINAL_APPROACH
        # ==================>
        elif (self.state == FSM_STATES.FSM_FINAL_APPROACH):
            # Move forward for desired duration to approach target object
            if (rospy.Time.now() < self.moveStartTime + self.moveDuration):
                self.speed.linear.x = 0.1
                self.speed.angular.z = 0.0
                self.state = FSM_STATES.FSM_FINAL_APPROACH
            # Once duration has elapsed move to spin state
            else:
                self.speed.linear.x = 0.0
                self.speed.angular.z = 0.0
                self.init_yaw = self.yaw
                if(self.init_yaw >= 0):
                    self.target_yaw = self.init_yaw - math.pi
                else:
                    self.target_yaw = self.init_yaw + math.pi
                self.state = FSM_STATES.FSM_SPIN



        # FSM_SPIN STATE
        # =============>
        elif (self.state == FSM_STATES.FSM_SPIN):
            self.KpYaw = 1.0
            self.KdYaw = 0.5
            self.yawStable = False

            # Spin
            self.yawError = self.target_yaw - self.yaw
            self.pYawError = self.yawError*self.KpYaw
            self.dYawError = self.KdYaw*(self.yawError-self.yawDerivator)
            self.yawDerivator = self.yawError
            self.anglePID = self.pYawError + self.dYawError

            if(math.fabs(self.anglePID)>self.maxRotSpeed):
                self.anglePID=self.maxRotSpeed*math.fabs(self.anglePID)/self.anglePID;

            if(math.fabs(self.yawError)<self.tol_ang):
                self.speed.angular.z = 0.0
                self.yawDerivator = 0.0
                self.yawStable = True
            else:
                self.speed.angular.z = self.anglePID



            # Next state transitions
            # -Until bin sees object-! (Current content is temporary)
            if (self.yawStable):
                self.state = FSM_STATES.FSM_WAIT_FOR_ACTION
            elif (self._move_base.simple_state == 1): # if goal is detected
                self.state = FSM_STATES.FSM_NAVIGATE
            else:
                self.state = FSM_STATES.FSM_SPIN

        # FSM_WAIT_FOR_ACTION STATE
        # =============>
        elif (self.state == FSM_STATES.FSM_WAIT_FOR_ACTION):

            # Stop motors
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0

            # Next state transitions
            if ((self._move_base.simple_state != 2)or(self.serv_state.data == 'BIN_TO_BASE')): # if a goal is detected
                self.state = FSM_STATES.FSM_MOVE_TO_BASE # change to MOVE_TO_BASE state
            else: # if object not dropped
                self.state = FSM_STATES.FSM_WAIT_FOR_ACTION # loop back


        # FSM_MOVE_TO_BASE STATE
        # =============>
        elif (self.state == FSM_STATES.FSM_MOVE_TO_BASE):
            self.base_pose.target_pose.header.frame_id = 'map'
            self.base_pose.target_pose.header.stamp = rospy.Time.now()
            self.base_pose.target_pose.pose.position.x = 0.0
            self.base_pose.target_pose.pose.position.y = -4.0
            self.base_pose.target_pose.pose.position.z = 0.0
            self.base_pose.target_pose.pose.orientation.x = 0.0
            self.base_pose.target_pose.pose.orientation.y = 0.0
            self.base_pose.target_pose.pose.orientation.z = 0.5
            self.base_pose.target_pose.pose.orientation.w = 1.0
            self._move_base.send_goal(self.base_pose)
            self.goal_state = self._move_base.get_state()
            rospy.loginfo("Moving back to base")
            rospy.loginfo("Goal Status: %f" %self.goal_state)
            self._move_base.wait_for_result()
            if self._move_base.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo('Goal Reached!')
            else:
                rospy.loginfo('Goal aborted!')
            self.state = FSM_STATES.FSM_IDLE
            rospy.wait_for_service('/move_base/clear_costmaps')
            self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            try:
                rospy.wait_for_service('/move_base/clear_costmaps')
                self.clear_costmaps()
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))


        # Print general info
        self.printINFO()

        # Publish /bin_bot_base/state
        self.state_pub.publish(String(FSM_STATES.__geitem__(self.state)))

        # Only publish velocity when no goal is being pursued or expected
        if (self.state != FSM_STATES.FSM_NAVIGATE):
            self.speed_pub.publish(self.speed)



    ############
    # getPose(): Returns pose of received blob relative to the camera.
    ############
    def getPose(self, blob):
        self.pose = Vector3()
        self.pose.x = blob.center.x
        self.pose.y = blob.center.y
        self.pose.z = blob.center.z
        return self.pose



    #################
    # update_avg_z(): Updates the avg_value of distance z from blob.
    #               #   Higher weight is given to blobs received with lower distance than the average,
    #               #   and vice versa, such that the closest distance is ultimately tracked.
    #################
    def update_avg_z(self):
        if (self.old_tracked_blob.linear.z == 0):
            self.avg_z = self.tracked_blob.z
        else:
            if(self.tracked_blob.linear.z < self.avg_z): # Give heigher weight to closer blobs
                self.avg_z = (self.avg_z + 10*self.tracked_blob.linear.z)/(11)
            else:
                self.avg_z = (10*self.avg_z + self.tracked_blob.linear.z)/(11)



    ########################
    # pub_tracked_blob_tf(): Publishes a transform frame for the tracked blob.
    #######################
    def pub_tracked_blob_tf(self, pose):
        br = tf.TransformBroadcaster()
        br.sendTransform((1000*pose.z, -1000*pose.x, -1000*pose.y), tf.transformations.quaternion_from_euler(0, 0, 1), rospy.Time.now(), 'tracked_blob', 'camera_depth_frame')



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
        # Print blob info
        if(self.blobs_detected):
            rospy.loginfo("Blob distance: %f" %(self.tracked_blob.linear.z))
            rospy.loginfo("miss_counter = %f" %(self.miss_counter))
        else:
            rospy.loginfo("No blobs detected")
            rospy.loginfo("miss_counter = %f" %(self.miss_counter))
        # Print yaw (**DEBUG**)
        rospy.loginfo(self.yaw)
        rospy.loginfo(self.serv_state.data)




    #########
    # spin(): Provides desired rate for speed_callback() and ensures termination
    ########
    def spin(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.speed_callback()
            rate.sleep()




### main()====================================================================>
if __name__ == '__main__':
    rospy.init_node("bin_bot_base")

    # Get params from launch file, or else set to default value

    KpRot = rospy.get_param('bin_bot_base/KpRot', 1.0)
    KdRot = rospy.get_param('bin_bot_base/KdRot', 0.0)
    tol_z = rospy.get_param('bin_bot_base/tol_z', 0.05) #0.0005
    tol_ang= rospy.get_param('bin_bot_base/tol_ang', 0.15)
    maxRotSpeed = rospy.get_param('bin_bot_base/maxRotSpeed', 1.0)
    max_z = rospy.get_param('bin_bot_base/max_z', 0.003)
    min_z= rospy.get_param('bin_bot_base/min_z', 0.0002)#0.0005
    try:
        my_bin_bot_base = bin_bot_base(KpRot, KdRot, tol_z, tol_ang, maxRotSpeed, max_z, min_z)
        my_bin_bot_base.spin()
    except rospy.ROSInterruptException: pass
