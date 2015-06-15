#!/usr/bin/env python
import roslib; roslib.load_manifest('arm_bot_base')
import rospy
import math

from std_msgs.msg import String
from move_base_msgs.msg import *
from actionlib import SimpleActionClient, GoalStatus
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from cmvision_3d.msg import Blobs3d, Blob3d

RATE = 10
class Enumerate(object):
    
    def __init__(self, names):
        self.names=[]
        for number, name in enumerate(names.split()):
            setattr(self, name, number)
            self.names.append(name)
       
    def __geitem__(self, index):
        return self.names[index]

FSM_STATES = Enumerate("FSM_LOCALIZE FSM_NAVIGATE FSM_SEARCH FSM_OPTIMIZE FSM_APPROACH_OBJ FSM_WAIT_FOR_DROP FSM_STABLE FSM_SPIN FSM_MOVE_TO_BASE")

class bin_bot_base():
    
    def __init__(self, desired_z, KpLin, KdLin, KpRot, KdRot, tol_z, tol_ang, maxLinSpeed, maxRotSpeed, max_z, min_z):
        #global  
        self.speed = Twist() # variable to store velocity commands
        self.tracked_blob = Twist() # variable to store pose of tracked blob
        self.state = FSM_STATES.FSM_LOCALIZE # set initial state
        self.obj_pose = MoveBaseGoal() # variable
        self.goal = MoveBaseGoal()
        self.blobs_3d = Blobs3d()
        self.blobs_detected = False
        self.miss_counter = 0
        self.zStable = False # flag to signify desired z satisfied
        self.angStable = False # flag to signify desired angle satisfied
        self.arm_state = String

        #params
        self.desired_z = desired_z #0.001	#0.8
        self.KpLin = KpLin #1000.0
        self.KdLin = KdLin
        self.KpRot = KpRot #0.1
        self.KdRot = KdRot
        self.tol_z = tol_z #0.00005 #0.05
        self.tol_ang= tol_ang #0.5
        self.maxLinSpeed = maxLinSpeed #0.2
        self.maxRotSpeed = maxRotSpeed
        self.max_z = max_z #3.0
        self.min_z= min_z #0.0005 #0.2
       
        # move_base Action Client
        self._move_base = SimpleActionClient('move_base', MoveBaseAction)
        self._move_base.wait_for_server()
	    
        #subscribtions
        rospy.Subscriber('/blobs_3d', Blobs3d, self.blob_callback)
        rospy.Subscriber('/bin_bot_base/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/arm_node/state', String, self.arm_state_callback)
               
	    # publications
        self.speed_pub = rospy.Publisher('/navigation_velocity_smoother/raw_cmd_vel', Twist) #/navigation_velocity_smoother/raw_cmd_vel # /mobile_base/commands/velocity
        
        self.state_pub = rospy.Publisher('/bin_bot_base/state', String, queue_size=10) #/navigation_velocity_smoother/raw_cmd_vel # /mobile_base/commands/velocity



    #################
    # goal_callback : Run everytime '\arm_bot_base\goal' topic receives message.  
    #               #  Send a goal to the Action Server upon callback
    #################
    def goal_callback(self, pose_stamped):
        self.goal.target_pose = pose_stamped
        if(self.goal):
            self._move_base.send_goal(self.goal)
            rospy.loginfo("Just passed a goal")



    #################
    # blob_callback : Run everytime \blobs_3d receives message.
    #################  
    def blob_callback(self, blobs_3d):
        self.blobs_3d = blobs_3d
        self.blob_area = 0
        if(len(self.blobs_3d.blobs)):
            self.blobs_detected = True
            self.miss_counter = 0
            for blob in blobs_3d.blobs:
                if (blob.area>self.blob_area):
                    self.blob_area = blob.area
                    self.tracked_blob.linear = self.getPose(blob)             
        else:
            if(self.miss_counter>=5): # Allow for 5 callbacks before signaling non-detection
                self.blobs_detected = False
            self.miss_counter += 1
        
    ######################
    # arm_state_callback : Run everytime the arm updates it's state.
    ######################   
    def arm_state_callback(self, arm_state):
        self.arm_state = arm_state
        

    #################
    # speed_callback : Pubishes velocity and handles state transitions
    #################
    def speed_callback(self):
        
        self.moveDuration = rospy.Duration(3) # duration of forward movement
        
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
            if(self.blobs_detected): # if a blob is seen
                rospy.loginfo("Arm Bot detected! Approaching robot...")
                self.state = FSM_STATES.FSM_OPTIMIZE            
            elif (self._move_base.simple_state == 2):
                self.goal_state = 'No goals received'
                rospy.loginfo("Goal Status: %s" %self.goal_state)
                self.state = FSM_STATES.FSM_LOCALIZE
            else: # if a goal is detected
                self.goal_state = self._move_base.get_state()
                rospy.loginfo("Goal Status: %f" %self.goal_state)
                self.state = FSM_STATES.FSM_NAVIGATE
            

        # FSM_NAVIGATE STATE
        # =============>
        elif self.state == FSM_STATES.FSM_NAVIGATE:

            # No action required, simply allow path planner to navigate    
    	    self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0

            # Next state transitions
            if(self.blobs_detected): # and (self.tracked_blob.linear.z < 3.0): # if a blob is seen (and closer than 3 meters)
                self._move_base.cancel_all_goals() # cancell all nav goals
                rospy.loginfo("Goal Status: %f" %self._move_base.get_state())
                rospy.loginfo("Arm Bot detected! Aborting goal and approaching robot...")
                self.state = FSM_STATES.FSM_OPTIMIZE 
            else:
                if(self._move_base.get_state() != 3.0):
                    rospy.loginfo("Navigating to Arm Bot at pose (%f, %f, %f)..." %(self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y, self.goal.target_pose.pose.position.z)) 
                else:
                     rospy.loginfo("Waiting for goal...")   
                rospy.loginfo("Goal Status: %f" %self._move_base.get_state())
                self.state = FSM_STATES.FSM_NAVIGATE    



        # FSM_OPTIMIZE STATE (PID/PD controller)
        # =================>
        elif self.state == FSM_STATES.FSM_OPTIMIZE :
        
            rospy.loginfo("Blob detected at x:%f, y:%f, z:%f "%(self.tracked_blob.linear.x, self.tracked_blob.linear.y, self.tracked_blob.linear.z))
            #self.LinDerivator = 0.0
            self.RotDerivator = 0.0
            
            # if statement to check Arm Bot is with range (TO BE DELETED/AMENDED!)
            if (self.tracked_blob.linear.z > self.max_z or self.tracked_blob.linear.z < self.min_z):
                self.speed.linear.x = 0.0
                self.speed.angular.z = 0.0
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
                    self.anglePID=self.maxRotSpeed*math.fabs(self.anglePID)/self.anglePID;          
                
                # Set tolerance in angle to signify stable orientation
                if(math.fabs(self.angError) < self.tol_ang):
                    self.speed.angular.z = 0.0;
                    RotDerivator = 0.0
                    self.angStable = True
                else:
                    self.speed.angular.z = self.anglePID;
                    self.angStable = False
                
                # Debug messages
                rospy.loginfo("distance: %f" %(self.tracked_blob.linear.z))    
                rospy.loginfo("angError: %f, pRotError: %f, dRotError: %f" %(self.angError, self.pRotError, self.dRotError))
            
    
            # Next state transitions
            if (self.angStable): # if stable position and orientation have been achieved
                    self.state = FSM_STATES.FSM_APPROACH_OBJ # move to stable state
                    self.moveStartTime = rospy.Time.now()
            elif(self.blobs_detected):    # else if (while) blobs are being detected
                    self.state = FSM_STATES.FSM_OPTIMIZE # loop in same state
            else:
                self.state = FSM_STATES.FSM_LOCALIZE # else move to nav state
       

        # FSM_APPROACH_OBJ STATE
        # =============>
        elif self.state == FSM_STATES.FSM_APPROACH_OBJ:

            self.obj_pose.target_pose.header.frame_id = 'camera_depth_frame'
            self.obj_pose.target_pose.header.stamp = rospy.Time.now()
            self.obj_pose.target_pose.pose.orientation.w = 1.0
            

            # if-else statements to perform corrections in target pose 
            # if distance is higher than 2.20m             
            if ( self.tracked_blob.linear.z > 0.0022):
                self.obj_pose.target_pose.pose.position.x = 1000*(self.tracked_blob.linear.z - 0.002) # set goal to (distance - 2m)
                self._move_base.send_goal(self.obj_pose) # send goal
                rospy.loginfo('Performing approach to 2m. Pose x: %f, w: %f.' %(self.obj_pose.target_pose.pose.position.x, self.obj_pose.target_pose.pose.orientation.w))
                rospy.loginfo('Waiting for result...')
                self._move_base.wait_for_result()
                if self._move_base.get_state() == GoalStatus.SUCCEEDED:
                    rospy.loginfo('Goal Reached!')
                else:
                    rospy.loginfo('Goal aborted!')
                self.state = FSM_STATES.FSM_OPTIMIZE # loop back to FSM_OPTIMIZE to correct angle
            
            # else if distance is less than 2m and more than 1.20m 
            elif (self.tracked_blob.linear.z > 0.0012):
                self.obj_pose.target_pose.pose.position.x = 1000*(self.tracked_blob.linear.z - 0.001) # set goal to (distance - 1m)
                self._move_base.send_goal(self.obj_pose) # send goal
                rospy.loginfo('Performing approach to 1m. Pose x: %f, w: %f.' %(self.obj_pose.target_pose.pose.position.x, self.obj_pose.target_pose.pose.orientation.w))
                rospy.loginfo('Waiting for result...')
                self._move_base.wait_for_result()
                if self._move_base.get_state() == GoalStatus.SUCCEEDED:
                    rospy.loginfo('Goal Reached!')
                else:
                    rospy.loginfo('Goal aborted!')
                    
                rospy.loginfo(self._move_base.get_result())
                self.state = FSM_STATES.FSM_OPTIMIZE # loop back to FSM_OPTIMIZE to correct angle
            
            # else if distance is less than 1m 
            else:
                self.obj_pose.target_pose.pose.position.x = 1000*(self.tracked_blob.linear.z) - 0.4 # set goal to final target 
                self._move_base.send_goal(self.obj_pose) # send goal
                rospy.loginfo('Performing final approach. Pose x: %f, w: %f.' %(self.obj_pose.target_pose.pose.position.x, self.obj_pose.target_pose.pose.orientation.w))
                rospy.loginfo('Waiting for result...')
                self._move_base.wait_for_result()
                if self._move_base.get_state() == GoalStatus.SUCCEEDED:
                    rospy.loginfo('Goal Reached!')
                else:
                    rospy.loginfo('Goal aborted!')
                self.moveStartTime = rospy.Time.now()
                self.state = FSM_STATES.FSM_SPIN # move to FSM_SPIN
            


        # FSM_SPIN STATE
        # =============>
        elif (self.state == FSM_STATES.FSM_SPIN):
            
            # Spin     
            if (rospy.Time.now() < self.moveStartTime + self.moveDuration):
                self.speed.linear.x = 0.0
                self.speed.angular.z = 0.5
                self.state = FSM_STATES.FSM_SPIN 
            else:
                self.state = FSM_STATES.FSM_WAIT_FOR_DROP 
        


        # FSM_WAIT_FOR_DROP STATE
        # =============>
        elif (self.state == FSM_STATES.FSM_WAIT_FOR_DROP):
        
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0     
        

            # Next state transitions
            if (self.arm_state.data == 'OBJ_DROPPED'): # if arm signifies that object has been dropped
                self.state = FSM_STATES.FSM_MOVE_TO_BASE # change to MOVE_TO_BASE state
            else: # if object not dropped
                self.state = FSM_STATES.FSM_WAIT_FOR_DROP # loop back

        
        # FSM_MOVE_TO_BASE STATE
        # =============>
        elif (self.state == FSM_STATES.FSM_MOVE_TO_BASE):
            
             # **TEST** wait for coordinates of base to be passed and then move to base
            if (self._move_base.simple_state == 2):
                rospy.loginfo("Waiting for base coordinates...")
                self.state = FSM_STATES.FSM_MOVE_TO_BASE
            else: # if a goal is detected
                rospy.loginfo("Moving back to base")
                rospy.loginfo("Goal Status: %f" %self._move_base.get_state())
                self._move_base.wait_for_result()
                if self._move_base.get_state() == GoalStatus.SUCCEEDED:
                    rospy.loginfo('Goal Reached!')
                else:
                    rospy.loginfo('Goal aborted!')
                self.state = FSM_STATES.FSM_NAVIGATE


        # Print general info
        self.printINFO()

        # Publish /arm_bot_base/state
        self.state_pub.publish(String(FSM_STATES.__geitem__(self.state)))           

        # Only publish velocity when no goal is being pursued or expected        
        if (self.state != FSM_STATES.FSM_NAVIGATE and self.state != FSM_STATES.FSM_MOVE_TO_BASE and self.state != FSM_STATES.FSM_APPROACH_OBJ):        
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


    def printINFO(self):
        
        # Print debug info about linear and angular speeds        
        rospy.loginfo("speedPID: %f" %(self.speed.linear.x))
        rospy.loginfo("anglePID: %f" %(self.speed.angular.z))
        if(self.blobs_detected):
            rospy.loginfo("Blob distance: %f" %(self.tracked_blob.linear.z))
            rospy.loginfo("miss_counter = %f" %(self.miss_counter))            
        else:
            rospy.loginfo("No blobs detected")
            rospy.loginfo("miss_counter = %f" %(self.miss_counter))
       
 
    def spin(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.speed_callback() 
            rate.sleep()	

   
        
        

if __name__ == '__main__':
    rospy.init_node("arm_bot_base")
    
    # Get params from launch file, or else set to default value
    desired_z = rospy.get_param('bin_bot_base/desired_z', 0.8) #0.001	#0.8
    KpLin = rospy.get_param('bin_bot_base/KpLin', 0.5) #1000.00
    KdLin = rospy.get_param('bin_bot_base/KdLin', 0.2) #1000.00
    KpRot = rospy.get_param('bin_bot_base/KpRot', 1.0)
    KdRot = rospy.get_param('bin_bot_base/KdRot', 0.4)
    tol_z = rospy.get_param('bin_bot_base/tol_z', 0.05) #0.0005
    tol_ang= rospy.get_param('bin_bot_base/tol_ang', 0.05)
    maxLinSpeed = rospy.get_param('bin_bot_base/maxLinSpeed', 0.4)
    maxRotSpeed = rospy.get_param('bin_bot_base/maxRotSpeed', 0.5)
    max_z = rospy.get_param('bin_bot_base/max_z', 3.0)
    min_z= rospy.get_param('bin_bot_base/min_z', 0.2) #0.0005    
    try:
        my_bin_bot_base = bin_bot_base(desired_z, KpLin, KdLin, KpRot, KdRot, tol_z, tol_ang, maxLinSpeed, maxRotSpeed, max_z, min_z)
        my_bin_bot_base.spin()
    except rospy.ROSInterruptException: pass
