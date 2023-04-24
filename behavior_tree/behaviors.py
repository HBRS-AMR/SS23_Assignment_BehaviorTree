#! /usr/bin/env python

import py_trees as pt
import py_trees_ros as ptr
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
import numpy as np

class rotate(pt.behaviour.Behaviour):

    """
    Rotates the robot about z-axis 
    """

    def __init__(self, name="rotate platform", topic_name="/cmd_vel", direction=1, max_ang_vel=0.1):
        rospy.loginfo("[ROTATE] initialising rotate behavior")

        # Set up topic name to publish rotation commands
        self.topic_name = topic_name

        # Set up Maximum allowable rotational velocity
        self.max_ang_vel = max_ang_vel # units: rad/sec

        # Set up direction of rotation
        self.direction = direction

        # Execution checker
        self.sent_goal = False

        # become a behaviour
        super(rotate, self).__init__(name)

    def setup(self, timeout):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        rospy.loginfo("[ROTATE] setting up rotate behavior")
        self.cmd_vel_pub = rospy.Publisher(self.topic_name, Twist, queue_size = 2)
        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Rotating the robot at maximum allowed angular velocity in a given direction, 
        where, if **direction** is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        """
        rospy.loginfo("[ROTATE] update: updating rotate behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        

        # Send the rotation command to self.topic_name in this method using the message type Twist()
        
        ## YOUR CODE HERE ##

        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        rospy.loginfo("[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.angular.z = 0
                    
        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)

class stop_motion(pt.behaviour.Behaviour):

    """
    Stops the robot when it is controlled using joystick or by cmd_vel command
    """

    def __init__(self, name="stop platform", topic_name1="/cmd_vel", topic_name2="/joy"):
        rospy.loginfo("[STOP] initialising stopping behavior")

        # Set up topic name to publish rotation commands
        self.cmd_vel_topic = topic_name1
        self.joy_topic = topic_name2

        # become a behaviour
        super(stop_motion, self).__init__(name)

    def setup(self, timeout):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        rospy.loginfo("[ROTATE] setting up rotate behavior")
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size = 2)
        self.joy_pub = rospy.Publisher(self.joy_topic, Joy, queue_size = 2)
        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Rotating the robot at maximum allowed angular velocity in a given direction, 
        where if _direction_ is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        """
        rospy.loginfo("[STOP] update: updating stop behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # Send the zero rotation command to self.cmd_vel_topic and self.joy_pub in this method 
        # using the message type Twist() and Joy() respectively. The frame_id for Joy() message is
        # "/dev/input/js0"
        
        ## YOUR CODE HERE ##

        
        return pt.common.Status.SUCCESS


    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        rospy.loginfo("[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.angular.z = 0
                    
        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)

class battery_status2bb(ptr.subscribers.ToBlackboard):

    """
    Checking battery status
    """
    def __init__(self,  name, topic_name="/mileage", threshold=30.0):
        rospy.loginfo("[BATTERY] initialising battery_status2bb")
        super(battery_status2bb, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=Float32,
                                           blackboard_variables={'battery': 'data'},
                                           initialise_variables={'battery':0.0},
                                           clearing_policy=pt.common.ClearingPolicy.NEVER  # to dictate when data should be cleared/reset.
                                           )
        self.blackboard = pt.blackboard.Blackboard()
        self.blackboard.battery_low_warning = False   # decision making
        self.threshold = threshold

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Call the parent to write the raw data to the blackboard and then check against the
        threshold to determine if the low warning flag should also be updated.
        """
        rospy.loginfo('[BATTERY] update: running batter_status2bb update')
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(battery_status2bb, self).update()

        # check battery voltage level. By comparing with threshold value, update the value of 
        # self.blackboad.battery_low_warning

        ## YOUR CODE HERE ##

class laser_scan_filtered2bb(ptr.subscribers.ToBlackboard):

    """
    Checking filtered laser_scan to avoid possible collison
    """
    def __init__(self, name, topic_name="/scan_filtered", safe_range=0.25):
        rospy.loginfo("[LASER SCAN] initialising laser_scan_filtered2bb")
        super(laser_scan_filtered2bb, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=LaserScan,
                                           blackboard_variables={'laser_scan':'ranges'},
                                           clearing_policy=pt.common.ClearingPolicy.NEVER  # to dictate when data should be cleared/reset.
                                           )
        self.blackboard = pt.blackboard.Blackboard()
        self.blackboard.collison_warning = False   # decision making
        self.safe_min_range = safe_range
        self.blackboard.point_at_min_dist = 0.0

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Call the parent to write the raw data to the blackboard and then check against the
        threshold to set the warning if the robot is close to any obstacle.
        """
        rospy.loginfo("[LASER SCAN] update: running laser_scan_filtered2bb update")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(laser_scan_filtered2bb, self).update()


        # based on the closeness of laser scan data, update the value of self.blackboard.collison_warning
        
        ## YOUR CODE HERE ##   

