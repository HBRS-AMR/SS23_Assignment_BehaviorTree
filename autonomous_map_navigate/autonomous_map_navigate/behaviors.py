#!/usr/bin/env python3

import py_trees as pt
import py_trees_ros as ptr
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import time
import os, signal
import subprocess
from ament_index_python.packages import get_package_share_directory

class launchNodes(pt.behaviour.Behaviour):
    """
    to run launch files
    """
    def __init__(self, name="launch nodes", pkg="slam_toolbox", launch_file=None, mapping_time_out=30, mapping_dist_thresh=0.2, map_name='map'):
        super(launchNodes, self).__init__(name)

        self.name = launch_file
        self.pkg = pkg
        self.launch_file= launch_file
        self.mapping_time_out = mapping_time_out
        self.mapping_dist_thresh = mapping_dist_thresh
        self.map_name = map_name

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="mapping_status", access=pt.common.Access.WRITE)
        self.blackboard.register_key(key="map_name", access=pt.common.Access.WRITE)
        self.blackboard.set("mapping_status", "START")
        self.blackboard.register_key(key="localisation_status", access=pt.common.Access.WRITE)
        self.bringup_dir = get_package_share_directory('slam_toolbox')
        self.map_path = self.bringup_dir+'/maps/'+self.map_name
        
    def setup(self, **kwargs):
        """
        Set up things that should be setup only for one time and which generally might 
        require time to prevent delay in tree initialisation
        """
        info = "[LAUNCH "+self.name+"] setup"
        self.logger.info(info)

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.feedback_message = "setup"
        return True          

    def update(self):
        """
        Primary function of the behavior is implemented in this method
        """
        info = "[LAUNCH "+self.name+"] update"
        self.logger.info(info)
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.launch_file=="online_async_launch.py":
            if self.blackboard.get("mapping_status")=="START":
                self.map_name = self.blackboard.set("map_name", self.map_name)
                self.mapping_start_time = time.time()
                self.mapping_process = subprocess.Popen(['ros2', 'launch', 'slam_toolbox', self.launch_file])                  
                self.blackboard.set("mapping_status", "RUNNING")
                info = "[LAUNCH "+self.name+"] update: started"
                self.logger.info(info)
                return pt.common.Status.RUNNING
            elif self.blackboard.get("mapping_status")=="RUNNING":
                # current_coordinate = np.array([self.blackboard.odom_data.position.x, self.blackboard.odom_data.position.y])
                # dist_from_start_coord = np.linalg.norm(self.blackboard.start_coordinate-current_coordinate)
                current_time = time.time()
                time_passed = current_time-self.mapping_start_time
                if time_passed > self.mapping_time_out:
                    self.logger.info("[SAVE MAP] saving map")
                    subprocess.run(['ros2', 'run', 'nav2_map_server','map_saver_cli','-f', self.map_path, '--ros-args', '-p', 'save_map_timeout:=10000'])
                    self.blackboard.set("mapping_status", "SAVED")
                    self.blackboard.set("localisation_status","START")
                    # terminating the mapping node
                    # self.mapping_process.terminate()
                    mappinng_process_name = "slam_toolbox"
                    for line in os.popen("ps ax | grep " + mappinng_process_name + " | grep -v grep"): # grep -v grep is used to remove the grep process which is also listed
                        fields = line.split()
                        # getting the PID of the node running as a process
                        pid = fields[0]
                        # terminating the process
                        os.kill(int(pid), signal.SIGKILL)                    
                    return pt.common.Status.SUCCESS
                return pt.common.Status.RUNNING

        elif self.launch_file=="localization_launch.py":
            self.map_name = self.blackboard.get("map_name")
            if self.blackboard.get("localisation_status")=="START":
                self.localisation_process = subprocess.Popen(['ros2','launch','slam_toolbox', self.launch_file, 'map_name:='+self.map_name])
                self.blackboard.set("localisation_status", "RUNNING")
                info = "[LAUNCH "+self.name+"] update: started"
                self.logger.info(info)

        return pt.common.Status.SUCCESS

class rotate(pt.behaviour.Behaviour):

    """
    Rotates the robot about z-axis 
    """

    def __init__(self, name="rotate platform", topic_name="/cmd_vel", direction=1, max_ang_vel=1.0):

        # self.logger.info("[ROTATE] initialising rotate behavior")

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

    def setup(self, **kwargs):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        self.logger.info("[ROTATE] setting up rotate behavior")
        
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.topic_name,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Rotating the robot at maximum allowed angular velocity in a given direction, 
        where, if **direction** is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        """
        self.logger.info("[ROTATE] update: updating rotate behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # Send the rotation command to self.topic_name in this method using the message type Twist()
        
        ## YOUR CODE HERE ##
        
        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
                    
        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)

class stop_motion(pt.behaviour.Behaviour):

    """
    Stops the robot when it is controlled using joystick or by cmd_vel command
    """

    def __init__(self, 
                 name: str="stop platform", 
                 topic_name1: str="/cmd_vel", 
                 topic_name2: str="/joy"):
        super(stop_motion, self).__init__(name)
        # Set up topic name to publish rotation commands
        self.cmd_vel_topic = topic_name1
        self.joy_topic = topic_name2

    def setup(self, **kwargs):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        self.logger.info("[STOP MOTION] setting up stop motion behavior")

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.cmd_vel_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        # Create publisher to override joystick commands
        self.joy_pub = self.node.create_publisher(
            msg_type=Joy,
            topic=self.joy_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )
        
        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Rotating the robot at maximum allowed angular velocity in a given direction, 
        where if _direction_ is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        """
        self.logger.info("[STOP] update: updating stop behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        """
        Send the zero rotation command to self.cmd_vel_topic
        """
        
        ## YOUR CODE HERE ##

        """
        sending self.joy_pub in this method. The frame_id for Joy() message is
        "/dev/input/js0". It is similar to just pressing the deadman button on the joystick.
        Nothing to implement here.
        """
        ## Uncomment the following lines to publish Joy() message when running on the robot ##
        # joyMessage = Joy()
        # joyMessage.header.frame_id = "/dev/input/js0"
        # joyMessage.axes = [0, 0, 0, 0, 0, 0]
        # joyMessage.buttons = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
        # self.joy_pub.publish(joyMessage)        

        return pt.common.Status.SUCCESS


    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
                    
        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)

class battery_status2bb(ptr.subscribers.ToBlackboard):

    """
    Checking battery status
    """
    def __init__(self, 
                 topic_name: str="/battery_voltage",
                 name: str=pt.common.Name.AUTO_GENERATED, 
                 threshold: float=30.0):
        super().__init__(name=name,
                        topic_name=topic_name,
                        topic_type=Float32,
                        blackboard_variables={'battery': 'data'},
                        initialise_variables={'battery': 0.0},
                        clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                        qos_profile=ptr.utilities.qos_profile_unlatched()
                        )
        self.blackboard.register_key(
            key='battery_low_warning',
            access=pt.common.Access.WRITE
        ) 
        self.blackboard.battery_low_warning = False   # decision making
        self.threshold = threshold

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Call the parent to write the raw data to the blackboard and then check against the
        threshold to determine if the low warning flag should also be updated.
        """
        self.logger.info('[BATTERY] update: running batter_status2bb update')
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(battery_status2bb, self).update()
        
        """
        check battery voltage level stored in self.blackboard.battery. By comparing with threshold value, update the value of 
        self.blackboad.battery_low_warning
        """

        ## YOUR CODE HERE ##
        
        return status

class laser_scan_2bb(ptr.subscribers.ToBlackboard):

    """
    Checking laser_scan to avoid possible collison
    """
    def __init__(self, 
                 topic_name: str="/scan",
                 name: str=pt.common.Name.AUTO_GENERATED, 
                 safe_range: float=0.25):
        super().__init__(name=name,
                        topic_name=topic_name,
                        topic_type=LaserScan,
                        blackboard_variables={'laser_scan':'ranges'},
                        clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                        # qos_profile=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
                        qos_profile=QoSProfile(
                                    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                    depth=10
                                )
                        )
        self.blackboard.register_key(
            key='collison_warning',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='point_at_min_dist',
            access=pt.common.Access.WRITE
        )
        self.blackboard.collison_warning = False   # decision making
        self.safe_min_range = safe_range
        self.blackboard.point_at_min_dist = 0.0

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Call the parent to write the raw data to the blackboard and then check against the
        threshold to set the warning if the robot is close to any obstacle.
        """
        self.logger.info("[LASER SCAN] update: running laser_scan_2bb update")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(laser_scan_2bb, self).update()

        """
        Based on the closeness of laser scan points (any of them) to robot, update the value of self.blackboard.collison_warning.
        Assign the minimum value of laser scan to self.blackboard.point_at_min_dist. 
        Note: The min() function can be used to find the minimum value in a list.
        """

        ## YOUR CODE HERE ##   

        return status
